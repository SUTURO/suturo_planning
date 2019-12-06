(in-package :plc)

;;; TF ;;;
         
(defparameter *tf-listener* nil)

(defun get-tf-listener ()
  (unless *tf-listener*
    (setf *tf-listener* (make-instance 'cl-tf2:buffer-client))
    (handler-case
     (cl-tf2:lookup-transform *tf-listener* "map" "odom" :timeout 3)
      (CL-TRANSFORMS-STAMPED:TRANSFORM-STAMPED-ERROR () (roslisp:ros-warn (get-tf-listener) "tf-listener takes longer than 20 seconds to get odom in map."))
      (CL-TRANSFORMS-STAMPED:TIMEOUT-ERROR
       () (roslisp:ros-warn (get-tf-listener) "tf-listener takes longer than 20 seconds to get odom in map."))))
  *tf-listener*)

(defun kill-tf-listener ()
  (setf *tf-listener* nil))

(roslisp-utilities:register-ros-cleanup-function kill-tf-listener)

;;; TRANSFORMS ;;;
(defun map-T-odom-pose (pose-map)
  (get-tf-listener)
  (let* ((map-T-odom (cl-tf2:lookup-transform (get-tf-listener) "map" "odom"))
         (pose-odom
           (cl-tf:transform->pose
            (cl-tf:transform*
             (cl-tf:transform-inv map-T-odom)
             (cl-tf:pose->transform pose-map)))))
    pose-odom))

(defun make-pose-stamped (x y euler-z &optional (frame-id "map"))
  (cl-tf:make-pose-stamped
                        frame-id
                        (roslisp::ros-time)
                        (cl-tf:make-3d-vector x y 0.0)
                        (cl-tf:euler->quaternion :ax 0.0 :ay 0.0 :az euler-z)))

;;; CLOSEST OBJECT ;;;
(defun frame-closest-to-robot (objects-list)
  ;; does not involve z-axis calculation
  (if objects-list
      (car (sort objects-list '<
                 :key (alexandria:compose
                       'cl-tf:v-norm
                       (lambda (trans) (cl-tf:copy-3d-vector trans :z 0))
                       'cl-tf:translation
                       (lambda (tf-name)
                         (cl-tf2:lookup-transform (get-tf-listener)
                                                 "base_footprint" tf-name :timeout 5)))))
      (roslisp:ros-warn (closest-object-pose-on-table) "There are no objects to investigate")))




;; (defun check-rotation (nav-goal)
;;   ;; make sure the robot is facing the direction he is driving
;;   (let* ((facing-direction (cl-tf2:lookup-transform
;;                             (plc::get-tf-listener)
;;                             "map"
;;                             "base_footprint"
;;                             :timeout 5))
;;          (rotation-difference
;;            (- (car (last (cl-tf2:quaternion->euler (cl-tf2:rotation facing-direction))))
;;               (car (last (cl-tf::quaternion->euler (cl-tf:orientation nav-goal)))))))
;;             ;; compare base_footprint orientation with current aka. within a margin.

;;     (format t "difference: ~a" rotation-difference)))

;; In progress
(defun force-rotation (pose)
  (let* ((current-pose (cl-tf2:lookup-transform
                            (plc:get-tf-listener)
                            "map"
                            "base_footprint"
                            :timeout 5))
         (goal-pose (cl-tf:make-pose-stamped
                     "map"
                     (roslisp:ros-time)
                     (cl-tf2:translation current-pose)
                     (cl-tf2:orientation pose))))
     goal-pose))

(defun map-T-odom (pose)
  "transfrom the given transform/pose from being relative to map to being
relative to odom"
  (let* ((lookup-pose (cl-tf2::lookup-transform (plc::get-tf-listener) "map" "odom"))
         (make-pose (cl-tf2:make-transform
                     (cl-tf2:translation lookup-pose)
                     (cl-tf2:rotation lookup-pose))))
    
    (cl-tf2:transform*
     (cl-tf2:transform-inv
      make-pose)
     (cl-tf2:pose->transform pose))))

(defun pose-in-shelf (shelf)
  (cl-tf2:lookup-transform (plc:get-tf-listener)
                           "map" (concatenate
                                  'String
                                  "environment/shelf_floor_"
                                  shelf "_piece") :timeout 5))



(defun normalize-euler (rotation)
  "normalizes the euler-angle of the Z rotation of a given transform"
  (let* ((euler-angle (cl-tf:quaternion->euler rotation))
         (z-rot (car (last euler-angle))))
    
    (unless (< (- (/ pi 2)) z-rot (/ pi 2))
      (setf z-rot (- z-rot (* (signum z-rot) pi))))
    z-rot))

(defun transform->grasp-side (tf-frame)
  "Takes a transform from an object in base_footprint and returns :LEFT or :RIGHT, recommending for grasping from the left or the right."

  (let* ((transform (cl-tf:lookup-transform (get-tf-listener)
                                            "environment/table_front_edge_center"
                                            tf-frame :timeout 5))
         (euler-angle (cl-tf:quaternion->euler (cl-tf:rotation transform)))
         (z-rot (nth (1+ (position :AZ euler-angle)) euler-angle))
         (dimensions (lli:prolog-object-dimensions tf-frame)))
    (unless (< (- (/ pi 2)) z-rot (/ pi 2))
      (setf z-rot (- z-rot (* (signum z-rot) pi))))
    (if (> (car (last dimensions)) 0.15)
        "FRONT"
        (if (and (< (abs z-rot) (/ pi 6))
                 (< (car dimensions) 0.13))
            "FRONT"
            "TOP"))))

(defun pose-stamped->transform (pose-stamped)
  (cl-tf2:make-transform
   (cl-tf2:translation pose-stamped)
   (cl-tf2:rotation pose-stamped)))

(defun transform-stamped->pose-stamped(transform)
  (cl-tf:make-pose-stamped
   "map"
   (roslisp:ros-time)
   (cl-tf2:translation transform)
   (cl-tf2:rotation transform)))

(defun transform->pose-stamped (transform)
  (cl-tf:make-pose-stamped
   "map"
   (roslisp:ros-time)
   (cl-tf:translation transform)
   (cl-tf:rotation transform)))

;;;;;;;;;;;;;;; TESTING ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun test-rotation()
  (let* ((object (plc::transform-stamped->pose-stamped
                  (cl-tf2:lookup-transform
                   (plc:get-tf-listener)
                   "map"
                   "environment/shelf_origin"
                   :timeout 5)))
         (base (plc::transform-stamped->pose-stamped
                (cl-tf2:lookup-transform
                 (plc:get-tf-listener)
                 "map"
                 "base_footprint"
                 :timeout 5))))
    (plc::calculate-look-towards-target object base)))

(defun test-pose()
  (plc::pose-stamped->transform
                  (cl-tf2:lookup-transform
                   (plc:get-tf-listener)
                   "environment/table_front_edge_center"
                   "Rewecremefraichecuplr_KHLMVZBT"
                   :timeout 5)))

(defun test-obj ()
  (plc::pose-stamped->transform
                (cl-tf2:lookup-transform
                 (plc:get-tf-listener)
                 "map"
                 "Other_NZDCRPEI"
                 :timeout 5)))

(defun vector->pose-stamped (vector &optional quaternion)
  (cl-tf:make-pose-stamped
   "map"
   (roslisp:ros-time)
   (cl-tf:make-3d-vector (first vector)
                         (second vector)
                         (third vector))
   (if quaternion
       quaternion
   (cl-tf:make-identity-rotation))))

(defun spawn-4-markers (poses-stamped &optional (id 0))
  (let* ((counter id))
    (mapcar (lambda (pose)
              (lli:publish-marker-pose
               pose
               :parent "map"
               :id (setq counter
                         (+ counter 1))))
            poses-stamped))) 
             
(defun vector->transform (vector quaternion)
  (cl-tf:make-transform
   (cl-tf:make-3d-vector (first vector)
                         (second vector)
                         (third vector))
   quaternion))
