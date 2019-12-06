(in-package :plc)

;; offset fairly close, for perception
(defparameter *x-offset-perception* 0.5)
(defparameter *y-offset-perception* 0.8)

;; bigger offset allowing for space to move arm
(defparameter *x-offset-manipulation* 0.9)
(defparameter *y-offset-manipulation* 1.0)

(defparameter *height-offset* 0.35)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; BASE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun pose-infront-shelf(&optional &key (manipulation NIL) (rotation NIL))
  "Calculates the pose for navigation to go to infront of the shelf.
If the manipulation parameter is set, the distance offset is higher
So that the robot can move his arm safely."
  ;;(lli:publish-challenge-step 5)
  (let* ((shelf (cl-tf2:lookup-transform
                 (plc:get-tf-listener)
                 "map"
                 "environment/shelf_base_center"
                 :timeout 5))
         (pose (cl-tf:make-pose
                (cl-tf:translation shelf)
                (cl-tf:rotation shelf)))
         
         (result-pose (cram-tf:translate-pose pose
                                              :x-offset 0.0
                                              :y-offset (if manipulation
                                                            *y-offset-manipulation*
                                                            *y-offset-perception*)
                                              :z-offset 0.0)))

    (if rotation
        (setq result-pose (cram-tf:rotate-pose (cl-tf:pose->pose-stamped
                                                "map"
                                                (roslisp::ros-time)
                                                result-pose)
                                               :z (case rotation
                                                    (:RIGHT (/ pi 2))
                                                    (:LEFT (/ pi -2))))))                                        
    (cl-tf:make-pose-stamped "map"
                             (roslisp:ros-time)
                             (cl-tf:origin result-pose)
                             (cl-tf:orientation result-pose))))


(defun pose-infront-table(&optional &key (manipulation NIL) (rotation NIL))
  "Calculates the pose for navigation to go to infront of the table.
If the manipulation parameter is set, the distance offset is higher
So that the robot can move his arm safely."
  ;;(lli:publish-challenge-step 2)
  (let* ((table (cl-tf2:lookup-transform
                 (plc:get-tf-listener)
                 "map"
                 "environment/table_front_edge_center"
                 :timeout 5))
         (pose (cl-tf:make-pose
                (cl-tf:translation table)
                (cl-tf:rotation table)))
         
         (result-pose (cram-tf:translate-pose pose
                                              :x-offset (if manipulation
                                                            (+ *x-offset-manipulation*)
                                                            (+ *x-offset-perception*))
                                              :y-offset 0.0
                                              :z-offset 0.0)))
    (if rotation
        (setq result-pose (cram-tf:rotate-pose (cl-tf:pose->pose-stamped
                                                "map"
                                                (roslisp::ros-time)
                                                result-pose)
                                               :z (case rotation
                                                    (:RIGHT (/ pi 2))
                                                    (:LEFT (/ pi -2))))))

    ;;(lli:publish-marker-pose result-pose)
    (cl-tf:make-pose-stamped "map"
                             (roslisp:ros-time)
                             (cl-tf:make-3d-vector
                              (cl-tf:x (cl-tf:origin result-pose))
                              (cl-tf:y (cl-tf:origin result-pose))
                              0.0)
                             (cl-tf:orientation result-pose))))

;;;;;;;;;;;;;;;;;;;;;;;;;;; HEAD ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun table-head-difference( )
  (let* ((table-pos (cl-tf2:lookup-transform
                      (plc:get-tf-listener)
                      "map"
                      "environment/table_front_edge_center"
                      :timeout 5))
         (table-height (cl-tf2:z
                        (cl-tf2:translation table-pos)))
         
         (head-pos (cl-tf2:lookup-transform
                      (plc:get-tf-listener)
                      "map"
                      "head_pan_link"
                      :timeout 5))
         (head-height (cl-tf2:z
                       (cl-tf2:translation head-pos)))
         ;; abs ensures number stays positive
         (diff (- table-height head-height)))
    
    (format t "diff: ~a" (+ diff *height-offset*))
    (+ diff *height-offset*)))


(defun shelf-head-difference ( shelf-level )
  (let* ((shelf-pos (cl-tf2:lookup-transform
                      (plc:get-tf-listener)
                      "map"
                      (concatenate
                       'String
                       "environment/shelf_floor_"
                       shelf-level
                       "_piece")
                      :timeout 5))
         (shelf-height (cl-tf2:z
                        (cl-tf2:translation shelf-pos)))
         
         (head-pos (cl-tf2:lookup-transform
                      (plc:get-tf-listener)
                      "map"
                      "head_pan_link"
                      :timeout 5))
         (head-height (cl-tf2:z
                       (cl-tf2:translation head-pos)))
         ;;TODO FIX THIS HARDCODED 0.2
         (result (/ (- shelf-height 0.2) 1.5)))

    (format t "pre result: ~a" result)
    (if (> result 0.65)
        (setq result  0.65)
        (if (< result 0.35)
            (setq result 0.0)))

    (format t "GOAL POSE ~a" result)
    (format t "HEIGHT of shelf: ~a" shelf-height)
    result))

;;;;;;;;;;;;;;;;;;;;;;;;;; NAVIGATION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; adapted from  cram/cram_pr2/cram_pr2_fetch_deliver_plans/src/fetch-and-deliver-plans.lisp 
(defun calculate-look-towards-target (look-pose-stamped robot-pose-stamped)
  "Given a `look-pose-stamped' and a `robot-pose-stamped',
calculate table-height new robot-pose-stamped, which is rotated with an angle to point towards
the `look-pose-stamped'."
  (let* ((world->robot-transform
           (cram-tf:pose-stamped->transform-stamped robot-pose-stamped "robot"))
         (robot->world-transform
           (cl-transforms:transform-inv world->robot-transform))
         (world->look-pose-origin
           (cl-transforms:origin look-pose-stamped))
         (look-pose-in-robot-frame
           (cl-transforms:transform-point
            robot->world-transform
            world->look-pose-origin))
         (rotation-angle
           (atan
            (cl-transforms:y look-pose-in-robot-frame)
            (cl-transforms:x look-pose-in-robot-frame))))
    (cram-tf:rotate-pose robot-pose-stamped :z rotation-angle)))

(defparameter *arm-offset* 0.4)
(defparameter *short-dist* 0.2)
(defparameter *long-dist* 0.5)
;; use transform of tablecenter-T-obj
(defun calculate-possible-poses-from-obj (object-frame &optional &key
                                                                   (facing-direction :OBJECT)
                                                                   (relative-to :TABLE)
                                                                   (manipulation NIL))
"facing-direction: can be SHELF or TABLE or OBJECT. Determines what the robot will look at.
 relative-to: determines relative to what the calculations are made.
if you want to stand infront of the table and look at an object, this parameter has to be TABLE."
  ;;make list of poses
  (let* ((obj-transform (plc::pose-stamped->transform
                         (cl-tf2:lookup-transform
                          (plc::get-tf-listener)
                          (case relative-to
                            (:TABLE "environment/table_front_edge_center")
                            (:SHELF "environment/shelf_center"))
                          object-frame)))

         (x (cl-tf:x (cl-tf:translation obj-transform)))

         (vector-list '())
         (poses-list '())
         (rotation-pose '())
         (result-pose '())
         (map-T-table (plc::pose-stamped->transform
                       (cl-tf2:lookup-transform
                        (plc::get-tf-listener)
                        "map"
                        (case relative-to
                            (:TABLE "environment/table_front_edge_center")
                            (:SHELF "environment/shelf_center"))))))
    
    (push (list (list (* 1.0 *short-dist*) 0.0 0.0) :x+) vector-list)
    (push (list (list (* -1.0 *short-dist*) 0.0 0.0) :x-) vector-list)
    (push (list (list 0.0 (* 1.0 *short-dist*) 0.0) :y+) vector-list)
    (push (list (list 0.0 (* -1.0 *short-dist*) 0.0) :y-) vector-list)

    ;; make list of transforms which are the offsets
     (mapcar (lambda (vector)
               (push (list (plc::vector->transform
                            (car vector)
                            (cl-tf:make-identity-rotation))
                           (cdr vector)) poses-list)) vector-list)
    
    ;; multiply offset transform onto original object pose
    (mapcar (lambda (transform)
              (setq poses-list (delete transform poses-list :test #'equal))
              (push (list
                     (cl-tf:transform*
                      obj-transform
                      (cl-tf:transform-inv
                       (car transform)))
                     (cdr transform))
                    poses-list))
            poses-list)
  
    ;;cut poses which are far away from edge
    ;;temp is in object frame
    ;; list of transforms
    (setq result-pose (let* ((temp))
                        (mapcar (lambda (pose)
                                  (unless temp
                                    (setq temp pose))
                                  
                                  (if (< (cl-tf:x (cl-tf:translation (car pose)))
                                         (cl-tf:x (cl-tf:translation (car temp))))
                                      (setq temp pose)))
                                poses-list)
                        (list                     
                         (car temp)
                         (cdr temp))
                        temp))
    ;; at this point we have map-T-obj
    ;; this is just for debugging..?
    (setq poses-list (mapcar (lambda (pose)
                                (plc::transform->pose-stamped
                                (cl-tf:transform*
                                 map-T-table
                                 (car pose))))
                             poses-list))
    ;; transform into map
    (setq result-pose (list (cl-tf:transform*
                             map-T-table
                             (car result-pose))
                            (cdr result-pose)))


    
    ;;(plc::spawn-4-markers poses-list)
    ;; works till here. now elongate result-pose
    
    ;; add big offset
    ;; find out offset.
    ;; from table to obj: obj-transform
    (let* ((offset (if manipulation
                       (+ *long-dist* *arm-offset*)
                       *long-dist*))
           (large-offset-transform
             (case (caaaar (cdr result-pose)) ;;TODO make this pretty
               (:x+ (progn (format t "x+ ")
                           (list (* 1.0 (+ offset x)) 0.0 0.0)))
               (:x- (progn (format t "x- ")
                           (list (* -1.0 (+ offset x)) 0.0 0.0)))
               (:y+ (progn (format t "y+ ")
                           (list 0.0 (* 1.0 (+ offset x)) 0.0)))
               (:y- (progn (format t "y- ")
                           (list 0.0 (* -1.0 (+ offset x)) 0.0))))))
      
      (setq large-offset-transform (cl-tf:make-transform
                                    (cl-tf:make-3d-vector (first large-offset-transform)
                                                          (second large-offset-transform)
                                                          (third large-offset-transform))
                                    (cl-tf:make-identity-rotation)))

      (setq result-pose (cl-tf:transform*
                          (car result-pose)
                         ;;(cl-tf:transform-inv  map-T-obj)
                         (cl-tf:transform-inv large-offset-transform))))
     ;;calculate rotation for new pose, so that the robot will face the object.
    (setq rotation-pose
          (let* ((direction (case facing-direction
                              (:TABLE "environment/table_front_edge_center")
                              (:SHELF "environment/shelf_center")
                              (:OBJECT object-frame))))
            (plc::calculate-look-towards-target (plc::transform->pose-stamped
                                                 (plc::pose-stamped->transform
                                                  (cl-tf2:lookup-transform
                                                   (plc::get-tf-listener)
                                                   "map"
                                                   direction)))

             (plc::transform-stamped->pose-stamped
            result-pose))))
    

    
    ;;building the result pose
    (setq result-pose
          (cl-tf:make-pose-stamped
           "map"
           (roslisp:ros-time)
           (cl-tf:make-3d-vector (cl-tf:x (cl-tf:translation result-pose))
                                 (cl-tf:y (cl-tf:translation result-pose))
                                 0.0)
           (cl-tf:orientation rotation-pose)))
    (format t "result pose: ~a" result-pose)

    ;;(planning-communication::publish-marker-pose result-pose :id 100)
    ;;(planning-communication::publish-marker-pose rotation-pose :id 6)
    result-pose
    ))
