(in-package :clean)
(defparameter *start-time* 0)
;;(defparameter *last-timestamp* 0)
;;(defparameter *last-inner-timestamp* 0)
(defparameter *init-timestamp* 0)
(defparameter *movement-timestamp* 0)
(defparameter *manipulation-timestamp* 0)
(defparameter *detection-timestamp* 0)

(defparameter *total-init-time* 0)
(defparameter *total-movement-time* 0)
(defparameter *total-manipulation-time* 0)
(defparameter *total-detection-time* 0)


;; used in cleanup
;; author Luca Krohm
(defun new-execute-cleanup ()
  (init-cleanup)  
  (comf::with-hsr-process-modules
    (set-timestamp 'manipulation)
    (llif::call-take-pose-action 1)
    (total-time 'manipulation)
    
    (comf::announce-plan-start "clean up")
    (mapc #'cleanup-room (llif::prolog-all-rooms))
    ;;(poi-search)
    (announce-final-time)))

;; used in cleanup
;;@author Luca Krohm
(defun init-cleanup ()
  ;; initialize the interfaces and announce how long it took
  (reset-timestamps)
  (set-timestamp 'start)
  (set-timestamp 'init)
  (init-interfaces)
  (total-time 'init)
  (roslisp::ros-info (execute-cleanup) "Initialisation took ~a seconds"
                     *total-init-time*))

;; used in cleanup
;;@author Luca Krohm
(defun cleanup-room (room)
  ;; get the surfaces in the current room and sort them by distance
  ;; and set them as not visited 
  (mapc #'set-surface-not-visited (comf::sort-surfaces-by-distance
                                   (llif::prolog-room-surfaces
                                    (llif::prolog-current-room))))
  ;; sort all surfaces marked as not visited by distance
  ;; and start cleaning them up one after another
  (mapc #'next-cleanup (comf::sort-surfaces-by-distance
                          (llif::prolog-surfaces-not-visited-in-room
                           room))))

;; used in cleanup
;;@author Luca Krohm
(defun set-surface-not-visited (surface-info)
  (let* ((surface (car surface-info))
         (surface-region-name (llif::prolog-surface-region surface))
         (surface-names '("bin" "tray" "chair" "container" "Shelf" "Floor"))         )
    ;; if non of the surface-names are being found in surface or surface-region-name
    ;; set all surfaces as "not visited"
    (when (and (notany (lambda (n)                       
                         (search n surface))                                
                       surface-names)
               (notany (lambda (n)
                         (search n surface-region-name))
                       surface-names))
      (roslisp::ros-info (execute-cleanup)
                         "Surface set not visited, surface: ~a region name: ~a"
                         surface
                         surface-region-name)
      ;; set all surfaces as "not visited"
      (llif::prolog-set-surface-not-visited (car surface-info)))))

;; used in cleanup
;;@author Luca Krohm
(defun next-cleanup (surface-info)
  (set-timestamp 'movement)
  ;; move robot to surface
  (comf::move-to-surface (car surface-info) t)
  (total-time 'movement)
  ;; perceive the surface
  (comf::perceive-surface (car surface-info))
  ;; if objects were detected, handle them accordingly
  (handle-detected-objects))

;; used in cleanup
;;@author Torge Olliges, Luca Krohm
(defun handle-detected-objects ()
  (let ((next-object (llif::prolog-next-object)))
    ;; if there is not next object, exit "handle-detected-objects
    (unless (or (eq next-object nil)
                (eq next-object 1))
      (print next-object)
      (let ((source-surface (llif::prolog-object-source next-object))
            (target-surface (llif::prolog-object-goal next-object)))
        (print "Source-surface:")
        (print source-surface)
        (print "Target-surface:")
        (print target-surface)
        (cond
          ;; if there is no surface being found, or the surface is floor
          ;; handle the object according to handle-floor-surface
          ((or (eq source-surface nil) (search "Floor" source-surface))
           (handle-floor-surface next-object target-surface))
          ;; otherwise handle it according to handle-surfaces
          (t (handle-surfaces next-object target-surface source-surface)))
        (handle-detected-objects)))))

;; used in cleanup
;;@author Torge Olliges, Luca Krohm
(defun handle-surfaces (next-object target-surface source-surface)
  ;;(comf::announce-movement-to-surface "future" source-surface)
  (roslisp::ros-info (handle-found-objects)
                     "Moving to source surface of ~a namely ~a" next-object source-surface)
  (set-timestamp 'movement)
  ;; move robot to surface where the object was detected
  (comf::move-to-surface source-surface nil)
  (total-time 'movement)
  ;; pick up next-object and deliver it to the target-surface
  (deliver-object next-object target-surface source-surface))

;; used in cleanup
;;@author Torge Olliges, Luca Krohm
(defun handle-floor-surface (next-object target-surface)
  (set-timestamp 'movement)
  (comf::move-hsr
   ;; apparently someone hardcoded the position where the robot should go when handling
   ;; the floor as a surface
   (cl-tf2::make-pose-stamped
    "map"
    0
    (cl-tf2::make-3d-vector 0.3567 0.744 0)
    (cl-tf2::q* (cl-tf2::make-quaternion 0 0 0 1)
                (cl-tf2::euler->quaternion
                 :ax 0
                 :ay 0
                 :az 0.75))))
  (total-time 'movement)
  (deliver-object next-object target-surface))

;; used in cleanup
;; @author Luca Krohm
(defun deliver-object (next-object target-surface &optional source-surface)
  ;;(comf::announce-grasp-action "future" next-object)
  (roslisp::ros-info (handle-found-objects) "Trying to grasp ~a" next-object)
  (comf::grasp-handling next-object)
  (let ((objects-in-gripper (llif::prolog-object-in-gripper)))
    (roslisp::ros-info (deliver-object) "Objects in gripper ~a" objects-in-gripper)
    (cond
      ((eq (length objects-in-gripper) 0)
       (llif::prolog-set-object-handled next-object))
      (t (handle-object-in-gripper next-object target-surface source-surface)))))

;; used in cleanup
;; @author Luca Krohm
(defun handle-object-in-gripper (next-object target-surface &optional source-surface)
  ;;(comf::announce-movement-to-surface "future" target-surface)
  (comf::ros-info (handle-found-objects)
                  "Moving to target surface of ~a namely ~a" next-object target-surface)
  (set-timestamp 'movement)
  (comf::move-to-surface target-surface nil)
  (total-time 'movement)
  ;;(comf::announce-place-action "future" next-object)
  (roslisp::ros-info (handle-found-objects)
                     "Trying to place ~a from ~a on ~a" next-object source-surface target-surface)
  (set-timestamp 'manipulation)
  (llif::call-take-pose-action 6)
  (comf::place-object next-object 1)
  (llif::call-take-pose-action 1)
  (total-time 'manipulation))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; we should outsource the following functions into a different file wich handles the time ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; used in cleanup
;;@author Luca Krohm
(defun announce-final-time ()
  (roslisp::ros-info (execute-cleanup) "Plan execution took ~a seconds"
                      (total-time 'start))
  (roslisp::ros-info (execute-cleanup) "Of that ~a seconds were used for initialisation, ~a seconds were used for detection ~a seconds for movement and ~a seconds for manipulation actions the remaining ~a seconds were used for querying for information from the database"
                     *total-init-time*
                     *total-detection-time*
                     *total-movement-time*
                     *total-manipulation-time*
                     (- (total-time 'start)
                        *total-detection-time*
                        *total-movement-time*
                        *total-manipulation-time*
                        *total-init-time*)))

;; used in cleanup
;; @author Luca Krohm
(defun total-time (type)
  (case type
    (start  (- (get-universal-time) *start-time*))
    (init (calculate-time *init-timestamp* *total-init-time*))
    (movement  (calculate-time *movement-timestamp* *total-movement-time*))
    (manipulation  (calculate-time *manipulation-timestamp* *total-manipulation-time*))
    (detection  (calculate-time *detection-timestamp* *total-detection-time*))))

;; used in cleanup
;;@author Luca Krohm
(defun calculate-time (timestamp total)
  (setf total
        (+
         (- (get-universal-time) timestamp)
         total)))

;; used in cleanup
;;@author Luca Krohm
(defun set-timestamp (type)
  (case type
    (start  (setf *start-time* (get-universal-time)))
    (init  (setf *init-timestamp* (get-universal-time)))
    (movement  (setf *movement-timestamp* (get-universal-time)))
    (manipulation  (setf *manipulation-timestamp* (get-universal-time)))
    (detection  (setf *detection-timestamp* (get-universal-time)))))

;; used in cleanup
(defun reset-timestamps ()
  (setf *start-time* 0)
  ;;(setf *last-timestamp* 0)
  ;;(setf *last-inner-timestamp* 0)
  (setf *init-timestamp* 0)
  (setf *movement-timestamp* 0)
  (setf *manipulation-timestamp* 0)
  (setf *detection-timestamp* 0)
  (setf *total-init-time* 0)
  (setf *total-movement-time* 0)
  (setf *total-manipulation-time* 0)
  (setf *total-detection-time* 0))
