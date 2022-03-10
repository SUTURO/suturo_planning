(in-package :comf)
(defparameter *poi-distance-threshold* 0.75)
(defparameter *grasp-mode* nil)
(defparameter *object-list* nil)

;;;; Navigation ;;;;

;;used in go-get-it
;;@author Torge Olliges
;;Lets the robot move to the given position with a motion designator    
(defun move-hsr (nav-goal-pose-stamped)
    (let* ((?successfull-pose 
        (try-movement-stamped-list 
				    (list 
             nav-goal-pose-stamped))))
      (exe:perform 
            (desig:a motion
                (type going) 
                    (pose ?successfull-pose)))))

;; used in cleanup
;;@author Torge Olliges
(defun move-to-surface (surface-id turn)
  (roslisp:ros-info (move-too-room) "Moving to surface ~a" surface-id)
  (let* ((perceive-pose (llif::prolog-pose-to-perceive-surface surface-id))
        (?goal-pose
          (cl-tf::make-pose-stamped
            "map" 0
            (cl-tf::make-3d-vector
             (first (first perceive-pose))
             (second (first perceive-pose))
             (third (first perceive-pose)))
            (if turn
                (cl-transforms:q* 
                 (cl-tf::make-quaternion
                  (first (second perceive-pose))
                  (second (second perceive-pose))
                  (third (second perceive-pose))
                  (fourth (second perceive-pose)))
                 (cl-transforms:euler->quaternion :ax 0 :ay 0 :az (/ pi 2)))
                (cl-tf::make-quaternion
                 (first (second perceive-pose))
                 (second (second perceive-pose))
                 (third (second perceive-pose))
                 (fourth (second perceive-pose)))))))
    (exe::perform (desig:a motion
                           (type going)
                           (pose ?goal-pose)))))


;; used in cleanup
;;@author Torge Olliges
(defun perceive-surface (surface-id)
  ;;(comf::announce-perceive-action-surface "present" surface-id)
  (let ((surface-pose (first (llif::prolog-surface-pose surface-id))))
    ;;(llif::call-take-gaze-pose-action
    ;;                             :px (first surface-pose)
    ;;                             :py (second surface-pose)
    ;;                             :pz (third surface-pose))
    (llif::call-take-pose-action 2))
  
  (let* ((surface-region (llif::prolog-surface-region surface-id))
         (perceived-objects
           (llif::call-robosherlock-object-pipeline
            (vector surface-region) t))
         (confident-objects (comf::get-confident-objects perceived-objects)))
    ;;(print perceived-objects)
    (print confident-objects)
    (setf *object-list* confident-objects)
    (roslisp::with-fields (detectiondata)
          confident-objects
        (progn
          (roslisp::ros-info (poi-search) "Number of objects detected: ~a" (length detectiondata))
          (when (> (length detectiondata) 0)
              (llif::insert-knowledge-objects confident-objects)))))
  (llif::prolog-set-surface-visited surface-id)
  ;;(clean::spawn-btr-objects confident-objects))
  (llif::call-take-pose-action 1))

;; used in cleanup
;;@author Torge Olliges
(defun grasp-handling (next-object)
  (dynamic-grasp next-object)
  (roslisp::ros-info (grasp-handling) "Starting grasp handling for object ~a" next-object)
  ;;(setf clean::*last-inner-timestamp* (get-universal-time))
  (let ((grasp-action-result (comf::grasp-object next-object *grasp-mode*)))
    ;;(setf clean::*total-manipulation-time*
    ;;      (+
    ;;       (- (get-universal-time) clean::*last-inner-timestamp*)
    ;;       clean::*total-manipulation-time*))
    (print grasp-action-result)
    (roslisp::with-fields (error_code)
        grasp-action-result
      (if (eq error_code 1)
          ;;(comf::announce-grasp-action "past" next-object)
          (cpl:with-retry-counters ((grasp-retries 3))
            (cpl:with-failure-handling
                (((or 
                   common-fail:low-level-failure 
                   cl::simple-error
                   cl::simple-type-error)
                     (e)
                   ;;(comf::announce-grasp-action "failed"  next-object)
                   (cpl:do-retry grasp-retries
                     (roslisp:ros-warn (grasp-handling) "~%Failed to grasp the object~%")
                     (cpl:retry))
                   (roslisp:ros-warn (grasp-handling) "~%No more retries~%")))
              ;;(dynamic-grasp next-object)  ;;sets the graspmode should be replaced with the function from knowledge when that is finished
              ;;(setf clean::*last-inner-timestamp* (get-universal-time))
              (comf::grasp-object next-object *grasp-mode*)
              ;;(setf clean::*total-manipulation-time*
              ;;      (+
              ;;       (- (get-universal-time) clean::*last-inner-timestamp*)
              ;;clean::*total-manipulation-time*))
              )))))
  (llif::call-take-pose-action 1)
  ;;(comf::announce-grasp-action "past" next-object))
)

;; used in cleanup
;;Author Jan Schimpf
;; Quick solution making use of the second Graspmode
;; Graspmode 1 in case the Object is larger than is higher than 6 cm
;; and Graspmode 2 in case the Object 6cm high or less 
(defun dynamic-grasp (object-id)
  (third (llif:prolog-object-dimensions object-id))
  (if (< 0.06 (third (llif:prolog-object-dimensions object-id)))
      (setf *grasp-mode* 1)
      (setf *grasp-mode* 2)))
