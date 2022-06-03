(in-package :comf)
(defparameter *poi-distance-threshold* 0.75)
(defparameter *grasp-mode* nil)
(defparameter *object-list* nil)
(defparameter *transform-listener* nil)

(defun get-transform-listener ()
  (or *transform-listener*
      (setf *transform-listener* (make-instance 'cl-tf:transform-listener))))

;;;; Navigation ;;;;

;;used in go-get-it
;;@author Torge Olliges    
(defun move-hsr (nav-goal-pose-stamped)
  "Receives pose `nav-goal-pose-stamped'. Lets the robot move to the given pose with a motion designator"
  (let* ((?successfull-pose (try-movement-stamped-list
                             (list nav-goal-pose-stamped))))
    (exe:perform 
     (desig:a motion
              (type going) 
              (pose ?successfull-pose)))))

;;used in go-get-it
;;@author Torge Olliges, Phillip Klein
(defun try-movement-stamped-list (listStamped)
  "Receives a list of stamped poses `listStamped'. Tries out all poses in the bulletworld simulation and returns a possible pose."
  ;;  (car listStamped))
  (let ((?nav-pose listStamped))
    (cpl:with-retry-counters ((going-retry 3))
      (cpl:with-failure-handling
          (((or common-fail:low-level-failure 
                cl::simple-error
                cl::simple-type-error)
               (e)
             (setf ?nav-pose (cdr ?nav-pose))
             (setf listStamped (cdr ?nav-pose))
             (cpl:do-retry going-retry
               (roslisp:ros-warn (going-demo movement-fail)
                                 "~%Failed to move to given position~%")
               (cpl:retry))
             (roslisp:ros-warn  (going-demo movement-fail)
                                "~%No more retries~%")))
        (let ((?actual-nav-pose (car ?nav-pose))) 
          (exe:perform
           (desig:a motion
                    (type going)
                    (pose ?actual-nav-pose)))
          (setf listStamped (cdr ?nav-pose))
          ?actual-nav-pose)))))

;; used in cleanup
;;@author Torge Olliges, @2ndAuthor Luca Krohm
(defun move-to-surface (surface-id turn)
  "Receives surface ID `surface-id' and value `turn'. Moves the robot to surface, ready to perceive"
  (roslisp:ros-info (move-too-room)
                    "Moving to surface ~a"
                    surface-id)
  (let* ((perceive-pose (llif::prolog-pose-to-perceive-surface surface-id))
         (?goal-pose
           (cl-tf::make-pose-stamped
            "map" 0
            (cl-tf::make-3d-vector
             (first (first perceive-pose))
             (second (first perceive-pose))
             (third (first perceive-pose)))
            (cond
              (turn (cl-transforms:q* 
                     (cl-tf::make-quaternion
                      (first (second perceive-pose))
                      (second (second perceive-pose))
                      (third (second perceive-pose))
                      (fourth (second perceive-pose)))
                     (cl-transforms:euler->quaternion :ax 0 :ay 0 :az (/ pi 2))))
              (t (cl-tf::make-quaternion
                  (first (second perceive-pose))
                  (second (second perceive-pose))
                  (third (second perceive-pose))
                  (fourth (second perceive-pose))))))))
    (exe::perform (desig:a motion
                           (type going)
                           (pose ?goal-pose)))))


;; used in cleanup
;;@author Torge Olliges
(defun perceive-surface (surface-id)
  "Receives surface ID `surface-id'. Percieves objects inside a surface-region and filters out confident objects before inserting them into Knowledge."
  ;;(comf::announce-perceive-action-surface "present" surface-id)
  ;; (let ((surface-pose (first (llif::prolog-surface-pose surface-id))))
  ;;   (llif::call-take-gaze-pose-action
  ;;                               :px (first surface-pose)
  ;;                               :py (second surface-pose)
  ;;                               :pz (third surface-pose))
  (llif::call-take-pose-action 2)
  (let* ((surface-region (llif::prolog-surface-region surface-id))
         (perceived-objects (llif::call-robosherlock-object-pipeline
                             (vector surface-region) t))
         (confident-objects (comf::get-confident-objects perceived-objects)))
    ;;(print perceived-objects)
    (print confident-objects)
    (setf *object-list* confident-objects)
    (roslisp::with-fields (detectiondata) confident-objects
      (roslisp::ros-info (poi-search)
                         "Number of objects detected: ~a"
                         (length detectiondata))
      (cond
        ((> (length detectiondata) 0) (llif::insert-knowledge-objects confident-objects)))))
  (llif::prolog-set-surface-visited surface-id)
  ;;(clean::spawn-btr-objects confident-objects))
  (llif::call-take-pose-action 1))


;; used in wipe
;;@author Felix Krause
;; TODO: Save handle into knowledge
(defun perceive-surface-drawer (surface-id)
  "Receives surface ID `surface-id'. Saves the percieved drawer-handle into knowledge."
  (comf::move-to-surface surface-id t)
  (llif::call-take-pose-action 2)
  (let* ((surface-region (llif::prolog-surface-region surface-id))
         (drawer-handle (llif::call-robosherlock-object-pipeline-drawer
                             (vector surface-region) t)))
    (cond
      (drawer-handle
       (roslisp:ros-info (perceive handle)
                         "perceived drawer handle ~a"
                         drawer-handle)
       (print drawer-handle)
       ;; (llif::insert-knowledge-drawer drawer-handle)
       ;; (llif::prolog-set-surface-visited surface-id)
       (llif::call-take-pose-action 1))
      (T (roslisp::ros-error (perceive-surface-drawer) "Message is NIL")))
    drawer-handle))



;; used in cleanup
;;@author Torge Olliges, Luca Krohm
(defun grasp-handling (next-object &optional (fixed nil))
  "Receives next object `next-object'. Tries to grasp next object, retries up to 3 times"
  (dynamic-grasp next-object)
  (when fixed
    (setf *grasp-mode* fixed))
  (roslisp::ros-info (grasp-handling)
                     "Starting grasp handling for object ~a"
                     next-object)
  (let ((grasp-action-result (measure-time :manipulation
                               (comf::grasp-object next-object *grasp-mode*))))
    (print grasp-action-result)
    (roslisp::with-fields (error_code)
        grasp-action-result
      (cond
        ((eq error_code 1) (retry-grasp 3 next-object)))))
   ;;(comf::announce-grasp-action "past" next-object))
  (llif::call-take-pose-action 1))

;; used in cleanup
;;@author Jan Schimpf, Luca Krohm
(defun retry-grasp (times next-object)
  "Receives integer `times' and next object `next-object'. Retries grasping next object for up to 'times' times"
  ;;(comf::announce-grasp-action "past" next-object)
  (cpl:with-retry-counters ((grasp-retries times))
    (cpl:with-failure-handling
        (((or 
           common-fail:low-level-failure 
           cl::simple-error
           cl::simple-type-error)
             (e)
           ;;(comf::announce-grasp-action "failed" next-object)
           (cpl:do-retry grasp-retries
             (roslisp:ros-warn (grasp-handling)
                               "~%Failed to grasp the object~%")
             (cpl:retry))
           (roslisp:ros-warn (grasp-handling)
                             "~%No more retries~%")))
      ;;(dynamic-grasp next-object)  ;;sets the graspmode should be replaced with the function from knowledge when that is finished
      (measure-time :manipulation
        (comf::grasp-object next-object *grasp-mode*)))))

;; used in cleanup
;;@author Jan Schimpf, Luca Krohm
;; Quick solution making use of the second Graspmode
;; Graspmode 1 in case the Object is larger than is higher than 6 cm
;; and Graspmode 2 in case the Object 6cm high or less 
(defun dynamic-grasp (object-id)
  "Receives object ID `object-id'. Chooses a grasp mode depending on the object dimensions"
  (let ((dimensions (third (llif:prolog-object-dimensions object-id))))
    (cond
      ((< 0.06 dimensions) (setf *grasp-mode* 1))
      ((> 0.06 dimensions) (setf *grasp-mode* 2)))))
      ;;((> 0.06 dimensions) (setf *grasp-mode* 3)))))






;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;; Luca technische Präsentation ;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;; @author Luca Krohm
(defun find-object (object surface-id)
  "Receives surface ID `surface-id'. Percieves objects inside a surface-region and filters out sponges before inserting them into Knowledge"
  ;;(comf::announce-perceive-action-surface "present" surface-id)
  ;; (let ((surface-pose (first (llif::prolog-surface-pose surface-id))))
  ;;   (llif::call-take-gaze-pose-action
  ;;                               :px (first surface-pose)
  ;;                               :py (second surface-pose)
  ;;                               :pz (third surface-pose))
  (roslisp::ros-info (find sponge)
                     "Trying to find the sponge")
  (llif::call-take-pose-action 4)
  (let* (;;(surface-region (llif::prolog-surface-region surface-id))
         (surface-region "inner_lower_drawer")
         (perceived-objects (llif::call-robosherlock-object-pipeline
                             (vector surface-region) t))
         (confident-objects perceived-objects));;(comf::get-confident-objects perceived-objects nil object)))
    (unless confident-objects
      (roslisp:ros-info (find-object) "First Attempt failed, retriying")
      (setf confident-objects (llif::call-robosherlock-object-pipeline
                               (vector surface-region) t)))
    (print confident-objects)
    (setf *object-list* confident-objects)
    (roslisp::with-fields (detectiondata) confident-objects
      (roslisp::ros-info (poi-search)
                         "Number of objects detected: ~a"
                         (length detectiondata))
      (cond
        ((> (length detectiondata) 0) (llif::insert-knowledge-objects confident-objects)))))
  (llif::call-take-pose-action 1))
