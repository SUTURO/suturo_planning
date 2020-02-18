(in-package :comf)


(defvar *goal* nil)
(defvar *classgrasp* nil)
(defvar *dimensions* nil)
(defvar *pose* nil)
(defvar *classplace* nil)

;;@author Jan Schimpf
;;todo add checks for nil;
;;add the text to speech: what class the object we place has,
;; the name of the goal and if the action is done;
; add ways to handle failure and recoveery
(defun place-object (object-id graspmode)
  (setq *goal* (llif:prolog-object-goal-pose object-id))
  (setq *classplace* (llif:object-name->class object-id))
  ;;say: place object of class *class* at *goal*

  (cpl:with-retry-counters ((grasping-retry 3))
   (cpl:with-failure-handling
      (((or common-fail:low-level-failure 
                cl::simple-error
         cl::simple-type-error)
       (e)
       (declare (ignore e))
       (cpl:do-retry grasping-retry
         (roslisp:ros-warn (grasp-fail)
                                 "~%Failed to grasp the object~%")
         (cpl:retry))
  
  (let ((point-x-goal (nth 0 (nth 0 *pose*)))
        (point-y-goal (nth 1 (nth 0 *pose*)))
        (point-z-goal (nth 2 (nth 0 *pose*)))
        (quaterion-value-1 (nth 0 (nth 1 *pose*)))
        (quaterion-value-2 (nth 1 (nth 1 *pose*)))
        (quaterion-value-3 (nth 2 (nth 1 *pose*)))
        (quaterion-value-4 (nth 3 (nth 1 *pose*))))
    (llif:call-place-action point-x-goal point-y-goal point-z-goal
                            quaterion-value-1 quaterion-value-2
                            quaterion-value-3 quaterion-value-4
                            object-id graspmode))
  ;;say: done placeing object
  )))))

;;@author Jan Schimpf
;;todo add checks for nil;
;; what class the object we grasp has,
;; the name of the goal and if the action is done;
;; add ways to handle failure and recoveery
(defun grasp-object (object-id grasp-pose)
  (setq *dimensions* (llif:prolog-object-dimensions object-id))
  (setq *pose* (llif:prolog-object-pose object-id))
  (setq *classgrasp* (llif:object-name->class object-id))

  
  (cpl:with-retry-counters ((grasping-retry 3))
   (cpl:with-failure-handling
      (((or common-fail:low-level-failure 
                cl::simple-error
         cl::simple-type-error)
       (e)
       (declare (ignore e))
       (cpl:do-retry grasping-retry
         (roslisp:ros-warn (grasp-fail)
                                 "~%Failed to grasp the object~%")
         (cpl:retry))
       
  ;;say:grasping object of class *class*
 
  (let ((point-x-object (nth 0 (nth 2 *pose*)))
        (point-y-object (nth 1 (nth 2 *pose*)))
        (point-z-object (nth 2 (nth 2 *pose*)))
        (quaterion-value-1 (nth 0 (nth 3 *pose*)))
        (quaterion-value-2 (nth 1 (nth 3 *pose*)))
        (quaterion-value-3 (nth 2 (nth 3 *pose*)))
        (quaterion-value-4 (nth 3 (nth 3 *pose*)))
        (size_x (nth 0 *dimensions*))
        (size_y (nth 1 *dimensions*))
        (size_z (nth 2 *dimensions*)))
    (if (equalp grasp-pose 0)
        (llif:call-grasp-action point-x-object point-y-object point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size_x size_y size_z object-id 0))
    (if (equalp grasp-pose 1)
        (llif:call-grasp-action point-x-object point-y-object point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size_x size_y size_z object-id 1)))
       ))))
       ;;say: Grasping object was successful
)
