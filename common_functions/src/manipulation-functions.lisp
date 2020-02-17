(in-package :comf)


(defvar *goal*)
(defvar *classgrasp*)
(defvar *dimensions*)
(defvar *pose*)
(defvar *classplace*)

  
(defun place-object (object-id graspmode)
  (setq *goal* (llif:prolog-object-goal-pose object-id))
  (setq *classplace* (llif:object-name->class object-id))
  ;;say: place object of class *class* at *goal*

  (let ((point-x-goal (nth 0 (nth 0 *pose*)))
        (point-y-goal (nth 1 (nth 0 *pose*)))
        (point-z-goal (nth 2 (nth 0 *pose*)))
        (quaterion-value-1 (nth 0 (nth 1 *pose*)))
        (quaterion-value-2 (nth 1 (nth 1 *pose*)))
        (quaterion-value-3 (nth 2 (nth 1 *pose*)))
        (quaterion-value-4 (nth 3 (nth 1 *pose*))))
    (llif:call-place-action point-x-goal point-z-goal point-z-goal
                            quaterion-value-1 quaterion-value-2
                            quaterion-value-3 quaterion-value-4
                            object-id graspmode))
  ;;say: done placeing object
  )

(defun grasp-object (object-id grasp-pose)
  (setq *dimensions* (llif:prolog-object-dimensions object-id))
  (setq *pose* (llif:prolog-object-pose object-id))
  (setq *classgrasp* (llif:object-name->class object-id))
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
  ;;say: Grasping object was successful
  )
 
