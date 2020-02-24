(in-package :comf)


(defvar *goal* nil)
(defvar *classgrasp* nil)
(defvar *dimensions* nil)
(defvar *pose* nil)
(defvar *classplace* nil)
(defvar *list*)

;;@author Jan Schimpf
;;todo add checks for nil;
;;add the text to speech: what class the object we place has,
;; the name of the goal and if the action is done;

(defun place-object (place-list)
    (llif:call-place-action (nth 0 place-list) (nth 1 place-list) (nth 2 place-list)
                            (nth 3 place-list) (nth 4 place-list) (nth 5 place-list)
                            (nth 6 place-list) (nth 9 place-list) (nth 10 place-list)))
  ;;say: done placeing object
  

;;@author Jan Schimpf
;;todo add checks for nil;
;; what class the object we grasp has,
;; the name of the goal and if the action is done;
;; add ways to handle failure and recoveery
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
       )
       ;;say: Grasping object was successful

(defun create-place-list (object-id grasp-pose)
  (setf *dimensions* (llif:prolog-object-dimensions object-id))
  (setf *pose* (llif:prolog-object-pose object-id))

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
     (setf *list* (list (list point-x-object point-y-object point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size_x size_y size_z object-id grasp-pose)
                        (list (+ point-x-object 0.05) point-y-object point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size_x size_y size_z object-id grasp-pose)
                        (list (- point-x-object 0.05) point-y-object point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size_x size_y size_z object-id grasp-pose)
                        (list point-x-object (+ point-y-object 0.05) point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size_x size_y size_z object-id grasp-pose)
                        (list point-x-object (- point-y-object 0.05) point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size_x size_y size_z object-id grasp-pose)
                        ))))
  
