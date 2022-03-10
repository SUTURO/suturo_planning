(in-package :comf)
;;TODO: fix indentation similar to high-level/grocery-execute

(defparameter *currentOrigin* Nil)
(defparameter *goalOrigin* Nil)
(defparameter *newgoalOrigin* Nil)
(defparameter *orientation* Nil)
(defparameter *newgoalstamped* Nil)
(defparameter *radians* Nil)
(defparameter *alternatePositions* Nil)
(defparameter *polyCorner1* Nil)
(defparameter *isPoly* Nil)
(defparameter *removed* Nil)
(defvar *pose*)

;;used in go-get-it
;;@author Jan Schimpf
;;gets a list to call a move designator with
;; ((3-d-vector)(quaternion))
(defun get-motion-des-going-for-doors (nav-pose turn)
  (let ((?goal-pose
        (cl-tf2::make-pose-stamped "map" 0
            (cl-tf2::make-3d-vector 
                (nth 0 (nth 0 nav-pose))
                (nth 1 (nth 0 nav-pose))
                (nth 2 (nth 0 nav-pose)))
            (if turn
                 (cl-transforms:q*
                 (cl-tf::make-quaternion
                     (nth 0 (nth 1 nav-pose))
                     (nth 1 (nth 1 nav-pose))
                     (nth 2 (nth 1 nav-pose))
                     (nth 3 (nth 1 nav-pose)))
                     (cl-transforms:euler->quaternion :ax 0 :ay 0 :az (/ pi 2)))
                (cl-tf::make-quaternion
                     (nth 0 (nth 1 nav-pose))
                     (nth 1 (nth 1 nav-pose))
                     (nth 2 (nth 1 nav-pose))
                     (nth 3 (nth 1 nav-pose)))))))
    (exe::perform
     (desig:a motion
              (type going) 
              (pose ?goal-pose))))) 
