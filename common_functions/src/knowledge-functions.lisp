(in-package :comf)

(defvar *objects* NIL)
(defvar *robot-pose-stamped* NIL)
(defvar *robot-transform* NIL)

(defun next-object (from)
    "get the next Object to grasp"
    (setq *objects* (list nil))
     (case (intern (string-upcase from))
        (table (setq *objects* (llif:prolog-table-objects)))
        (shelf (setq *objects* (llif:prolog-all-objects-in-shelf)))
        ;;(pose (setq *objects* (llif:prolog-objects-around-pose (get-pose) threshold)))
        ;;TODO: use new prolog query to knowledge)
    (if (null *objects*) 
        (error 'no-object)))
    (first *objects*))
       

(defun get-pose ()
  ;;get robot position
  (setf *robot-transform* 
        (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))
  ;;create robot pose
  (setf *robot-pose-stamped* (cl-transforms:make-pose 
                (cl-tf:origin *robot-pose-stamped*) 
                (cl-tf:orientation *robot-pose-stamped*))))
