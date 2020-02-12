(in-package :comp)

(defvar *table-objects* NIL)
(defvar *shelf-objects* NIL)
(defvar *robot-pose-stamped* NIL)

(defun next-object (from &optional (threshold 3.0))
    "get the next Object to grasp"
    (setq *objects* (list nil))
    ;;(case (intern (string-upcase from))
        ;;(table (setq *table-objects* (llif:prolog-table-objects)))
        ;;(shelf (setq *shelf-objects* (llif:prolog-all-objects-in-shelf)))
        ;;(pose (setq *objects* (llif:prolog-objects-around-pose (get-pose) threshold)))
    ;;(if (null *objects*) 
        ;;(error 'no-object))
  ;;(first *objects*)))
  (setf *table-objects* (llif:prolog-table-objects))
  (first *table-objects*))
       

;;(defun get-pose ()
;;(setf *robot-pose-stamped* (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))
;;(setf *pose* (cl-transforms:make-pose (cl-tf:origin q*robot-pose-stamped*) (cl-tf:orientation *robot-pose-stamped*))))
