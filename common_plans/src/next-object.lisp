(in-package :comp)

(defun next-object (from)
    "get the next Object to grasp"
    (setq *objects* (list nil))
    (case (intern (string-upcase from))
        (table (setq *objects* (llif:prolog-table-objects)))
        (shelf (setq *objects* (llif:prolog-all-objects-in-shelf)))
        (pose (setq *objects* (llif:prolog-objects-around-pose (get-pose))))
    (if (null *objects*) 
        (error 'no-object))
        (first *objects*)))

(defun get-pose ()
(setf *pose-stamped* (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))
(setf *pose* (cl-transforms:make-pose (cl-tf:origin *pose-stamped*) (cl-tf:orientation *pose-stamped*))))