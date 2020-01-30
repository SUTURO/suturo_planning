(in-package :comp)

(defun next-object (from)
    "get the next Object to grasp"
    (setq *objects* (list nil))
    (case (intern (string-upcase from))
        (table (setq *objects* (llif:prolog-table-objects)))
        (shelf (setq *objects* (llif:prolog-all-objects-in-shelf)))
        (pose ()))
    (if (null *objects*) 
        (error 'no-object))
        (first *objects*))