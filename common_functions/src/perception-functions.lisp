(in-package :comf)

(defvar *objects* NIL)

;;@author Torge Olliges
(defun get-confident-objects (&optional (regions-value (string "robocup_table")))
  (setf *objects* (llif::call-robosherlock-pipeline regions-value))
   (roslisp-msg-protocol:list-to-ros-message
   (delete-if 
    (lambda (obj)
     (roslisp::with-fields 
      (confidence_class) obj 
      (if (> confidence_class 0.6) 
          (roslisp::ros-info 
            (perception-functions) "Object confidence high enough")
          (roslisp::ros-info 
            (perception-functions) "Object confidence to low"))))
      *objects*))
    *objects*)
