(in-package :comf)

;;@author Torge Olliges
(defun get-confident-objects(perception-msg, &optional (threshold 0.5))
    (roslisp::with-fields (detectiondata) perception-msg
        (delete-if 
            (lambda (object) 
            (roslisp::with-fields (confidence_class confidence_shape confidence_color) object 
                ;;TODO: why is this commented out? does it not work? if it doesnt fix it and uncomment
                ;;(roslisp:ros-info (confident) "~%class: = ~F~% shape = ~F~% color: ~F" confidence_class confidence_shape confidence_color) 
                (if (>= confidence_class threshold) 
                  NIL
                  (if (>= confidence_shape threshold) 
                      NIL
                      (if (>= confidence_color threshold) 
                          NIL
                      T)))))
            detectiondata)
        (roslisp:make-msg "SUTURO_PERCEPTION_MSGS/EXTRACTOBJECTINFORESULT"
            (detectionData) detectionData)))
