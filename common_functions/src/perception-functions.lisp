(in-package :comf)

;;@author Torge Olliges
(defun get-confident-objects(perception-msg)
 (roslisp::with-fields (detectiondata) perception-msg
  (delete-if 
   (lambda (object) 
    (roslisp::with-fields (confidence_class confidence_shape confidence_color) object 
     ;;(roslisp:ros-info (confident) "~%class: = ~F~% shape = ~F~% color: ~F" confidence_class confidence_shape confidence_color) 
     (if (>= confidence_class 0.5) 
         NIL
         (if (>= confidence_shape 0.5) 
           NIL
           (if (>= confidence_color 0.5) 
               NIL
               ;;(print "neither class shape or color confidence is high enough"
               T)))
      )
    )
   detectiondata
   )
   (roslisp:make-msg "SUTURO_PERCEPTION_MSGS/EXTRACTOBJECTINFORESULT"
                     (detectionData) detectionData)
 )
  ;;(roslisp::make-msg )
)
