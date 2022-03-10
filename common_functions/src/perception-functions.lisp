(in-package :comf)

;; used in cleanup
;;@author Torge Olliges, Tom-Eric Lehmkuhl
(defun get-confident-objects(perception-msg &optional (threshold 0.0))
  (roslisp::with-fields (detectiondata) perception-msg
    (roslisp::make-msg
     "SUTURO_PERCEPTION_MSGS/EXTRACTOBJECTINFORESULT"
     (detectionData)
     (delete-if 
      (lambda (object) 
        (roslisp::with-fields
            (confidence_class confidence_shape confidence_color) object 
          (if (>= confidence_class threshold) 
              NIL
              (if (>= confidence_shape threshold) 
                  NIL
                  (if (>= confidence_color threshold) 
                      NIL
                      T)))))
     detectiondata))))
