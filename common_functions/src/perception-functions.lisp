(in-package :comf)

;; used in cleanup
;;@author Torge Olliges, Tom-Eric Lehmkuhl, Luca Krohm
(defun get-confident-objects(perception-msg &optional (threshold 0.0) (class nil))
  "Receives perception message `perception-msg' and an optional threshold `threshold'. Filters for confident objects"
  (and perception-msg
       (roslisp::with-fields (detectiondata) perception-msg
         (roslisp::ros-info (filtering)
                     "trying to filter ~a from ~a"
                     class
                     detectiondata)
         (roslisp::make-msg
          "SUTURO_PERCEPTION_MSGS/EXTRACTOBJECTINFORESULT"
          (detectionData)
          (delete-if 
           (lambda (object) 
             (roslisp::with-fields
                 (obj_class confidence_class confidence_shape confidence_color) object 
               (cond
                 ((>= confidence_class threshold) NIL)
                 ((>= confidence_shape threshold) NIL)
                 ((>= confidence_color threshold) NIL)
                 ;;;;Have a look at how to make this filter work
                 ;;((search class obj_class) NIL)
                 ;;((search "lego_duplo" obj_class) NIL)
                 (t T))))
           detectiondata)))))
