(in-package :clean)

;;@author Torge Olliges
;;spawns detected objects in the bulletworld
(defun spawn-btr-objects
    (mapcar 
        (lambda (object) 
            ((if (roslisp::with-fields object) (detectiondata))) 
    comf::get-confident-objects)))