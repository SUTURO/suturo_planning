(in-package :clean)

;;@author Torge Olliges
;;spawns detected objects in the bulletworld
(defun spawn-btr-objects ()
    (mapcar 
        (lambda (object) 
            (roslisp::with-fields (width height depth shape) object
                (case shape
                    (0 (btr:add-object btr:*current-bullet-world* 
                         :primit-cylinder 'cylinder-1 '((0.5 0.5 0.05)(0 0 0 1)) 
                         :mass 0.5 :size (cl-tf:make-3d-vector width depth height)))
                    (1 (btr:add-object btr:*current-bullet-world* 
                         :primit-cube 'cylinder-1 '((0.5 0.5 0.05)(0 0 0 1)) 
                         :mass 0.5 :size (cl-tf:make-3d-vector width depth height)))
                    (2 (btr:add-object btr:*current-bullet-world* 
                         :primit-sphere 'cylinder-1 '((0.5 0.5 0.05)(0 0 0 1)) 
                         :mass 0.5 :size (cl-tf:make-3d-vector width depth height)))
                    (3 (btr:add-object btr:*current-bullet-world* 
                         :primit-cylinder 'cylinder-1 '((0.5 0.5 0.05)(0 0 0 1)) 
                         :mass 0.5 :size (cl-tf:make-3d-vector width depth height))))) 
          )))