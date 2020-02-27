(in-package :clean)

(defparameter *perception-msg*)
(defparameter *cylinder-counter* 0)
(defparameter *cube-counter* 0)
(defparameter *sphere-counter* 0)

;;@author Torge Olliges
;;spawns detected objects in the bulletworld
(defun spawn-btr-objects ()
  (roslisp:with-fields (detectiondata) *perception-msg*
    (loop for elem across detectiondata do
      (roslisp::with-fields (width height depth shape) elem
                (case shape
                    (0 (setf *cube-counter* (+ *cylinder-counter* 1))
                    (1 (roslisp::ros-info (spawn-btr-objects) "Spawning Cube")
                     (btr:add-object btr:*current-bullet-world* 
                         :primit-cube 'cube-1 '((0.5 0.5 0.05)(0 0 0 1)) 
                         :mass 0.5 :size (cl-tf:make-3d-vector width depth height)))
                    (2 (setf *sphere-counter* (+ *cylinder-counter* 1))
                     (roslisp::ros-info (spawn-btr-objects) "Spawning Sphere")
                     (btr:add-object btr:*current-bullet-world* 
                         :primit-sphere 'sphere-1 '((0.5 0.5 0.05)(0 0 0 1)) 
                         :mass 0.5 :size (cl-tf:make-3d-vector width depth height)))
                    (3 (setf *cylinder-counter* (+ *cylinder-counter* 1))
                     (roslisp::ros-info (spawn-btr-objects) "Spawning Cylinder")
                     (btr:add-object btr:*current-bullet-world* 
                         :primit-cylinder 'cylinder-1 '((0.5 0.5 0.05)(0 0 0 1)) 
                         :mass 0.5 :size (cl-tf:make-3d-vector width depth height))))) 
      )
    )
  )
