(in-package :clean)

;;@author Torge Olliges
;;spawns detected objects in the bulletworld
(defun spawn-btr-objects (perception-msg)
  (roslisp:with-fields (detectiondata) perception-msg
    (loop for elem across detectiondata do
      (roslisp::with-fields (width height depth shape pose) elem
        (roslisp::with-fields (pose) pose
          (roslisp::with-fields (position) pose
            (roslisp::with-fields (x y z) position 
            (case shape
              (0 (print "spawn cylinder 0")
               (print position)
               (btr:add-object btr:*current-bullet-world* 
                         :primit-cylinder 'cylinder-0 
                             (cl-tf::make-pose 
                              (cl-tf::make-3d-vector x y z)
                              (cl-tf::make-identity-rotation))
                         :mass 0.5 :size (cl-tf:make-3d-vector (/ width 2) (/ depth 2) (/ height 2))))
              (1 (print "spawned cylinder 1")
               (print position)
               (print x)
               (btr:add-object btr:*current-bullet-world* 
                         :primit-cylinder 'cylinder-1 
                             (cl-tf::make-pose 
                              (cl-tf::make-3d-vector x y z)
                              (cl-tf::make-identity-rotation))
                         :mass 0.5 :size (cl-tf:make-3d-vector (/ width 2) (/ depth 2) (/ height 2))))
              (2 (print "spawn cylinder 2")
               (print position)
               (btr:add-object btr:*current-bullet-world* 
                         :primit-cylinder 'cylinder-2 
                             (cl-tf::make-pose 
                              (cl-tf::make-3d-vector x y z)
                              (cl-tf::make-identity-rotation))
                         :mass 0.5 :size (cl-tf:make-3d-vector (/ width 2) (/ depth 2) (/ height 2))))
              (3 (print "spawn cylinder 3")
               (print position)
               (btr:add-object btr:*current-bullet-world* 
                         :primit-cylinder 'cylinder-3 
                             (cl-tf::make-pose 
                              (cl-tf::make-3d-vector x y z)
                              (cl-tf::make-identity-rotation))
                         :mass 0.5 :size (cl-tf:make-3d-vector (/ width 2) (/ depth 2) (/ height 2))))
              )
            )
          )
        )
      )
    ) 
  )
  (prolog:prolog '(and (btr:bullet-world ?world)
                              (btr:simulate ?world 10)))
)
