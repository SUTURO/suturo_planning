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
           (print "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
           (case shape
             (0 (print "spawn cylinder 0")
              (print position)
              (btr-utils:spawn-object
                        'cylinder-0 ;;name
                        :primit-cylinder ;;type
                        :color (1 0 0)
                        :pose (cl-tf::make-pose 
                              (cl-tf::make-3d-vector x y z)
                              (cl-tf::make-identity-rotation))))
             (1 (print "spawned cylinder 1")
              (print position)
              (btr-utils:spawn-object
                        'cylinder-1
                        :primit-cylinder
                        :color (1 0 0)
                        :pose (cl-tf::make-pose 
                              (cl-tf::make-3d-vector x y z)
                              (cl-tf::make-identity-rotation))))
             (2 (print "spawn cylinder 2")
              (print position)
              (btr-utils:spawn-object
                        'cylinder-2
                        :primit-cylinder
                        :color (1 0 0)
                        :pose (cl-tf::make-pose 
                              (cl-tf::make-3d-vector x y z)
                              (cl-tf::make-identity-rotation))))
             (3 (print "spawn cylinder 3")
              (print position)
              (btr-utils:spawn-object
                        'cylinder-3
                        :primit-cylinder
                        :color (1 0 0)
                        :pose (cl-tf::make-pose 
                              (cl-tf::make-3d-vector x y z)
                              (cl-tf::make-identity-rotation))))))))))
 (prolog:prolog '(and (btr:bullet-world ?world)
                             (btr:simulate ?world 10)))))

;;(prolog:prolog '(and (btr:bullet-world ?world)
;;                     (assert (btr:object ?world :mesh mug-1 ((0 0 2) (0 0 0 1))
;;                                         :mass 0.2 :color (1 0 0) :mesh :mug))))