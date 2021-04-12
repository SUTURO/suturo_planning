(in-package :fetch)

(defparameter *spawned-objects-count* 0)
;;@author Torge Olliges
;;spawns detected objects in the bulletworld
(defun spawn-btr-objects (perception-msg)
 (roslisp:with-fields (detectiondata) perception-msg
   (loop for elem across detectiondata do
     (roslisp::with-fields (width height depth shape pose) elem
       (roslisp::with-fields (pose) pose
         (roslisp::with-fields (position) pose
           (roslisp::with-fields (x y z) position
             (print position)
              (case shape
                (0
                    (print (concatenate 'string "spawned object-" (write-to-string *spawned-objects-count*)))
                    (print position)
                    (btr-utils:spawn-object
                        (concatenate 'string "object-" (write-to-string *spawned-objects-count*))
                        :mug
                        :pose (cl-tf::make-pose 
                              (cl-tf::make-3d-vector x y z)
                              (cl-tf::make-identity-rotation)))
                    (setf *spawned-objects-count* (+ *spawned-objects-count* 1)))
                (1 
                  (print position)
                  (print (concatenate 'string "spawned object-" (write-to-string *spawned-objects-count*)))
                  (btr-utils:spawn-object
                        (concatenate 'string "object-" (write-to-string *spawned-objects-count*))
                        :mug
                        :pose (cl-tf::make-pose 
                              (cl-tf::make-3d-vector x y z)
                              (cl-tf::make-identity-rotation)))
                  (setf *spawned-objects-count* (+ *spawned-objects-count* 1)))
                (2 
                  (print position)
                  (print (concatenate 'string "spawned object-" (write-to-string *spawned-objects-count*)))
                  (btr-utils:spawn-object
                        (concatenate 'string "object-" (write-to-string *spawned-objects-count*))
                        :mug
                        :pose (cl-tf::make-pose 
                              (cl-tf::make-3d-vector x y z)
                              (cl-tf::make-identity-rotation)))
                  (setf *spawned-objects-count* (+ *spawned-objects-count* 1)))
                (3 
                  (print position)
                  (print (concatenate 'string "spawned object-" (write-to-string *spawned-objects-count*)))
                  (btr-utils:spawn-object
                        (concatenate 'string "object-" (write-to-string *spawned-objects-count*))
                        :mug
                        :pose (cl-tf::make-pose 
                              (cl-tf::make-3d-vector x y z)
                              (cl-tf::make-identity-rotation)))
                  (setf *spawned-objects-count* (+ *spawned-objects-count* 1)))
              ))))))
 (prolog:prolog '(and (btr:bullet-world ?world)
                      (btr:simulate ?world 10)))))
