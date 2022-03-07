(in-package :eclean)

(defparameter *surface-to-clean* "tall_table:table:table_center")

(defun extended-execute-cleanup ()
  (init-interfaces)
  
  (comf::with-hsr-process-modules
    (llif::call-take-pose-action 1)
    (comf::announce-plan-start "clean up")

    (let ((surface (llif:prolog-surface-from-urdf *surface-to-clean*)))

      (llif::prolog-set-surface-not-visited surface)
      (comf::move-to-surface surface t)
      (comf::perceive-surface surface)
      (handle-detected-objects))))




    
;;@author Torge Olliges
(defun handle-detected-objects ()
  (loop
    do
       (let ((next-object (llif::prolog-next-object)))
         (when (or (eq next-object nil)
                   (eq next-object 1)) (return))
         (print next-object)
         (let ((source-surface (llif::prolog-object-source next-object))
               (target-surface (llif::prolog-temporary-storage-surface))
               )
           (when (or (eq source-surface nil) (search "Floor" source-surface))
             (comf::move-hsr
              (cl-tf2::make-pose-stamped
               "map"
               0
               (cl-tf2::make-3d-vector 0.3567 0.744 0)
               (cl-tf2::q* (cl-tf2::make-quaternion 0 0 0 1)
                           (cl-tf2::euler->quaternion
                            :ax 0
                            :ay 0
                            :az 0.75))))
             (deliver-object next-object target-surface)
             (return-from handle-detected-objects))
           (progn 
             ;;(comf::announce-movement-to-surface "future" source-surface)
             (roslisp::ros-info (handle-found-objects)
                                "Moving to source surface of ~a namely ~a" next-object source-surface)
            
             (comf::move-to-surface source-surface nil))
        
           (deliver-object next-object target-surface source-surface)))))

(defun deliver-object (next-object target-surface &optional source-surface)
  (progn
      ;;(comf::announce-grasp-action "future" next-object)
    (roslisp::ros-info (handle-found-objects) "Trying to grasp ~a" next-object)
    (comf::grasp-handling next-object))
  (let ((objects-in-gripper (llif::prolog-object-in-gripper)))
    (roslisp::ros-info (deliver-object) "Objects in gripper ~a" objects-in-gripper)
    (when (eq (length objects-in-gripper) 0)
      (llif::prolog-set-object-handled next-object)
      (return-from deliver-object)))
  (progn
    ;;(comf::announce-movement-to-surface "future" target-surface)
    (comf::ros-info (handle-found-objects)
                    "Moving to target surface of ~a namely ~a" next-object target-surface)
    (print "0")
    (comf::move-to-surface target-surface nil))
  (progn
    ;;(comf::announce-place-action "future" next-object)
    (roslisp::ros-info (handle-found-objects)
                       "Trying to place ~a from ~a on ~a" next-object source-surface target-surface)
    (llif::call-take-pose-action 6)
    (print "1")
    (comf::place-object next-object 1 :wipe))
  
  (llif::call-take-pose-action 1)
  (llif::call-take-pose-action 1))




