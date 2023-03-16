(in-package :su-demos)

(defun serve-breakfast-demo()
  ;;(with-hsr-process-modules
  ;;(urdf-proj:with-simulated-robot
    ;; (unwind-protect
    ;;      (progn
  ;;        (roslisp-utilities:startup-ros)

  (park-robot)
    
  (move-hsr (create-pose (call-knowledge "hsr_pose_shelf" :result 'pose)))

  (park-robot)

  (let* ((?source-object-desig
           (desig:an object
                     (type :muesli)))
         ;; detect object and safe the return value
         (?object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?source-object-desig))))
         (?target-pose (create-pose (call-knowledge "object_dest_pose_1" :result 'pose))))
    (exe:perform (desig:an action
                           (type detecting)
                           (object ?source-object-desig)))
    
    ;; Extracts pose from the return value of the detecting Designator.
    (roslisp:with-fields 
        ((?pose
          (cram-designators::pose cram-designators:data))) 
        ?object-desig

      
      ;;Moves the gripper to the cereal box, implicitly opens the gripper beforehand.
      (let ((?object-size
              (cl-tf2::make-3d-vector 0.04 0.1 0.2)))
        (exe:perform (desig:an action
                               (type picking-up)
                               (object-pose ?pose)
                               (object-size ?object-size)
                               (collision-mode :allow-all)))))

    (park-robot)
    
    (move-hsr (create-pose (call-knowledge "hsr_pose_dinner_table" :result 'pose)))

    (let ((?object-height 0.28d0))    
      (exe:perform (desig:an action
                             (type :placing)
                             (target-pose ?target-pose)
                             (object-height ?object-height)
                             (collision-mode :allow-all))))

    (park-robot)
    ;; (roslisp-utilities:shutdown-ros))
))



;; (cram-occasions-events:on-event
;;      (make-instance 'cram-plan-occasions-events:object-perceived-event
;;                          :object-designator desig
;;                          :perception-source :whatever))
