(in-package :su-demos)

;;@author Tim Rienits, Felix Krause
(defun clean-the-table-demo ()
  (roslisp-utilities:startup-ros)
  ;;(with-hsr-process-modules
    (unwind-protect
         (progn
           (init-interfaces)
           ;;Calls knowledge to retrieve the percieve pose of the cereal box. Moves to the box immediatly afterwards.
           (let* ((urdf (call-knowledge2 "has_urdf_name"
                                         :param-list (list "tall_table:table:table_front_edge_center")
                                         :result 'object))
                  (pose (call-knowledge "object_rel_pose"
                                        :param-list (list urdf "perceive")
                                        :result 'pose))
                  (frame (first pose))
                  (vector (apply #'cl-tf2::make-3d-vector (second pose)))
                  (rotation (apply #'cl-tf2::make-quaternion (third pose))))
                      
             (move-hsr (cl-tf2::make-pose-stamped frame 0 vector rotation)))

           ;;Detects the cereal box, the cereal box is defined as "muesli". Calls detecting Designator.
           (let* ((?grasp-type 1)
                  (?open-drawer 0)
                  (?source-object-desig
                    (desig:an object
                              (type :muesli)))
                  (?source-perceived-object-desig
                    (exe:perform (desig:an action
                                           (type detecting)
                                           (object ?source-object-desig)))))
             
             ;;Extracts pose from the return value of the detecting Designator.
             (roslisp:with-fields 
                 ((?pose
                   (cram-designators::pose cram-designators:data))) 
                 ?source-perceived-object-desig

               ;;Moves the gripper to the cereal box, implicitly opens the gripper beforehand.
               (exe:perform (desig:a motion
                                     (type :moving-tcp)
                                     (box-pose ?pose)
                                     (grasp-type ?grasp-type)
                                     (collision-mode :allow-all)))

               ;;Function call that closes the gripper.
               ;;(giskard::call-custom-gripper-action :open-gripper 0)

               ;;Takes the cereal box from the table.
               (exe:perform (desig:a motion
                                     (type :moving-tcp)
                                     (knob-pose ?pose)
                                     (open-drawer ?open-drawer)
                                     (collision-mode :allow-all))))))
      (roslisp-utilities:shutdown-ros)))

(defun clean-the-table-demo2()
  ;;(with-hsr-process-modules
  ;;(urdf-proj:with-simulated-robot
    ;; (unwind-protect
    ;;      (progn
  ;;        (roslisp-utilities:startup-ros)

  (park-robot)

  (move-hsr (create-pose (call-knowledge "hsr_pose_dinner_table" :result 'pose)))

  (park-robot)
  
  (let* ((?source-object-desig
           (desig:an object
                     (type :muesli)))
         (?object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?source-object-desig))))
         (?target-pose (create-pose (call-knowledge "object_dest_pose_3" :result 'pose))))
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
    
    (move-hsr (create-pose (call-knowledge "hsr_pose_other_table" :result 'pose)))

    (let ((?object-height 0.28d0))    
      (exe:perform (desig:an action
                             (type :placing)
                             (target-pose ?target-pose)
                             (object-height ?object-height)
                             (collision-mode :allow-all))))

    (move-hsr (create-pose (call-knowledge "hsr_pose_other_table" :result 'pose)))
    
    (park-robot)
    ;; (roslisp-utilities:shutdown-ros))
))


;;@author Tim Rienits
(defun clean-the-table-demo-bw ()
  
  (roslisp-utilities:startup-ros)
   (urdf-proj:with-simulated-robot
           (unwind-protect
                (progn
                  ;;(init-interfaces)
                  (terpri)
                  (format T "!PLAN WIRD GESTARTET!")
                  (terpri)
                  (spawn-pringles-on-table)
                  (sleep 3)
                  (move-in-front-of-pringles)
                  (sleep 3)
                  (pick-up-the-pringles)
                  (sleep 3)

                  (let*((?first-pose (knowledge-get-drawer-pose)))
                    (exe:perform
                     (desig:an action
                               (type going)
                               (target (desig:a location (pose ?first-pose))))))
                  
                  ))))

(defun spawn-pringles-on-table ()
  "Spawn primitive cylinder as :pringles item."
    (btr:add-object btr:*current-bullet-world* :cylinder-item 'cylinder-1
                    '((1.6 -1.4 0.787) (0 0 1 1))
                    :mass 0.2 :size (cl-transforms:make-3d-vector 0.03 0.03 0.08)
                    :item-type :pringles))

(defun move-in-front-of-pringles ()
  "Move in front of les pringles."
  (let* ((vector  (cl-tf2::make-3d-vector 1.0d0 -1.4d0 0d0))
         (rotation (cl-tf2::make-quaternion 0.0 0.0 0.0 1.0)))
    
    (move-hsr (cl-tf2::make-pose-stamped "map" 0 vector rotation))
    ))

(defun pick-up-the-pringles ()
  "Detect and pick up les pringles."
  (let*((?pringles-desig
                    (desig:an object
                              (type :pringles)))
       (?perceived-object-desig
                    (exe:perform (desig:an action
                                           (type detecting)
                                           (object ?pringles-desig)))))
    
    (exe-perform-type :picking-up :arm :left :object ?perceived-object-desig)))

  
