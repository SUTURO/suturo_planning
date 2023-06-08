(in-package :su-demos)



(defun start-ros () (roslisp-utilities:startup-ros))

;;@author Felix Krause
(defun storing-groceries-demo-alt ()
  (roslisp-utilities:startup-ros)
  ;;(su-real::with-hsr-process-modules
    (unwind-protect
         (progn
           ;;(init-interfaces)
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


(defun storing-groceries-demo2()
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
         (?target-pose (create-pose (call-knowledge "object_dest_pose_2" :result 'pose))))
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
    
    (move-hsr (create-pose (call-knowledge "hsr_pose_shelf" :result 'pose)))

    (let ((?object-height 0.28d0))    
      (exe:perform (desig:an action
                             (type :placing)
                             (target-pose ?target-pose)
                             (object-height ?object-height)
                             (collision-mode :allow-all))))

    (move-hsr (create-pose (call-knowledge "hsr_pose_shelf" :result 'pose)))
    
    (park-robot)
    ;; (roslisp-utilities:shutdown-ros))
))

(defun knowledge-get-drawer-pose()
  (cl-tf:make-pose-stamped
    "map" 0.0
    (cl-tf:make-3d-vector 0 0.8 0.0)
    (cl-tf:make-quaternion 0 0 1 1)))
              


(defun storing-groceries-demo-bw ()
  
  (roslisp-utilities:startup-ros)
  (urdf-proj:with-simulated-robot
    (spawn-pringles-on-table)
    (sleep 2)
    (move-in-front-of-pringles)
    (sleep 2)
    (pick-up-the-pringles)
    (sleep 2)
    
    
    ;; (btr:add-object btr:*current-bullet-world* :cylinder-item 'cylinder-1
    ;;                 '((-0.7 -0.7 0.85) (0 0 1 1))
    ;;                 :mass 0.2
    ;;                 :size (cl-transforms:make-3d-vector 0.04 0.04 0.09)
    ;;                 :item-type :pringles)
    
    (let*((?first-pose (knowledge-get-drawer-pose)))
    (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location (pose ?first-pose))))))


    ))




(defun storing-groceries-demo-rw ()
  
  (roslisp-utilities:startup-ros)
  ;;(with-hsr-process-modules

    ;,This is where the correct frame goes
    ;; (let* ((urdf (call-knowledge2 "has_urdf_name"
    ;;                                      :param-list (list "left_table:table:table_front_edge_center")
    ;;                                      :result 'object))
    ;;               (pose (call-knowledge "object_rel_pose"
    ;;                                     :param-list (list urdf "perceive")
    ;;                                     :result 'pose))
    ;;               (frame (first pose))
    ;;               (vector (apply #'cl-tf2::make-3d-vector (second pose)))
    ;;               (rotation (apply #'cl-tf2::make-quaternion (third pose))))
                      
    ;;   (move-hsr (cl-tf2::make-pose-stamped frame 0 vector rotation)))

     ;; (let* ((urdf (call-knowledge2 "has_urdf_name"
     ;;                                     :param-list (list "shelf:shelf:shelf_base_center")
     ;;                                     :result 'object))
     ;;              (pose (call-knowledge "object_rel_pose"
     ;;                                    :param-list (list urdf "perceive")
     ;;                                    :result 'pose))
     ;;              (frame (first pose))
    ;;
    ;; (vector (apply #'cl-tf2::make-3d-vector (second pose)))
     ;;              (rotation (apply #'cl-tf2::make-quaternion (third pose))))
                      
     ;;  (move-hsr (cl-tf2::make-pose-stamped frame 0 vector rotation)))
    
    
    ;;Perceive the object
    (let*
        ((?grasp-type 1)
         (?open-drawer 0)
         (?source-object-desig
                    (desig:an object
                              (type :muesli)))
        (?object-desig
             (exe:perform (desig:an action
                                    (type detecting)
                                    (object ?source-object-desig)))))
     ;; Extracts pose from the return value of the detecting Designator.
             (roslisp:with-fields 
                 ((?pose
                   (cram-designators::pose cram-designators:data))) 
                 ?object-desig

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
                                     (collision-mode :allow-all)))))
  ;;)
               (roslisp-utilities:shutdown-ros))



(defun test ()
  
  (with-knowledge-result (result)
      `(and ("has_urdf_name" object ,"left_table:table:table_front_edge_center")
            ("object_rel_pose" object "perceive" result))
    (move-hsr (make-pose-stamped-from-knowledge-result result)))
   
    (let ((?arm :left)
          (?handle-link "iai_kitchen/shelf:shelf:shelf_door_left:handle")
          (?poses  (list (cl-tf:make-pose-stamped
                    "map" 0.0
                    (cl-tf:make-3d-vector 0 0.8 0.0)
                    (cl-tf:make-quaternion 0 0 1 1)))))



      ;(su-real::call-gripper-action 0.1)
      (park-robot)
     ;; (exe:perform (desig:an action
     ;;                        (type :pulling)
     ;;                        (arm ?arm)
     ;;                        (collision-mode :allow-all)
     ;;                        ))

       

       

      
      (exe:perform (desig:a motion
                            (type reaching)
                            (collision-mode :allow-all)
                            (object-name ?handle-link)
                            )
                     )

      ;; (exe:perform (desig:a motion
      ;;                       (type :closing-gripper)))

         ;(su-real::call-gripper-action -0.1)


        ;(cram-giskard::call-environment-manipulation-action :open-or-close :open :arm :left :handle-link ?handle-link :joint-angle -0.5)
        ;(su-real::call-gripper-action 0.1)

      ;; (exe:perform (desig:a motion
      ;;                   (type :retracting)(su-real::with-hsr-process-modules 
      ;;                   (collision-mode :allow-fingers)
      ;;                   (tip-link t)))
       )


  )





(defun test-perception ()

  ;;(park-robot)


  ;; (with-knowledge-result (result)
  ;;     `(and ("has_urdf_name" object ,"left_table:table:table_front_edge_center")
  ;;           ("object_rel_pose" object "perceive" result))
  ;;   (move-hsr (make-pose-stamped-from-knowledge-result result)))


  
  (let* ( (?source-object-desig
           (desig:all object
                     (type :everything)))
         (?object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?source-object-desig)))))
    (print ?object-desig))




  )




         ;; (roslisp:with-fields 
         ;;     ((?pose
         ;;       (cram-designators::pose cram-designators:data))) 
         ;;     ?object-desig
         
      
           ;;Moves the gripper to the cereal box, implicitly opens the gripper beforehand.
           
          ;; (let (;; (?pose (cl-tf2::make-pose-stamped
                 ;;         "map" 0
                 ;;         (cl-tf2::make-3d-vector 1.5990473514373078d0 -0.9011317748330292d0 0.715d0)
                 ;;         (cl-tf2::make-quaternion 0 0 0 1)))
                 ;; (?object-size
                 ;;   (cl-tf2::make-3d-vector 0.08 0.08 0.06)))
      
    ;; (exe:perform (desig:a motion
    ;;                       (type :opening-gripper)))
      
      ;; (exe:perform (desig:a motion
       ;;                      (type reaching)
       ;;                      (collision-mode :allow-hand)
       ;;                      ;(collision-object-b-link "Shelf_OGTVKLRY")
       ;;                      (object-name "Shelf_OGTVKLRY"))
             ;; (exe:perform (desig:an action
             ;;                        (type picking-up)
             ;;                        (handle-link ?handle-link)
             ;;                        (collision-mode :avoid-all)
             ;;                        ()))
             ;;))

    ;;(park-robot)

  ;;))  


  

(defun storing-groceries-demo (number-of-objects)
  (print "Starting Storing Groceries Demo")


  (park-robot)


  ;;Wait for start signal

  ;;Move to shelf
    (with-knowledge-result (result)
      `(and ("has_urdf_name" object "shelf:shelf:shelf_base_center")
            ("object_rel_pose" object "perceive" result))
    (move-hsr (make-pose-stamped-from-knowledge-result result)))

  ;; ;;Open the shelf


  ;; (let ((?handle-link "iai_kitchen/shelf:shelf:shelf_door_right:handle")
  ;;       (?joint-angle 0.4))
    
  ;;   (exe:perform (desig:an action
  ;;                          (type opening-door)
  ;;                          (handle-link ?handle-link)
  ;;                          (joint-angle ?joint-angle)
  ;;                          (tip-link t)
  ;;                          (collision-mode :allow-all))))


  (park-robot)
  

  
  ;;Move to table
  (with-knowledge-result (result)
      `(and ("has_urdf_name" object ,"left_table:table:table_front_edge_center")
            ("object_rel_pose" object "perceive" result))
    (move-hsr (make-pose-stamped-from-knowledge-result result)))



  ;;Perceive and Pick up the Object
  (let* ((?source-object-desig
           (desig:an object
                     (type :pringles)))
         (?object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?source-object-desig)))))
    


    (roslisp:with-fields 
        ((?pose
          (cram-designators::pose cram-designators:data))) 
        ?object-desig

      (let ((?object-size
              (cl-tf2::make-3d-vector 0.06 0.145 0.215)))
        (exe:perform (desig:an action
                               (type picking-up)
                               (object-pose ?pose)
                               (object-size ?object-size)
                               (collision-mode :allow-all)))))


    )



  ;;Move back to the Shelf
  (with-knowledge-result (result)
      `(and ("has_urdf_name" object ,"iai_kitchen/shelf:shelf:shelf_center")
            ("object_rel_pose" object "perceive" result))
    (move-hsr (make-pose-stamped-from-knowledge-result result)))

  (let ((?object-height 0.1)
        (?target-pose (cl-tf:make-pose-stamped
                       "map" 0.0
                       (cl-tf:make-3d-vector 0.0839 1.68 0.725)
                       (cl-tf:make-quaternion 0 0 0 1))))    
    (exe:perform (desig:an action
                           (type :placing)
                           (target-pose ?target-pose)
                           (object-height ?object-height)
                           (collision-mode :allow-all))))





      )
