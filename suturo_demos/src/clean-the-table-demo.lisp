(in-package :su-demos)

(defun clean-the-table-demo ()

   (park-robot)

   (move-hsr (get-table-pos))
  
   (let* ((?source-object-desig
           (desig:an object
                     (type :bowl)))
         (?object-desig
           (exe:perform (desig:all action
                                  (type detecting)
                                  (object ?source-object-desig))))
          
          (?target-pose (get-target-pos)))
     
     (print ?object-desig)

      (roslisp:with-fields 
              ((?pose
                (cram-designators::pose cram-designators:data))) 
              ?object-desig

            (let ((?object-size
                    (cl-tf2::make-3d-vector 0.16 0.06 0.215))) ;; TODO: BOWL SIZE
              (exe:perform (desig:an action
                                     (type picking-up)
                                     (object-pose ?pose)
                                     (object-size ?object-size)
                                     (collision-mode :allow-all)))))

      (park-robot)

     (move-hsr (get-dishwasher-pos))

      (let ((?object-height 0.215d0)) ;; TODO: BOWL HEIGHT, SEE BOWL SIZE ABOVE
            (exe:perform (desig:an action
                                   (type :placing)
                                   (target-pose ?target-pose)
                                   (object-height ?object-height)
                                   (collision-mode :allow-all))))
     
     )
  )



;; (defun get-table-pos ()
;;   (cl-tf2::make-pose-stamped
;;    "map" 0
;;    (cl-tf2::make-3d-vector 3.8 8.4 0)
;;    (cl-tf2::make-quaternion 0 0 0 1)))

;; (defun get-dishwasher-pos ()
;;   (cl-tf2::make-pose-stamped
;;    "map" 0
;;    (cl-tf2::make-3d-vector 4.7 8.9 0)
;;    (cl-tf2::make-quaternion 0 0 0 1)))

;; (defun get-target-pos ()
;;   (cl-tf2::make-pose-stamped
;;    "map" 0
;;    (cl-tf2::make-3d-vector 4.65 8.25 0.8)
;;    (cl-tf2::make-quaternion 0 0 0 1)))

;;@author Tim Rienits, Felix Krause
(defun clean-the-table-demo-old ()
  
  (park-robot)

  (let* ((?source-object-desig
           (desig:an object
                     (type :bowl)))
         (?object-desig
           (exe:perform (desig:all action
                                  (type detecting)
                                  (object ?source-object-desig)))))
    (print ?object-desig))
  

  (roslisp:with-fields 
              ((?pose
                (cram-designators::pose cram-designators:data))) 
              ?object-desig
            
            ;; picks up the object by executing the following motions:
            ;; - opening the gripper
            ;; - reaching for the object
            ;; - closing the gripper, thus gripping the object
            ;; - lifting the object
            ;; - retracting the arm to retrieve the object from, for example, a shelf
            ;;(call-text-to-speech-action "Picking up the object Cereal-Box")
            (let ((?object-size
                    (cl-tf2::make-3d-vector 0.16 0.06 0.215)))
              (exe:perform (desig:an action
                                     (type picking-up)
                                     (object-type ?object-type)
                                     (object-pose ?pose)
                                     (object-size ?object-size)
                                     (collision-mode :allow-all)))))
 )

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
                 ;; (init-interfaces)
                  (terpri)
                  (format T "!PLAN WIRD GESTARTET!")
                  (terpri)
                  (spawn-pringles-on-table)
                  (sleep 3)
                  (move-in-front-of-pringles)
                  (sleep 3)
                  (pick-up-the-pringles)
                  (sleep 3)

                  ;;(let*((?first-pose (knowledge-get-drawer-pose)))
                  ;;  (exe:perform
                  ;;   (desig:an action
                  ;;             (type going)
                  ;;             (target (desig:a location (pose ?first-pose))))))
                  
                  ))))

(defun spawn-pringles-on-table ()
  "Spawn primitive cylinder as :pringles item."
    (btr:add-object btr:*current-bullet-world* :cylinder-item 'cylinder-1
                    '((1.6 -1.4 0.987) (0 0 1 1))
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
    
    ;;(exe-perform-type :picking-up :arm :left :object ?perceived-object-desig)

     (roslisp:with-fields 
                 ((?pose
                   (cram-designators::pose cram-designators:data))) 
                 ?perceived-object-desig

       ;;Moves the gripper to the cereal box, implicitly opens the gripper beforehand.

       (let ((?superpose (list ?pose)))
        (mapc
         (lambda (?current-left-approach-poses)
           (let ((?poses `(,?current-left-approach-poses)))
             (exe:perform
              (desig:an action
                  (type poking)
                  (left-poses ?poses)
                  (collision-mode :allow-all)))))
         ?superpose))
       
              ;; (exe:perform (desig:an action
              ;;                       (type approaching)
                                     ;;(box-pose ?pose)
                                     ;;(grasp-type 0)
              ;;                       (left-poses ?pose)
              ;;                       (collision-mode :allow-all)
              ;;                       ))

               ;;Function call that closes the gripper.
               ;;(giskard::call-custom-gripper-action :open-gripper 0)

               ;;Takes the cereal box from the table.
               )


   ;; (exe-perform-type :reaching :arm :left :object ?pringles-desig)
    ))

  
