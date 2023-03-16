(in-package :su-demos)

;;@author Felix Krause, Tim Rienits
(defun storing-groceries-demo ()
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






     



;; ;;All of the following is Designator/PM stuff.
;; (defmacro with-hsr-process-modules (&body body)
;;   "Receives a body of lisp code `body'. Runs the code contained in `body' with all the necessary process modules"
;;   `(cram-process-modules:with-process-modules-running
;;        (hsr-navigation
;;         giskard::giskard-pm
;;         common-desig:wait-pm
;;         rk:robokudo-perception-pm)
;;      (cpl-impl::named-top-level (:name :top-level),@body)))


;; ;;Process module itself
;; (cram-process-modules:def-process-module hsr-navigation (motion-designator)
;;   "Receives motion-designator `motion-designator'. Calls the process module HSR-NAVIGATION with the appropriate designator."
;;   (destructuring-bind (command argument &rest args)
;;       (desig:reference motion-designator)
;;     (declare (ignore args))
;;     (ecase command
;;       (cram-common-designators:move-base
;;        (su-demos::call-nav-action-ps argument)))));;change package in the future

;; ;;Denotes the PM as avaivailable
;; (cram-prolog:def-fact-group available-hsr-process-modules (cpm:available-process-module
;;                                                            cpm:matching-process-module)

;;   (cram-prolog:<- (cpm:available-process-module hsr-navigation))
  
;;   (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-navigation)
;;     (desig:desig-prop ?desig (:type :going))))


;; ;;Designator inference rules
;; (cram-prolog:def-fact-group hsr-motion-designators (desig:motion-grounding)

;;   (cram-prolog:<- (desig:motion-grounding ?designator (going ?pose))
;;     (desig:desig-prop ?designator (:type :going))
;;     (desig:desig-prop ?designator (:target ?pose)))

;;   (cram-prolog:<- (desig:motion-grounding ?designator (going goal-pose))
;;     (desig:desig-prop ?designator (:type :going))
;;     (desig:desig-prop ?designator (:x ?x))
;;     (desig:desig-prop ?designator (:y ?y))
;;     (desig:desig-prop ?designator (:angle ?angle))))

;; ;; ------
;; (prolog:def-fact-group giskard-pm (cpm:matching-process-module
;;                                    cpm:available-process-module)

;;   (prolog:<- (cpm:matching-process-module ?motion-designator giskard:giskard-pm)
;;     (or (desig:desig-prop ?motion-designator (:type :moving-tcp))
;;         (desig:desig-prop ?motion-designator (:type :moving-gripper-joint))
;;         (desig:desig-prop ?motion-designator (:type :moving-arm-joints))
;;         (desig:desig-prop ?motion-designator (:type :pulling))
;;         (desig:desig-prop ?motion-designator (:type :pushing))
;;         (desig:desig-prop ?motion-designator (:type :going))
;;         (desig:desig-prop ?motion-designator (:type :moving-torso))
;;         (desig:desig-prop ?motion-designator (:type :moving-custom))
;;         (desig:desig-prop ?motion-designator (:type :looking))
;;         (desig:desig-prop ?motion-designator (:type :closing-gripper))
;;         (desig:desig-prop ?motion-designator (:type :opening-gripper))))

;;   (prolog:<- (cpm:available-process-module giskard:giskard-pm)
;;     (prolog:not (cpm:projection-running ?_))))

