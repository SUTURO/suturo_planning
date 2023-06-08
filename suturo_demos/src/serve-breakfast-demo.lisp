(in-package :su-demos)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; LUCA TODO
;; rewrite~/SUTURO/SUTURO_WSS/planning_ws/src/cram/cram_external_interfaces/cram_giskard/src/collision-scene.lisp to function without using the bulletworld as reasoning tool, but rather use knowledge as reasoning tool. For example "update-object-pose-in-collision-scene"

;; Rewrite or duplicate and change the following functions (in order to preserve the original implementation in case its vital to other plans):
;; make-giskard-environment-request, uses btr in on the very bottom

;; reset-collision-scene

;; update-object-pose-in-collision-scene

;; add-object-to-collision-scene

;; detach-object-in-collision-scene

;; attach-object-to-arm-in-collision-scene

;; full-update-collision-scene

;; (cram-occasions-events:on-event
;;      (make-instance 'cram-plan-occasions-events:object-perceived-event
;;                          :object-designator desig
;;                          :perception-source :whatever))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defparameter *objects* '(:muesli));; :milk :spoon :bowl))

(defun serve-breakfast-demo()
  ;;(call-text-to-speech-action "Starting demo")
  
  (park-robot)

  ;; (call-text-to-speech-action "Positioning in front of shelf")
  ;; Calls knowledge to receive coordinates of the shelf pose, then relays that pose to navigation
  (move-hsr (get-shelf-pos)) ;;(create-pose (call-knowledge "hsr_pose_shelf" :result 'pose)))

  
  
  (park-robot)

  (let ((?handle-link "iai_kitchen/shelf:shelf:shelf_door_left:handle")
        (?joint-angle -1.2))

    ;;(call-text-to-speech-action "Opening shelf door")
    (exe:perform (desig:an action
                           (type opening-door)
                           (handle-link ?handle-link)
                           (joint-angle ?joint-angle)
                           (tip-link t)
                           (collision-mode :allow-all))))

  (park-robot)

  (move-hsr (get-shelf-pos))

  (park-robot)

  ;;(call-text-to-speech-action "Trying to detect the object Cereal-Box")
  (mapc #'(lambda (?object-type)
        (let* ((?source-object-desig
                 (desig:an object
                           (type ?object-type)))
               ;; detect object and safe the return value
               (?object-desig
                 (exe:perform (desig:an action
                                        (type detecting)
                                        (object ?source-object-desig))))
               (?target-pose (get-target-pos)))

          ;;(call-text-to-speech-action "Found object Cereal-Box")
          ;; Extracts pose from the return value of the detecting Designator.
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
          (park-robot)

          ;;(call-text-to-speech-action "Moving to target location")
          ;; Calls knowledge to receive coordinates of the dinner table pose, then relays that pose to navigation
          (move-hsr (get-table-pos)) ;; (create-pose (call-knowledge "hsr_pose_dinner_table" :result 'pose)))
          
          ;; places the object by executing the following motions:
          ;; - preparing to place the object, by lifting the arm to an appropriate height
          ;; - placing the object
          ;; - opening the gripper, thus releasing the object
          (let ((?object-height 0.215d0))
            ;;(call-text-to-speech-action "Placing object Cereal-Box")
            (exe:perform (desig:an action
                                   (type :placing)
                                   (target-pose ?target-pose)
                                   (object-height ?object-height)
                                   (collision-mode :allow-all))))
          
          ;; Calls knowledge to receive coordinates of the dinner table pose, then relays that pose to navigation
          (move-hsr (get-table-pos)) ;; (create-pose (call-knowledge "hsr_pose_dinner_table" :result 'pose)))
          
          (park-robot)))
        *objects*)
  
  ;; Pouring corn from bowl in the popcorn pot
  ;; from popcorn demo
  ;; (pour-into :popcorn-pot '(:right) :right-side)
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;; Hardcoded stuff for debugging ;;;;;;;;;;;;

(defun park-robot ()
  "Default pose"
  (call-take-pose-action 0 0 0 0 0 1.5 -1.5 0))

(defun take-new-default1 ()
  "Potential alternatives to the default pose"
  (call-take-pose-action 0 0 0 0.3 -2.6 0 1 0))

(defun take-new-default2 ()
  "Potential alternatives to the default pose"
  (call-take-pose-action 0 0 0 0.3 -2.6 1.5 -1.5 0.5))

(defun nav-zero-pos ()
  "Starting pose in IAI office lab"
  (let ((vector (cl-tf2::make-3d-vector 0 0 0))
        (rotation (cl-tf2::make-quaternion 0 0 0 1)))
    (move-hsr (cl-tf2::make-pose-stamped "map" 0 vector rotation))))

(defun get-shelf-pos ()
  (cl-tf2::make-pose-stamped
   "map" 0
   (cl-tf2::make-3d-vector 0.01 0.95 0)
   (cl-tf2::make-quaternion 0 0 1 1)))

(defun get-table-pos ()
  (cl-tf2::make-pose-stamped
   "map" 0
   (cl-tf2::make-3d-vector 0.7 -0.95 0)
   (cl-tf2::make-quaternion 0 0 0 1)))

(defun get-target-pos ()
  (cl-tf2::make-pose-stamped
   "map" 0
   (cl-tf2::make-3d-vector 1.5 -0.95 0.7)
   (cl-tf2::make-quaternion 0 0 0 1)))
        
(defun serve-breakfast-demo-alt()
 "Demo for the 'serve breakfast' challenge for the first stage of the robocup. Full plan will look roughly as follows:
- move to shelf
- open shelf
- perceive items in shelf
- pick up relevant items and transport them towards dinner table one after another
- pick up cerealbox
- pour cereals into the breakfast bowl
- place down cerealbox"
  
  ;; Takes standard pose
  (park-robot)

  ;; Calls knowledge to receive coordinates of the shelf pose, then relays that pose to navigation
  (move-hsr (get-shelf-pos)) ;;(create-pose (call-knowledge "hsr_pose_shelf" :result 'pose)))

  (park-robot)

  (let ((?handle-link "iai_kitchen/shelf:shelf:shelf_door_left:handle")
        (?joint-angle -1.1))
    
    (exe:perform (desig:an action
                           (type opening-door)
                           (handle-link ?handle-link)
                           (joint-angle ?joint-angle)
                           (tip-link t)
                           (collision-mode :allow-all))))

  (park-robot)

  (move-hsr (get-shelf-pos))

  (park-robot)
    

  (let* ((?object-type :muesli)
         (?source-object-desig
           (desig:an object
                     (type ?object-type)))
         ;; detect object and safe the return value
         (?object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?source-object-desig))))
         (?target-pose (get-target-pos)));;(create-pose (call-knowledge "object_dest_pose_1" :result 'pose))))
    
    ;; Extracts pose from the return value of the detecting Designator.
    (roslisp:with-fields 
        ((?pose
          (cram-designators::pose cram-designators:data))) 
        ?object-desig
;;       ;;(giskard::add-object-to-collision-scene2 :muesli-1)
;;       ;; (coe:on-event (make-instance 'cram-plan-occasions-events:object-perceived-event
;;       ;;                    :object-designator ?object-desig
;;       ;;                    :perception-source :whatever))
      
;;       ;; picks up the object by executing the following motions:
;;       ;; - opening the gripper
;;       ;; - reaching for the object
;;       ;; - closing the gripper, thus gripping the object
;;       ;; - lifting the object
;;       ;; - retracting the arm to retrieve the object from, for example, a shelf
      (let ((?object-size
              (cl-tf2::make-3d-vector 0.06 0.145 0.215)))
        (exe:perform (desig:an action
                               (type picking-up)
                               (object-type ?object-type)
                               (object-pose ?pose)
                               (object-size ?object-size)
                               (collision-mode :allow-all)))))

    (park-robot)

    ;; Calls knowledge to receive coordinates of the dinner table pose, then relays that pose to navigation
    (move-hsr (get-table-pos)) ;; (create-pose (call-knowledge "hsr_pose_dinner_table" :result 'pose)))

    ;; places the object by executing the following motions:
    ;; - preparing to place the object, by lifting the arm to an appropriate height
    ;; - placing the object
    ;; - opening the gripper, thus releasing the object
    (let ((?object-height 0.215d0))    
      (exe:perform (desig:an action
                             (type :placing)
                             (target-pose ?target-pose)
                             (object-height ?object-height)
                             (collision-mode :allow-all))))

    ;; Calls knowledge to receive coordinates of the dinner table pose, then relays that pose to navigation
    (move-hsr (get-table-pos));; (create-pose (call-knowledge "hsr_pose_dinner_table" :result 'pose)))

    (park-robot)))


    
  
(defun pouring-test ()
  (let* ((?source-object-desig
           (desig:an object
                     (type bowl)))
         (?object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?source-object-desig))))
         (?object-size1 (cl-tf2::make-3d-vector 0.09 0.09 0.05))
         (?object-size2 (cl-tf2::make-3d-vector 0.06 0.16 0.215))
         (?new-origin (cl-transforms:make-3d-vector
                       (/ (+ (cl-transforms:x ?object-size1)
                             (cl-transforms:x ?object-size2))
                          2)
                       0
                       (/ (+ (cl-transforms:z ?object-size1)
                             (cl-transforms:z ?object-size2))
                          2)))
         (?object-transform (man-int::get-object-transform ?object-desig))
         (?temp-transform (cl-tf2::make-pose-stamped
                           "base_footprint" 0
                           ?new-origin
                           (cl-tf2::make-quaternion 0 0 0 1)))
         (?reach-transform (cram-tf:apply-transform
                            (cl-tf:lookup-transform cram-tf:*transformer* "map" "base_footprint")
                            (cram-tf:apply-transform ?object-transform
                                                    (cram-tf:pose-stamped->transform-stamped
                                                     ?temp-transform
                                                     "base_footprint"))))
         (?reach-pose (cram-tf:transform->pose-stamped
                       "map" 0
                       ?reach-transform)))
    ?reach-pose))



;; Idea:
;; Change Reaching to approaching to generalize that kind of motion.
;; Planning also give the "context" to manipulation, so manipulation can differentiate
;; between for example, picking up and pouring
