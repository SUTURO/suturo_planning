(in-package :su-demos)

(defun serve-breakfast-demo()
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
    
    ;; TO BE REMOVED ONCE FIXED
    ;; perception currently publishes two seperate packages of returnvalues.
    ;; The second one is not usable to us, so we need to extract it to clear out the pipeline.
    (exe:perform (desig:an action
                           (type detecting)
                           (object ?source-object-desig)))
    
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
      (let ((?object-size
              (cl-tf2::make-3d-vector 0.04 0.1 0.2)))
        (exe:perform (desig:an action
                               (type picking-up)
                               (object-pose ?pose)
                               (object-size ?object-size)
                               (collision-mode :allow-all)))))

    (park-robot)

    ;; Calls knowledge to receive coordinates of the dinner table pose, then relays that pose to navigation
    (move-hsr (create-pose (call-knowledge "hsr_pose_dinner_table" :result 'pose)))

    ;; places the object by executing the following motions:
    ;; - preparing to place the object, by lifting the arm to an appropriate height
    ;; - placing the object
    ;; - opening the gripper, thus releasing the object
    (let ((?object-height 0.28d0))    
      (exe:perform (desig:an action
                             (type :placing)
                             (target-pose ?target-pose)
                             (object-height ?object-height)
                             (collision-mode :allow-all))))

    ;; Calls knowledge to receive coordinates of the dinner table pose, then relays that pose to navigation
    (move-hsr (create-pose (call-knowledge "hsr_pose_dinner_table" :result 'pose)))

    (park-robot)
))


;; LUCA TODO
;; rewrite~/SUTURO/SUTURO_WSS/planning_ws/src/cram/cram_external_interfaces/cram_giskard/src/collision-scene.lisp to function without using the bulletworld as reasoning tool, but rather use knowledge as reasoning tool. For example "update-object-pose-in-collision-scene" 

;; (cram-occasions-events:on-event
;;      (make-instance 'cram-plan-occasions-events:object-perceived-event
;;                          :object-designator desig
;;                          :perception-source :whatever))


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
