(in-package :comf)

;; used in cleanup
;;@author Jan Schimpf, Felix Krause
(defun place-object (object-id grasp-pose &optional (mode :cleanup) (distance 0) )
  "Receives object ID `object-id', grasp pose `grasp-pose' and a mode `mode' which is :cleanup (by default), :wipe or :wipe-return. Constructs a place motion-designator"
  ;;get the information from knowledge
  (let* ((object-dimensions (llif:prolog-object-dimensions object-id))
         (prolog-goal-pose (ccase mode
                             (:cleanup (llif:prolog-object-goal-pose object-id))
                             (:wipe (llif:prolog-temporary-storage-pose object-id))
                             (:wipe-return (llif::prolog-source-pose object-id))))
         (object-pose (list (first prolog-goal-pose)
                            (second prolog-goal-pose))))
    (roslisp::ros-info (place-object)
                       "Placing at ~a"
                       (car object-pose))
    ;;takes apart the messages for the needed information to consturct the place motion-designator 
    (let* ((?point-x-object (first (first object-pose)))
           (?point-y-object (second (first object-pose)))
           (?point-z-object (+ (third (first object-pose)) distance))
           (?quaterion-value-1 (first (second object-pose)))
           (?quaterion-value-2 (second (second object-pose)))
           (?quaterion-value-3 (third (second object-pose)))
           (?quaterion-value-4 (fourth (second object-pose)))
           (?size-x (first object-dimensions))
           (?size-y (second object-dimensions))       
           (?size-z (third object-dimensions))
           (?object-id object-id)
           (?grasp-mode grasp-pose)
           ;; construct the motion designator
           (place (desig:a motion
                           (:type :placing)
                           (:point-x ?point-x-object)
                           (:point-y ?point-y-object)
                           (:point-z ?point-z-object)
                           (:quaterion-value-1 ?quaterion-value-1)
                           (:quaterion-value-2 ?quaterion-value-2)
                           (:quaterion-value-3 ?quaterion-value-3)
                           (:quaterion-value-4 ?quaterion-value-4)
                           (:size-x ?size-x)
                           (:size-y ?size-y)
                           (:size-z ?size-z)
                           (:object-id ?object-id)
                           (:grasp-mode ?grasp-mode))))
      (ros-info (place-object)
                "Placing with ~a"
                place)
      (print place)
      ;; executes the motion designator for place
      (exe:perform place))))

;; used in cleanup
;;@author Jan Schimpf
(defun grasp-object (object-id grasp-pose)
  "Receives object ID `object-id' and grasp pose `grasp-pose'. Constructs a grasp motion-designator"
  ;;get the information from knowledge
  (let* ((object-dimensions (llif:prolog-object-dimensions object-id))
         (raw-object-pose (llif:prolog-object-pose object-id))
         (object-pose (list (third raw-object-pose)
                            (fourth raw-object-pose))))
    (print "object-id:")
    (print object-id)
    (print "object-dimensions:")
    (print object-dimensions)
    (print "raw-object-pose:")
    (print raw-object-pose)
    (print "object-pose:")
    (print object-pose)
    (roslisp::ros-info (grasp-object)
                       "Grasping at ~a"
                       (car object-pose))
    ;;takes apart the messages for the needed information to consturct the grasp motion-designator 
    (let* ((?point-x-object (first (first object-pose)))
           (?point-y-object (second (first object-pose)))
           (?point-z-object (third (first object-pose)))
           (?quaterion-value-1 (first (second object-pose)))
           (?quaterion-value-2 (second (second object-pose)))
           (?quaterion-value-3 (third (second object-pose)))
           (?quaterion-value-4 (fourth (second object-pose)))
           (?size-x (first object-dimensions))
           (?size-y (second object-dimensions))
           (?size-z (third object-dimensions))
           (?object-id object-id)
           (?grasp-mode grasp-pose)
           ;; construct the motion designator      
           (grasp
             (desig:a motion
                      (:type :grasping)
                      (:point-x ?point-x-object)
                      (:point-y ?point-y-object)
                      (:point-z ?point-z-object)
                      (:quaterion-value-1 ?quaterion-value-1)
                      (:quaterion-value-2 ?quaterion-value-2)
                      (:quaterion-value-3 ?quaterion-value-3)
                      (:quaterion-value-4 ?quaterion-value-4)
                      (:size-x ?size-x)
                      (:size-y ?size-y)
                      (:size-z ?size-z)
                      (:object-id ?object-id)
                      (:grasp-mode ?grasp-mode))))
      ;; executes the motion designator for grasp
      (exe:perform grasp))))

;;@author Felix Krause
(defun perform-wipe-motion (surface)
  "Receives urdf-surface `surface'. Calls PERFORM-WIPE-POSE with appropriate values for the wipe plan."
  (perform-wipe-pose surface -0.9 0.88 0.1)
  (perform-wipe-pose surface -0.9 -0.88 0.1)
  (perform-wipe-pose surface -0.7 -1 0.1)
  (perform-wipe-pose surface -0.7 1 0.1)
  (perform-wipe-pose surface -0.4 1 0.1)
  (perform-wipe-pose surface -0.4 -1 0.1))

;;@author Felix Krause
(defun perform-wipe-pose (surface x y zdistance)
  "Receives urdf-surface `surface' and three float values `x',`y' `zdistance'. Constructs a wipe motion-designator."
  (let* ((prolog-goal-pose (llif::prolog-surface-rel-pose surface x y zdistance))
         (object-pose (list (first prolog-goal-pose)
                            (second prolog-goal-pose)))) 
    (let* ((?point-x (first (first object-pose)))
           (?point-y (second (first object-pose)))
           (?point-z (third (first object-pose)))
           (?quaterion-value-1 (first (second object-pose)))
           (?quaterion-value-2 (second (second object-pose)))
           (?quaterion-value-3 (third (second object-pose)))
           (?quaterion-value-4 (fourth (second object-pose)))
           (wipe (desig:a motion
                           (:type :wiping)
                           (:point-x ?point-x)
                           (:point-y ?point-y)
                           (:point-z ?point-z)
                           (:quaterion-value-1 ?quaterion-value-1)
                           (:quaterion-value-2 ?quaterion-value-2)
                           (:quaterion-value-3 ?quaterion-value-3)
                           (:quaterion-value-4 ?quaterion-value-4))))
      (ros-info (wipe-surface)
                " ~a"
                wipe)
      (exe:perform wipe))))


;;@author Felix Krause
(defun perform-drawer-pose (perception-message ?mode ?drawer)
  "Receives a the pose of a drawerhandle `perception-message', a mode `?mode',
which determines whether you want to open or close the drawer and the drawer `?drawer'.
Constructs a openclose motion-designator to open/close the drawer."
  (llif::call-take-pose-action 2)
  (roslisp:with-fields
    ((?point-x (x position pose pose detectiondata ))			
     (?point-y (y position pose pose detectiondata ))
     (?point-z (z position pose pose detectiondata ))
     (?quaternion-value-1 (x orientation pose pose detectiondata))
     (?quaternion-value-2 (y orientation pose pose detectiondata))
     (?quaternion-value-3 (z orientation pose pose detectiondata))
     (?quaternion-value-4 (w orientation pose pose detectiondata)))
    perception-message
   
    (let ((open-close (desig:a motion
                              (:type :openclose)
                              (:point-x ?point-x)
                              (:point-y ?point-y)
                              (:point-z ?point-z)
                              (:quaternion-value-1 ?quaternion-value-1)
                              (:quaternion-value-2 ?quaternion-value-2)
                              (:quaternion-value-3 ?quaternion-value-3)
                              (:quaternion-value-4 ?quaternion-value-4)
                              (:mode ?mode)
                              (:name ?drawer))))
      (ros-info (drawer-pose)
                "Interacting with drawer ~a"
                open-close)
      (print open-close)
      (exe:perform open-close))))



;;==========================Dummmy-Functions=============================================================================

;;@author Felix Krause
(defun dummy-gripper (pos)
  "Receives a float value `pos'. Issues a Dummy-Command to the Gripper. Use 0.1 as pos to open the Gripper and 0.6 to open the Gripper. This Function is for use in the actual Plan - does not work in the REPL."
  (let ((pub (advertise "/hsrb/gripper_controller/command" "trajectory_msgs/JointTrajectory")))
      (sleep 1)
      (print (roslisp::subscriber-connections (gethash "/hsrb/gripper_controller/command" roslisp::*publications*)))
      (publish pub
               (roslisp:make-message "trajectory_msgs/JointTrajectory"
                                     (std_msgs-msg:stamp header) 
                                     (roslisp:ros-time)
                                     (std_msgs-msg:frame_id header)
                                     ""
                                     joint_names
                                     `#("hand_motor_joint")
                                     points
                                     `#(,(roslisp:make-message "trajectory_msgs/JointTrajectoryPoint"
                                                :positions `#(,pos)
                                                :velocities #(0)
                                                :accelerations #(0)
                                                :effort #(0.1)
                                                :time_from_start  1))))))


;;@author Felix Krause
(defun dummy-gripper-test (pos)
  "Receives a float value `pos'. Issues a Dummy-Command to the Gripper. Use 0.1 as pos to open the Gripper and 0.6 to open the Gripper. This Function is for use in the REPL - does not work in the actual Plan."
  (with-ros-node ("manipulationdummy")
    (let ((pub (advertise "/hsrb/gripper_controller/command" "trajectory_msgs/JointTrajectory")))
      (sleep 1)
      (print (roslisp::subscriber-connections (gethash "/hsrb/gripper_controller/command" roslisp::*publications*)))
      (publish pub
               (roslisp:make-message "trajectory_msgs/JointTrajectory"
                                     (std_msgs-msg:stamp header) 
                                     (roslisp:ros-time)
                                     (std_msgs-msg:frame_id header)
                                     ""
                                     joint_names
                                     `#("hand_motor_joint")                                    
                                     points
                                     `#(,(roslisp:make-message "trajectory_msgs/JointTrajectoryPoint"
                                                :positions `#(,pos)
                                                :velocities #(0)
                                                :accelerations #(0)
                                                :effort #(0.1)
                                                :time_from_start  1)))))))


;;@author Felix Krause
(defun dummy-arm (pos1 pos2 pos3 pos4 pos5)
  "Receives five float values `pos1', `pos2', `pos3', `pos4', `pos5'.  Issues a Dummy-Command to the Arm. This Function is for use in the actual Plan - does not work in the REPL."
  (let ((pub (advertise "/hsrb/arm_trajectory_controller/command" "trajectory_msgs/JointTrajectory")))
      (sleep 1)
      (print (roslisp::subscriber-connections (gethash "/hsrb/arm_trajectory_controller/command" roslisp::*publications*)))
      (publish pub
               (roslisp:make-message "trajectory_msgs/JointTrajectory"
                                     (std_msgs-msg:stamp header) 
                                     (roslisp:ros-time)
                                     (std_msgs-msg:frame_id header)
                                     ""
                                     joint_names
                                     `#("arm_lift_joint" "arm_flex_joint" "arm_roll_joint" "wrist_flex_joint" "wrist_roll_joint")                                     
                                     points
                                     `#(,(roslisp:make-message "trajectory_msgs/JointTrajectoryPoint"
                                                :positions `#(,pos1 ,pos2 ,pos3 ,pos4 ,pos5)
                                                :velocities #(0 0 0 0 0)
                                                :accelerations #(0 0 0 0 0)
                                                :effort #(0.1)
                                                :time_from_start  1))))))



;;@author Felix Krause
(defun dummy-arm-test (pos1 pos2 pos3 pos4 pos5)
  "Receives five float values `pos1', `pos2', `pos3', `pos4', `pos5'. Issues a Dummy-Command to the Arm. This Function is for use in the REPL - does not work in the actual Plan."
  (with-ros-node ("manipulationdummy")
    (let ((pub (advertise "/hsrb/arm_trajectory_controller/command" "trajectory_msgs/JointTrajectory")))
      (sleep 1)
      (print (roslisp::subscriber-connections (gethash "/hsrb/arm_trajectory_controller/command" roslisp::*publications*)))
      (publish pub
               (roslisp:make-message "trajectory_msgs/JointTrajectory"
                                     (std_msgs-msg:stamp header) 
                                     (roslisp:ros-time)
                                     (std_msgs-msg:frame_id header)
                                     ""
                                     joint_names
                                     `#("arm_lift_joint" "arm_flex_joint" "arm_roll_joint" "wrist_flex_joint" "wrist_roll_joint")                                     
                                     points
                                     `#(,(roslisp:make-message "trajectory_msgs/JointTrajectoryPoint"
                                                :positions `#(,pos1 ,pos2 ,pos3 ,pos4 ,pos5)
                                                :velocities #(0 0 0 0 0)
                                                :accelerations #(0 0 0 0 0)
                                                :effort #(0.1)
                                                :time_from_start  1)))))))
