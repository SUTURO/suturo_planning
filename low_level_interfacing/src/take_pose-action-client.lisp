(in-package :llif)

(defparameter *take_pose-action-timeout* 30.0 "in seconds")
(defparameter *take_pose-action-client* NIL)

(defun get-take_pose-action-client ()
  "returns the currently used take_pose-action client. If none yet exists,
   creates one." 
  (print "get-take-pose-action-client")
  (or *take_pose-action-client*
      (init-take_pose-action-client)))

(defun init-take_pose-action-client ()
  "initializes the take_pose-action-client and makes sure it is connected to the
action server."
(print "init-take-pose-action-client")
  (setf *take_pose-action-client*
        (actionlib:make-action-client "take_pose_server"
                                      "manipulation_action_msgs/TakePoseAction"))
  (loop until (actionlib:wait-for-server *take_pose-action-client*
                                         *take_pose-action-timeout*))
  (roslisp:ros-info (take_pose-action) "take_pose action client created"))

(defun make-take_pose-action-goal (goal head_pan head_tilt arm_lift
                                                       arm_flex arm_roll
                                                       wrist_flex wrist_roll px py pz)
  "Creates tge take_pose-action-goal"
(print "make-take-pose-action-goal")
  (actionlib:make-action-goal
      (get-take_pose-action-client)
    :pose_mode goal
    :head_pan_joint head_pan
    :head_tilt_joint head_tilt
    :arm_lift_joint arm_lift
    :arm_flex_joint arm_flex
    :arm_roll_joint arm_roll
    :wrist_flex_joint wrist_flex
    :wrist_roll_joint wrist_roll
    :gaze_point (roslisp:make-msg
                  "geometry_msgs/vector3"
                  :x px
                  :y py
                  :z pz)))

(defun ensure-take_pose-goal-reached (status mode head_pan head_tilt arm_lift
                                                       arm_flex arm_roll
                                                       wrist_flex wrist_roll px py pz)
  (roslisp:ros-warn (take_pose-action) "Status ~a" status)
  status
  mode
  head_pan
  head_tilt
  arm_lift
  arm_flex
  arm_roll
  wrist_flex
  wrist_roll
  px
  py
  pz
  T)


(defun call-take_pose-action (pose_mode &optional (head_pan 0.0)
                                               (head_tilt 0.0) (arm_lift 0.0)
                                               (arm_flex 0.0) (arm_roll 0.0)
                                               (wrist_flex 0.0) (wrist_roll 0.0) (px 0) (py 0) (pz 0))
  "torso-joint-state' state of torso-joint.
   take_pose_mode int value to describe take_pose_mode"
  ;;  (format t "take_pose called with state: ~a" state)
  (print "1")
  (multiple-value-bind (result status)
      (actionlib:call-goal (get-take_pose-action-client)
                           (make-take_pose-action-goal pose_mode head_pan
                                                       head_tilt arm_lift
                                                       arm_flex arm_roll
                                                       wrist_flex wrist_roll px py pz))
    (print "2")

    (roslisp:ros-info (take_pose-action) "take_pose action finished")
      (ensure-take_pose-goal-reached status pose_mode head_pan
                                                       head_tilt arm_lift
                                                       arm_flex arm_roll
                                                       wrist_flex wrist_roll px py pz)
      (values result status)))



