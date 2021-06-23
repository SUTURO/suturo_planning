(in-package :llif)

(defparameter *take-pose-action-timeout* 30.0 "in seconds")
(defparameter *take-pose-action-client* NIL)

(defun get-take-pose-action-client ()
  "returns the currently used take-pose-action client. If none yet exists,
   creates one." 
  (roslisp:ros-info (take-pose-action-client) "Getting take pose action client")
  (or *take-pose-action-client*
      (init-take-pose-action-client)))

(defun init-take-pose-action-client ()
  "initializes the take_pose-action-client and makes sure it is connected to the
action server."
  (roslisp:ros-info (take-pose-action-client) "Initialising take pose action client")
  (setf *take-pose-action-client*
        (actionlib:make-action-client "/take_pose_server"
                                      "manipulation_msgs/TakePoseAction"))
  (loop until
        (actionlib:wait-for-server *take-pose-action-client*))
  (roslisp:ros-info (take-pose-action-client) "Take pose action client initialised"))

(defun make-take-pose-action-goal (pose-mode head-pan
                                   head-tilt arm-lift
                                   arm-flex arm-roll
                                   wrist-flex wrist-roll
                                   px py pz)
  "Creates tge take_pose-action-goal"
(roslisp:ros-info (take-pose-action-client) "Creating take pose action message")
  (actionlib:make-action-goal
      (get-take-pose-action-client)
    :pose_mode pose-mode
    :head_pan_joint head-pan
    :head_tilt_joint head-tilt
    :arm_lift_joint arm-lift
    :arm_flex_joint arm-flex
    :arm_roll_joint arm-roll
    :wrist_flex_joint wrist-flex
    :wrist_roll_joint wrist-roll
    :gaze_point (roslisp:make-msg
                  "geometry_msgs/vector3"
                  :x px
                  :y py
                  :z pz)))

(defun ensure-take-pose-goal-reached (status mode head-pan
                                      head-tilt arm-lift
                                      arm-flex arm-roll
                                      wrist-flex wrist-roll
                                      px py pz)
  (roslisp:ros-warn (take-pose-action-client) "Status ~a" status)
  status
  mode
  head-pan
  head-tilt
  arm-lift
  arm-flex
  arm-roll
  wrist-flex
  wrist-roll
  px
  py
  pz
  T)


(defun call-take-pose-action (pose-mode &optional
                                          (head-pan 0.0) (head-tilt 0.0)
                                          (arm-lift 0.0) (arm-flex 0.0)
                                          (arm-roll 0.0) (wrist-flex 0.0)
                                          (wrist-roll 0.0) (px 0.0)
                                          (py 0.0) (pz 0.0))
  "torso-joint-state' state of torso-joint.
   take-pose-mode int value to describe take-pose-mode"
  (multiple-value-bind (result status)
      (actionlib:call-goal (get-take-pose-action-client)
                           (make-take-pose-action-goal pose-mode head-pan
                                                       head-tilt arm-lift
                                                       arm-flex arm-roll
                                                       wrist-flex wrist-roll
                                                       px py pz))
    (roslisp:ros-info (take-pose-action) "Take pose action ~a finished" pose-mode)
    (ensure-take-pose-goal-reached status pose-mode
                                   head-pan head-tilt
                                   arm-lift arm-flex
                                   arm-roll  wrist-flex
                                   wrist-roll px
                                   py pz)
      (values result status)))



(defun call-take-gaze-pose-action (&key
                                     ((:head-tilt head-tilt) 0)
                                     ((:px px) 0.0)
                                     ((:py py) 0.0)
                                     ((:pz pz) 0.0))
  (multiple-value-bind (result status)
      (actionlib:call-goal (get-take-pose-action-client)
                           (make-take-pose-action-goal 5 0 0 0 0 0 0 0 px py pz))
    (roslisp:ros-info (take-pose-action) "Take pose action GAZE finished point: (~a, ~a, ~a) and head tilt: ~a" px py pz head-tilt)
    (ensure-take-pose-goal-reached status 5 0 0 0 0 0 0 0 px py pz)
      (values result status)))
