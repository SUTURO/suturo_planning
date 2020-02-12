(in-package :llif)

(defparameter *perceive-action-timeout* 30.0 "in seconds")
(defparameter *perceive-action-client* NIL)

(defun get-perceive-action-client ()
  "returns the currently used perceive-action client. If none yet exists,
   creates one."
  (or *perceive-action-client*
      (init-perceive-action-client)))

(defun init-perceive-action-client ()
  "initializes the perceive-action-client and makes sure it is connected to the
action server."
  (setf *perceive-action-client*
        (actionlib:make-action-client "perceive_server"
                                      "manipulation_action_msgs/PerceiveAction"))
  (loop until (actionlib:wait-for-server *perceive-action-client*
                                         *perceive-action-timeout*))
  (roslisp:ros-info (perceive-action) "perceive action client created"))

(defun make-perceive-action-goal (goal)
  "Creates tge perc
eive-action-goal"
  (actionlib:make-action-goal
      (get-perceive-action-client)
      :perceive_mode goal))


(defun ensure-perceive-goal-reached (status mode)
  (roslisp:ros-warn (perceive-action) "Status ~a" status)
  status
  mode
  T)


(defun call-perceive-action (perceive_mode)
  "torso-joint-state' state of torso-joint.
   perceive_mode int value to describe perceive_mode"
  ;;  (format t "perceive called with state: ~a" state)
  (multiple-value-bind (result status)
      (actionlib:call-goal (get-perceive-action-client)
                       (make-perceive-action-goal perceive_mode))
      (roslisp:ros-info (perceive-action) "perceive action finished")
      (ensure-perceive-goal-reached status perceive_mode)
      (values result status)))



