;;; A client to prepare the robot to percieve objects. This is based on the HSR
;;; documentation and uses the topics and actions the HSR comes with.
;;; This can be used separately to giskard or replaced by giskard entierly.
(in-package :llif)

(defparameter *perceive-action-timeout* 300.0 "in seconds")
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
        (actionlib:make-action-client "manipulation/PercieveAction"
                                      "manipulation_action_msgs/PercieveAction"))
  (loop until (actionlib:wait-for-server *perceive-action-client*
                                         *perceive-action-timeout*))

  (roslisp:ros-info (perceive-action) "perceive action client created"))

;; NOTE most of these params have to be (vector ...)s 
(defun make-perceive-action-goal (torso-joint-state percieve-mode)
  "Creates the perceive-action-goal. Does not send it to the action server though."
  (actionlib:make-action-goal
      (get-perceive-action-client)
      ;; (cram-simple-actionlib-client::get-simple-action-client 'perceive-action)
    trajectory (roslisp:make-message
                "manipulation_action_msgs/PercieveAction" 
                (torso-joint-state) torso-joint-state
                (perceive-mode) perceive-mode)))
                                                 
(defun ensure-perceive-goal-reached (status mode)
  (roslisp:ros-warn (perceive-action) "Status ~a" status)
  status
  mode
  T)


(defun call-perceive-action (torso-joint-state percieve-mode)
  "torso-joint-state' state of torso-joint.
   percieve-mode int value to describe percieve-mode"
  ;;  (format t "perceive called with state: ~a" state)
   (multiple-value-bind (result status)
  (actionlib:call-goal (get-perceive-action-client)
                       (make-perceive-action-goal torso-joint-state percieve-mode
                        ))
     (roslisp:ros-info (perceive-action) "perceive action finished")
    (ensure-perceive-action-goal-reached status mode)
     (values result status)))

;;NOTE 0 0 is the deafault lookig straight position.
(defun test-perceive-action ()
  "A function to test the perceive action."
  (call-perceive-action 0 1))
