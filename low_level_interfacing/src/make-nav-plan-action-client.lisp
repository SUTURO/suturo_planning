(in-package :llif)

(defparameter *make-plan-action-timeout* 30.0 "in seconds")
(defparameter *make-plan-client* NIL)

(defun get-make-plan-action-client ()
  "returns the currently used make plan action client. If none yet exists,
   creates one."
  (or *make-plan-client*
      (init-make-plan-action-client)))

(defun init-make-plan-action-client ()
  "initializes the make-plan-action-client and makes sure it is connected to the
action server."
  (setf *make-plan-client*
        (actionlib:make-action-client "/make_plan_server"
                                      "manipulation_msgs/MakePlanAction"))
  (loop until (actionlib:wait-for-server *make-plan-client*
                                         *make-plan-action-timeout*))

  (roslisp:ros-info (gripper-action) "make plan action client created"))

;; NOTE most of these params have to be (v,,,ector ...)s 
(defun make-plan-action-goal (start-pose object-frame-id goal-pose object-size gripper-mode action-mode)
  "Creates the make-plan-action-goal."
  (actionlib:make-action-goal
      (get-make-plan-action-client)
    ;; (cram-simple-actionlib-client::get-simple-action-client 'make-plan-action) 
    :start_pose start-pose
    :object_frame_id object-frame-id
    :goal_pose goal-pose
    :object_size object-size
    :gripper_mode gripper-mode
    :action_mode action-mode))

(defun try-make-plan-action (start-pose object-frame-id goal-pose objet-size gripper-mode action-mode)
  ""
   (multiple-value-bind (result status)
        (actionlib:call-goal (get-make-plan-action-client)
                             (make-plan-action-goal 
                                start-pose 
                                object-frame-id 
                                goal-pose 
                                objet-size 
                                gripper-mode 
                                action-mode))
        (roslisp:ros-info (make-plan) "make plan action finished")
        (values result status)))

