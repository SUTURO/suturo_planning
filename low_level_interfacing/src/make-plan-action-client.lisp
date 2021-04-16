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

  (roslisp:ros-info (make-plan-action) "make plan action client created"))

;; NOTE most of these params have to be (v,,,ector ...)s 
(defun make-plan-action-goal (start-pose object-frame-id goal-pose object-size gripper-mode action-mode)
  "Creates the make-plan-action-goal."
  (actionlib:make-action-goal
      (get-make-plan-action-client)
    :start_pose start-pose
    :object_frame_id object-frame-id
    :goal_pose goal-pose
    :object_size object-size
    :gripper_mode gripper-mode
    :action_mode action-mode))

(defun try-make-plan-action (start-pose object-frame-id goal-pose object-size gripper-mode action-mode)
  ""
  (let ((goal (make-plan-action-goal 
                                (cl-transforms-stamped:to-msg start-pose) ;;;PoseStamped
                                object-frame-id ;;String
                                (cl-transforms-stamped:to-msg goal-pose) ;;PoseStamped
                                (cl-transforms-stamped:to-msg object-size) ;;Vector3
                                gripper-mode ;;uint8 or Free, Top, Fron
                                action-mode)))
    (roslisp:ros-info (make-plan-action) "Make Plan Goal: ~% ~a" goal)    
    (multiple-value-bind (result status)
        (actionlib:call-goal (get-make-plan-action-client)
                             goal)
      (values result status))))

