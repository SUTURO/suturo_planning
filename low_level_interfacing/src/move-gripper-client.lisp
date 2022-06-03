;;; A client to move the robots gripper. This is based on the HSR
;;; documentation and uses the topics and actions the HSR comes with.
;;; This can be used separately to giskard or replaced by giskard entierly.
(in-package :llif)

(defparameter *move-gripper-action-timeout* 30.0 "in seconds")
(defparameter *move-gripper-client* NIL)

(defun get-move-gripper-client ()
  "Returns the currently used move gripper client. If none yet exists, creates one."
  (or *move-gripper-client*
      (init-move-gripper-action-client)))

(defun init-move-gripper-action-client ()
  "Initializes the move-gripper-action-client and makes sure it is connected to the action server."
  (setf *move-gripper-client*
        (actionlib:make-action-client "/move_gripper_server"
                                      "manipulation_msgs/MoveGripperAction"))
  (loop until (actionlib:wait-for-server *move-gripper-client*
                                         *move-gripper-action-timeout*))

  (roslisp:ros-info (gripper-action) "gripper action client created"))

;; NOTE most of these params have to be (v,,,ector ...)s 
(defun make-move-gripper-action-goal (px py pz ox oy oz ow)
  "Receives pose in the form of the coordinates `px', `py', `pz', and the quaternion `ox', `oy', `oz', `ow'.
Creates the move-gripper-action-goal, but does not send it to the action server."
  (actionlib:make-action-goal
      (get-move-gripper-client)
    ;; (cram-simple-actionlib-client::get-simple-action-client 'move-gripper-action) 
    :goal_pose (cl-transforms-stamped:to-msg 
                (cl-tf::make-pose-stamped  
                 "map" 0.0 
                 (cl-tf:make-3d-vector px py pz) 
                 (cl-tf:make-quaternion ox oy oz ow)))))

(defun ensure-move-gripper-goal-reached (status pos)
  "Receives status `status' and pose `pos'. Evaluates all received variables one after another and returns T at the end."
  (roslisp:ros-warn (move-gripper)
                    "Status ~a"
                    status)
  status
  pos
  T)


(defun call-move-gripper-action (px py pz ox oy oz ow)
"Receives pose in the form of the coordinates `px', `py', `pz', and the quaternion `ox', `oy', `oz', `ow'.
Moves the gripper to the given position."
  ;;  (format t "move gripper called with pos: ~a" pos)
  (multiple-value-bind (result status)
      (actionlib:call-goal (get-move-gripper-client)
                           (make-move-gripper-action-goal px py pz ox oy oz ow))
    (roslisp:ros-info (move-gripper)
                      "move gripper action finished")
    ;;(ensure-move-gripper-goal-reached status pos)
    (values result status)))

