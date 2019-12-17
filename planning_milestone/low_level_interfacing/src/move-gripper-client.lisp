;;; A client to move the robots gripper. This is based on the HSR
;;; documentation and uses the topics and actions the HSR comes with.
;;; This can be used separately to giskard or replaced by giskard entierly.
(in-package :llif)

(defparameter *move-gripper-action-timeout* 300.0 "in seconds")
(defparameter *move-gripper-client* NIL)

(defun get-move-gripper-client ()
  "returns the currently used move gripper client. If none yet exists,
   creates one."
  (or *move-gripper-client*
      (init-move-gripper-action-client)))

(defun init-move-gripper-action-client ()
  "initializes the move-gripper-action-client and makes sure it is connected to the
action server."
  (setf *move-gripper-client*
        (actionlib:make-action-client "manipulation/MoveGripperAction"
                                      "manipulation_action_msgs/MoveGripperAction"))
  (loop until (actionlib:wait-for-server *move-gripper-client*
                                         *move-gripper-action-timeout*))

  (roslisp:ros-info (gripper-action) "gripper action client created"))

;; NOTE most of these params have to be (vector ...)s 
(defun make-move-gripper-action-goal (px py pz ox oy oz ow)
  "Creates the move-gripper-action-goal. Does not send it to the action server though."
  (actionlib:make-action-goal
      (get-move-gripper-client)
      ;; (cram-simple-actionlib-client::get-simple-action-client 'move-gripper-action)
    trajectory (roslisp:make-message
                "manipulation_action_msgs/MoveGripperAction" 
                trajectory (roslisp:make-message
                "geometry_msgs/PoseStamped" 
                (x position pose) px
                (y position pose) py
                (z position pose) pz
                (w orientation pose) ow
                (x orientation pose) ox
                (y orientation pose) oy
                (z orientation pose) oz))))
                                                 
(defun ensure-move-gripper-goal-reached (status pos)
  (roslisp:ros-warn (move-gripper) "Status ~a" status)
  status
  pos
  T)


(defun call-move-gripper-action (px py pz ox oy oz ow)
  "Moves the gripper to the given position.
   `pos' a vector of two values.
   First value describes +left or -right.
   Second value describes +up and -down.
   (vector 0.0 0.0) looks straight ahead."
  ;;  (format t "move gripper called with pos: ~a" pos)
   (multiple-value-bind (result status)
  (actionlib:call-goal (get-move-gripper-client)
                       (make-move-gripper-action-goal px py pz ox oy oz ow
                        ))
     (roslisp:ros-info (move-gripper) "move gripper action finished")
    (ensure-move-gripper-goal-reached status pos)
     (values result status)))

;;NOTE 0 0 is the deafault lookig straight position.
(defun test-move-gripper ()
  "A function to test the gripper movement."
  (call-move-gripper-action 1 1 1 5 5 5 5))
