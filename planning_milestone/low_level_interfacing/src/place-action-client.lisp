;;; A client to handle placing motions. This is based on the HSR
;;; documentation and uses the topics and actions the HSR comes with.
;;; This can be used separately to giskard or replaced by giskard entierly.
(in-package :llif)

(defparameter *place-action-timeout* 300.0 "in seconds")
(defparameter *place-action-client* NIL)

(defun get-place-action-client ()
  "returns the currently used move gripper client. If none yet exists,
   creates one."
  (or *place-action-client*
      (init-place-action-client)))

(defun init-place-action-client ()
  "initializes the place-action-client and makes sure it is connected to the
action server."
  (setf *place-action-client*
        (actionlib:make-action-client "manipulation/PlaceAction"
                                      "manipulation_action_msgs/PlaceAction"))
  (loop until (actionlib:wait-for-server *place-action-client*
                                         *place-action-timeout*))

  (roslisp:ros-info (gripper-action) "gripper action client created"))

;; NOTE most of these params have to be (vector ...)s 
(defun make-place-action-goal (px py pz ox oy oz ow)
  "Creates the place-action-goal. Does not send it to the action server though."
  (actionlib:make-action-goal
      (get-place-action-client)
      ;; (cram-simple-actionlib-client::get-simple-action-client 'place-action)
    trajectory (roslisp:make-message
                "manipulation_action_msgs/PlaceAction" 
                trajectory (roslisp:make-message
                "geometry_msgs/PoseStamped" 
                (x position pose) px
                (y position pose) py
                (z position pose) pz
                (w orientation pose) ow
                (x orientation pose) ox
                (y orientation pose) oy
                (z orientation pose) oz))))
                                                 
(defun ensure-place-action-goal-reached (status pos)
  (roslisp:ros-warn (place-action) "Status ~a" status)
  status
  pos
  T)


(defun call-place-action (px py pz ox oy oz ow)
  "Moves the gripper to the given position.
   `pos' a vector of two values.
   First value describes +left or -right.
   Second value describes +up and -down.
   (vector 0.0 0.0) looks straight ahead."
  ;;  (format t "move gripper called with pos: ~a" pos)
   (multiple-value-bind (result status)
  (actionlib:call-goal (get-place-action-client)
                       (make-place-action-goal px py pz ox oy oz ow
                        ))
     (roslisp:ros-info (place-action) "place action finished")
    (ensure-place-action-goal-reached status pos)
     (values result status)))

;;NOTE 0 0 is the deafault lookig straight position.
(defun test-place-action ()
  "A function to test the place action movement."
  (call-place-action 1 1 1 5 5 5 5))
