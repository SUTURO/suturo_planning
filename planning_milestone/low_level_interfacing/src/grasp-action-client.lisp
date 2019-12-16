;;; A client to grasp an object. This is based on the HSR
;;; documentation and uses the topics and actions the HSR comes with.
;;; This can be used separately to giskard or replaced by giskard entierly.
(in-package :llif)

(defparameter *grasp-action-timeout* 300.0 "in seconds")
(defparameter *grasp-action-client* NIL)

(defun get-grasp-action-client ()
  "returns the currently used grasp-action client. If none yet exists,
   creates one."
  (or *grasp-action-client*
      (init-grasp-action-client)))

(defun init-grasp-action-client ()
  "initializes the grasp-action-client and makes sure it is connected to the
action server."
  (setf *grasp-action-client*
        (actionlib:make-action-client "manipulation/GraspAction"
                                      "manipulation_action_msgs/GraspAction"))
  (loop until (actionlib:wait-for-server *grasp-action-client*
                                         *grasp-action-timeout*))

  (roslisp:ros-info (grasp-action) "grasp action client created"))

;; NOTE most of these params have to be (vector ...)s 
(defun make-grasp-action-goal (object-frame-id)
  "Creates the grasp-action-goal. Does not send it to the action server though."
  (actionlib:make-action-goal
      (get-grasp-action-client)
      ;; (cram-simple-actionlib-client::get-simple-action-client 'grasp-action)
    trajectory (roslisp:make-message
                "manipulation_action_msgs/GraspAction" 
                (object-frame-id) object-frame-id)))
                                                 
(defun ensure-grasp-goal-reached (status object-id)
  (roslisp:ros-warn (grasp-action) "Status ~a" status)
  status
  mode
  T)


(defun call-grasp-action (object-id)
  "object-id' object-id of frame-id of object to grasp"
  ;;  (format t "grasp called with state: ~a" state)
   (multiple-value-bind (result status)
  (actionlib:call-goal (get-grasp-action-client)
                       (object-id
                        ))
     (roslisp:ros-info (grasp-action) "grasp action finished")
    (ensure-grasp-goal-reached status object-id)
     (values result status)))

;;NOTE 0 0 is the deafault lookig straight position.
(defun test-grasp-action ()
  "A function to test the grasp action."
  (call-grasp-action "test-id"))
