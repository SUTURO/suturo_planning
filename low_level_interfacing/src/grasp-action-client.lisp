;;; A client to grasp an object. This is based on the HSR
;;; documentation and uses the topics and actions the HSR comes with.
;;; This can be used separately to giskard or replaced by giskard entierly.
(in-package :llif)

(defparameter *grasp-action-timeout* 30.0 "in seconds")
(defparameter *grasp-action-client* NIL)

(defun get-grasp-action-client ()
  "returns the currently used grasp-action client. If none yet exists,
   creates one."
  (or *grasp-action-client*
      (init-grasp-action-client)))

(defun init-grasp-action-client ()
  "initializes the grasp-action-client and makes sure it is connected to the
action server."
  (roslisp:ros-info (grasp-action) "start")
  (setf *grasp-action-client*
        (actionlib:make-action-client "/grasp_server"
                                      "manipulation_msgs/GraspAction"))
  (loop until (actionlib:wait-for-server *grasp-action-client*
                                         *grasp-action-timeout*))

  (roslisp:ros-info (grasp-action) "grasp action client created"))

;; NOTE most of these params have to be (vector ...)s 
(defun make-grasp-action-goal (point-x-object
                               point-y-object
                               point-z-object
                               quaterion-value-1
                               quaterion-value-2
                               quaterion-value-3
                               quaterion-value-4
                               size-x
                               size-y
                               size-z
                               object-id
                               grasp-mode)
  "Creates the grasp-action-goal. Does not send it to the action server though."
  (actionlib:make-action-goal
      (get-grasp-action-client)
      ;; (cram-simple-actionlib-client::get-simple-action-client 'grasp-action)
    :goal_pose (cl-transforms-stamped:to-msg
                (cl-tf::make-pose-stamped  
                 "map"
                 0.0 
                 (cl-tf:make-3d-vector point-x-object point-y-object point-z-object) 
                 (cl-tf:make-quaternion quaterion-value-1 quaterion-value-2
                                        quaterion-value-3 quaterion-value-4)))
    :grasp_mode grasp-mode
    :object_frame_id object-id
    :object_size (roslisp:make-msg
                  "geometry_msgs/vector3"
                  :x size-x
                  :y size-y
                  :z size-z)
    ))
                                                 
(defun ensure-grasp-goal-reached (status
                                  point-x-object
                                  point-y-object
                                  point-z-object
                                  quaterion-value-1
                                  quaterion-value-2
                                  quaterion-value-3
                                  quaterion-value-4
                                  size-x
                                  size-y
                                  size-z)
  (roslisp:ros-warn (grasp-action) "Status ~a" status)
  status
  point-x-object point-y-object point-z-object
  quaterion-value-1 quaterion-value-2 quaterion-value-3 quaterion-value-4
  size-x size-y size-z
  T)


(defun call-grasp-action (point-x-object
                          point-y-object
                          point-z-object
                          quaterion-value-1
                          quaterion-value-2
                          quaterion-value-3
                          quaterion-value-4
                          size-x
                          size-y
                          size-z
                          object-id
                          grasp-mode)
  "object-id' object-id of frame-id of object to grasp"
  ;;  (format t "grasp called with state: ~a" state)
   (multiple-value-bind (result status)
  (actionlib:call-goal (get-grasp-action-client)
                       (make-grasp-action-goal point-x-object
                                               point-y-object
                                               point-z-object
                                               quaterion-value-1
                                               quaterion-value-2
                                               quaterion-value-3
                                               quaterion-value-4
                                               size-x size-y size-z
                                               object-id grasp-mode
                        ))
     (roslisp:ros-info (grasp-action) "grasp action finished")
     (ensure-grasp-goal-reached status
                                point-x-object point-y-object point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size-x size-y size-z)
     (values result status)))

;;NOTE 0 0 is the deafault lookig straight position.
(defun test-grasp-action ()
  "A function to test the grasp action."
  (call-grasp-action 1 1 1 1 1 1 1 5 5 5 "Hallo" 0))
