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
  (setf *grasp-action-client*
        (actionlib:make-action-client "grasps_server"
                                      "manipulation_action_msgs/GraspAction"))
  (loop until (actionlib:wait-for-server *grasp-action-client*
                                         *grasp-action-timeout*))

  (roslisp:ros-info (grasp-action) "grasp action client created"))

;; NOTE most of these params have to be (vector ...)s 
(defun make-grasp-action-goal (px py pz ox oy oz ow size_x size_y size_z)
  "Creates the grasp-action-goal. Does not send it to the action server though."
  (actionlib:make-action-goal
      (get-grasp-action-client)
      ;; (cram-simple-actionlib-client::get-simple-action-client 'grasp-action)
    :goal_pose (cl-transforms-stamped:to-msg
                (cl-tf::make-pose-stamped  
                 "map"
                 0.0 
                 (cl-tf:make-3d-vector px py pz) 
                 (cl-tf:make-quaternion ox oy oz ow)))
    :object_size (roslisp:make-msg
                  "geometry_msgs/vector3"
                  :x size_x
                  :y size_y
                  :z size_z)
    ))
                                                 
(defun ensure-grasp-goal-reached (status px py pz ox oy oz ow size_x size_y size_z)
  (roslisp:ros-warn (grasp-action) "Status ~a" status)
  status
  px py pz ox oy oz ow size_x size_y size_z
  T)


(defun call-grasp-action (px py pz ox oy oz ow size_x size_y size_z)
  "object-id' object-id of frame-id of object to grasp"
  ;;  (format t "grasp called with state: ~a" state)
   (multiple-value-bind (result status)
  (actionlib:call-goal (get-grasp-action-client)
                       (make-grasp-action-goal px py pz ox oy oz ow size_x size_y size_z
                        ))
     (roslisp:ros-info (grasp-action) "grasp action finished")
    (ensure-grasp-goal-reached status px py pz ox oy oz ow size_x size_y size_z)
     (values result status)))

;;NOTE 0 0 is the deafault lookig straight position.
(defun test-grasp-action ()
  "A function to test the grasp action."
  (call-grasp-action 1 1 1 1 1 1 1 5 5 5))
