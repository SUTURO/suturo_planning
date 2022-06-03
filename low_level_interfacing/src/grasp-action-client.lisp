;;; A client to grasp an object. This is based on the HSR
;;; documentation and uses the topics and actions the HSR comes with.
;;; This can be used separately to giskard or replaced by giskard entierly.
(in-package :llif)

(defparameter *grasp-action-timeout* 30.0 "in seconds")
(defparameter *grasp-action-client* NIL)

;;@author Jan Schimpf
(defun get-grasp-action-client ()
  "Returns the currently used grasp-action client. If none yet exists, creates one."
  (or *grasp-action-client*
      (init-grasp-action-client)))

;; used in cleanup
;;@author Jan Schimpf
(defun init-grasp-action-client ()
  "Initializes the grasp-action-client and makes sure it is connected to the action server."
  (roslisp:ros-info (grasp-action)
                    "start")
  (setf *grasp-action-client*
        (actionlib:make-action-client "/grasp_server"
                                      "manipulation_msgs/GraspAction"))
  (loop until
        (actionlib:wait-for-server *grasp-action-client*))

  (roslisp:ros-info (grasp-action)
                    "grasp action client created"))

;;@author Jan Schimpf
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
  "Receives x, y, z coordinates and the 4 quaternion values required to create a pose, aswell as
dimensions of the object `size-x', `size-y' and `size-z', the object ID `object-id' and a integer
representing a graspmode `graspmode'. Makes and returns an grasp action goal. It turns the point-x,
point-y, point-z and the quaternions into an stamped pose and size-x, size-y and size-z into an vector3."
  (actionlib:make-action-goal
      (get-grasp-action-client)
    ;; (cram-simple-actionlib-client::get-simple-action-client 'grasp-action)
    :goal_pose (cl-transforms-stamped:to-msg
                (cl-tf::make-pose-stamped  
                 "map" 0.0 
                 (cl-tf:make-3d-vector point-x-object point-y-object point-z-object) 
                 (cl-tf:make-quaternion quaterion-value-1 quaterion-value-2
                                        quaterion-value-3 quaterion-value-4)))
    :grasp_mode grasp-mode
    :object_frame_id object-id
    :object_size (roslisp:make-msg
                  "geometry_msgs/vector3"
                  :x size-x
                  :y size-y
                  :z size-z)))

;;@author Jan Schimpf                                                 
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
  "Receives a status `status', x, y, z coordinates and the 4 quaternion values required to create a pose, aswell as
dimensions of the object `size-x', `size-y' and `size-z'. Evaluates all received variables one after another and
returns T at the end."
  (roslisp:ros-warn (grasp-action)
                    "Status ~a"
                    status)
  status
  point-x-object point-y-object point-z-object
  quaterion-value-1 quaterion-value-2 quaterion-value-3 quaterion-value-4
  size-x size-y size-z
  T)

;;@author Jan Schimpf
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
  "Receives x, y, z coordinates and the 4 quaternion values required to create a pose, aswell as
dimensions of the object `size-x', `size-y' and `size-z', the object ID `object-id' and a integer
representing a graspmode `graspmode'. Constructs a grasp action goal with using the values received."
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
                                                   object-id grasp-mode))
    (roslisp:ros-info (grasp-action)
                      "grasp action finished")
    (ensure-grasp-goal-reached status
                               point-x-object point-y-object point-z-object
                               quaterion-value-1 quaterion-value-2
                               quaterion-value-3 quaterion-value-4
                               size-x size-y size-z)
    (values result status)))


