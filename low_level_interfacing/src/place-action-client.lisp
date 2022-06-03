(in-package :llif)

(defparameter *place-action-timeout* 30.0 "in seconds")
(defparameter *place-action-client* NIL)

;;@author Jan schimpf
(defun get-place-action-client ()
  "Returns the currently used move gripper client. If none yet exists, creates one."
  (or *place-action-client*
      (init-place-action-client)))

;; used in cleanup
;;@author Jan schimpf
(defun init-place-action-client ()
  "Initializes the place-action-client and makes sure it is connected to the action server."
  (setf *place-action-client*
        (actionlib:make-action-client "/place_server"
                                      "manipulation_msgs/PlaceAction"))
  (loop until
        (actionlib:wait-for-server *place-action-client*))

  (roslisp:ros-info (gripper-action)
                    "gripper action client created"))

;; NOTE most of these params have to be (vector ...)s
;;@author Jan schimpf
(defun make-place-action-goal (point-x-object 
                               point-y-object 
                               point-z-object 
                               quaterion-value-1 
                               quaterion-value-2 
                               quaterion-value-3 
                               quaterion-value-4 
                               object-id
                               grasp-mode)
 "Receives x, y, z coordinates and the 4 quaternion values required to create a pose, aswell as
the object ID `object-id' and a integer representing a graspmode `graspmode'. Makes and returns
an place action goal. It turns the point-x, point-y, point-z and the quaternions into an stamped
pose and size-x, size-y and size-z into an vector3."
  (actionlib:make-action-goal
      (get-place-action-client)
    ;; (cram-simple-actionlib-client::get-simple-action-client 'place-action)   
    :place_mode grasp-mode  
    :object_frame_id object-id
    :goal_pose (cl-transforms-stamped:to-msg (cl-tf::make-pose-stamped  
                                              "map" 0.0 
                                              (cl-tf:make-3d-vector point-x-object
                                                                    point-y-object
                                                                    point-z-object) 
                                              (cl-tf:make-quaternion quaterion-value-1
                                                                     quaterion-value-2
                                                                     quaterion-value-3
                                                                     quaterion-value-4)))))

;;@author Jan schimpf                                                
(defun ensure-place-action-goal-reached (status
                                         point-x-object 
                                         point-y-object 
                                         point-z-object 
                                         quaterion-value-1
                                         quaterion-value-2
                                         quaterion-value-3
                                         quaterion-value-4
                                         object-id
                                         grasp-mode)
  "Receives a status `status', x, y, z coordinates and the 4 quaternion values required to create a pose, aswell as
the object ID `object-id' and a integer representing a graspmode `graspmode'. Evaluates all received variables one after another and
returns T at the end."
  (roslisp:ros-warn (place-action)
                    "Status ~a"
                    status)
  status
  point-x-object point-y-object point-z-object
  quaterion-value-1 quaterion-value-2 quaterion-value-3 quaterion-value-4
  object-id grasp-mode
  T)

;;@author Jan schimpf
(defun call-place-action (point-x-object 
                          point-y-object 
                          point-z-object 
                          quaterion-value-1
                          quaterion-value-2
                          quaterion-value-3
                          quaterion-value-4
                          object-id
                          grasp-mode)
"Receives x, y, z coordinates and the 4 quaternion values required to create a pose, aswell as
the object ID `object-id' and a integer representing a graspmode `graspmode'. Constructs
 a grasp action goal with using the values received."
  ;; (format t "move gripper called with pos: ~a" pos)
  (multiple-value-bind (result status)
      (actionlib:call-goal (get-place-action-client)
                           (make-place-action-goal point-x-object
                                                   point-y-object
                                                   point-z-object 
                                                   quaterion-value-1
                                                   quaterion-value-2
                                                   quaterion-value-3
                                                   quaterion-value-4
                                                   object-id grasp-mode))
    (roslisp:ros-info (place-action)
                      "place action finished")
    (ensure-place-action-goal-reached status
                                      point-x-object point-y-object point-z-object
                                      quaterion-value-1 quaterion-value-2
                                      quaterion-value-3 quaterion-value-4
                                      object-id
                                      grasp-mode)
    (values result status)))

