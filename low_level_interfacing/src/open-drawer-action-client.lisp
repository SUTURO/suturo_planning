(in-package :llif)

(defparameter *drawer-action-timeout* 30.0 "in seconds")
(defparameter *drawer-action-client* NIL)

;;@author Felix Krause
(defun get-drawer-action-client ()
  "Returns the currently used grasp-action client. If none yet exists, creates one."
  (or *drawer-action-client*
      (init-drawer-action-client)))

;; used in cleanup
;;@author Felix Krause
(defun init-drawer-action-client ()
  "Initializes the drawer-action-client and makes sure it is connected to the action server."
  (roslisp:ros-info (drawer-action)
                    "start")
  (setf *drawer-action-client*
        (actionlib:make-action-client "/open_server"
                                      "manipulation_msgs/OpenAction"))
  (loop until
        (actionlib:wait-for-server *grasp-action-client*))

  (roslisp:ros-info (grasp-action)
                    "drawer action client created")
	*drawer-action-client*
)



;;@author Felix Krause
(defun make-drawer-action-goal (point-x
                               point-y
                               point-z
                               quaterion-value-1
                               quaterion-value-2
                               quaterion-value-3
                               quaterion-value-4
                               mode
			       name)
"Receives x, y, z coordinates and the 4 quaternion values required to create a pose, aswell as a integer representing a graspmode `mode' and the name of the drawer `drawer'. Makes and returns an drawer action goal. It turns the point-x, point-y, point-z and the quaternions into an stamped pose."
  (actionlib:make-action-goal
      (get-drawer-action-client)
    :goal_pose (cl-transforms-stamped:to-msg
                (cl-tf::make-pose-stamped  
                 "map" 0.0 
                 (cl-tf:make-3d-vector point-x point-y point-z) 
                 (cl-tf:make-quaternion quaterion-value-1 quaterion-value-2
                                        quaterion-value-3 quaterion-value-4)))
    :openorclose mode
    :object_name name))
    

;;@author Felix Krause
(defun ensure-drawer-goal-reached (status
                                  point-x
                                  point-y
                                  point-z
                                  quaterion-value-1
                                  quaterion-value-2
                                  quaterion-value-3
                                  quaterion-value-4
				  mode
				  name)
  "Receives a status `status', x, y, z coordinates and the 4 quaternion values required to create a pose, aswell as a integer representing a graspmode `mode' and the name of the drawer `drawer'. Evaluates all received variables one after another and returns T at the end."
  (roslisp:ros-warn (drawer-action)
                    "Status ~a"
                    status)
  status
  point-x point-y point-z
  quaterion-value-1 quaterion-value-2 quaterion-value-3 quaterion-value-4
  T)

;;@author Felix Krause
(defun call-drawer-action (point-x
                          point-y
                          point-z
                          quaterion-value-1
                          quaterion-value-2
                          quaterion-value-3
                          quaterion-value-4
                          mode
                           name)
  "Receives x, y, z coordinates and the 4 quaternion values required to create a pose, aswell as a integer representing a graspmode `mode' and the name of the drawer `drawer'. Constructs a grasp action goal with using the values received."
  (print point-x)
  (print point-y)
  (print point-z)
  (print quaterion-value-1)
  (print quaterion-value-2)
  (print quaterion-value-3)
  (print quaterion-value-4)
  (print mode)
  (print name)

   
  (multiple-value-bind (result status)
      (actionlib:call-goal (get-drawer-action-client)
                           (make-drawer-action-goal point-x
                                                   point-y
                                                   point-z
                                                   quaterion-value-1
                                                   quaterion-value-2
                                                   quaterion-value-3
                                                   quaterion-value-4
                                                   mode
						   name))
    (roslisp:ros-info (drawer-action)
                      "drawer action finished")
    (ensure-drawer-goal-reached status
                               point-x point-y point-z
                               quaterion-value-1 quaterion-value-2
                               quaterion-value-3 quaterion-value-4
                               mode name)
    (values result status)))
