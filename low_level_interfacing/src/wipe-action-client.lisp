(in-package :llif)

(defparameter *wipe-action-timeout* 30.0 "in seconds")
(defparameter *wipe-action-client* NIL)


;; @author Felix Krause
(defun init-wipe-action-client ()
  "Initializes the wipe action client."
  (roslisp:ros-info (wipe-action-client)
                    "Initialising wipe action client")
  (setf *wipe-action-client*
        (actionlib:make-action-client "/move_gripper_server"
                                      "manipulation_msgs/MoveGripperAction"))
  (loop until
        (actionlib:wait-for-server *wipe-action-client*))
  (roslisp:ros-info (wipe-action-client)
                    "Wipe action client initialised")
  *wipe-action-client*)

;; @author Felix Krause
(defun get-wipe-action-client ()
  "Returns the wipe action client. If none exists, one is created."
  (roslisp:ros-info (wipe-action-client)
                    "Getting wipe action client")
  (or *wipe-action-client*
      (init-wipe-action-client)))

;; @author Felix Krause
(defun make-wipe-action-goal (point-x
                               point-y
                               point-z
                               quaterion-value-1
                               quaterion-value-2
                               quaterion-value-3
                               quaterion-value-4)
"Receives x, y, z coordinates and the 4 quaternion values required to create a pose. Makes and returns an grasp action goal."
  (actionlib:make-action-goal
      (get-wipe-action-client)
    :goal_pose (cl-transforms-stamped:to-msg
                (cl-tf::make-pose-stamped  
                 "map" 0.0 
                 (cl-tf:make-3d-vector point-x point-y point-z) 
                 (cl-tf:make-quaternion quaterion-value-1 quaterion-value-2
                                        quaterion-value-3 quaterion-value-4)))))

;;@author Felix Krause
(defun ensure-wipe-pose-goal-reached (status
                                  point-x
                                  point-y
                                  point-z
                                  quaterion-value-1
                                  quaterion-value-2
                                  quaterion-value-3
                                  quaterion-value-4)
  "Receives a status `status', x, y, z coordinates and the 4 quaternion values required to create a pose. Evaluates all received variables one after another and returns T at the end."
  (roslisp:ros-warn (wipe-action)
                    "Status ~a"
                    status)
  status
  point-x point-y point-z
  quaterion-value-1 quaterion-value-2 quaterion-value-3 quaterion-value-4
  T)

;; @author Felix Krause
(defun call-wipe-action (point-x
                          point-y
                          point-z
                          quaterion-value-1
                          quaterion-value-2
                          quaterion-value-3
                          quaterion-value-4)
  "Receives a x, y, z coordinates and the 4 quaternion values required to create a pose. Constructs a grasp action goal with using the values received."
  (multiple-value-bind (result status)
      (actionlib:call-goal (print (get-wipe-action-client))
                           (make-wipe-action-goal point-x
                                                   point-y
                                                   point-z
                                                   quaterion-value-1
                                                   quaterion-value-2
                                                   quaterion-value-3
                                                   quaterion-value-4))
    (roslisp:ros-info (wipe-action)
                      "Wipe action finished")
    (ensure-wipe-pose-goal-reached status
                               point-x point-y point-z
                               quaterion-value-1 quaterion-value-2
                               quaterion-value-3 quaterion-value-4)
    (values result status)))


