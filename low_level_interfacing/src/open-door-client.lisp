;;; A client to grasp an object. This is based on the HSR
;;; documentation and uses the topics and actions the HSR comes with.
;;; This can be used separately to giskard or replaced by giskard entierly.
(in-package :llif)

(defparameter *open-door-timeout* 30.0 "in seconds")
(defparameter *open-door-client* NIL)

(defun get-open-door-client ()
  "returns the currently used open-action client. If none yet exists,
   creates one."
  (or *open-door-client*
      (init-open-door-client)))

(defun init-open-door-client ()
  "initializes the open-action-client and makes sure it is connected to the
action server."
  (setf *open-door-client*
        (actionlib:make-action-client "/open_server"
                                      "manipulation_msgs/OpenAction"))
  (loop until (actionlib:wait-for-server *open-door-client*
                                         *open-door-timeout*))

  (roslisp:ros-info (open-action) "open door client created"))

;;@author Jan Schimpf
;;Makes and returns and action goal using the door id, door handle name 
;;from the tf tree and an angle. 
(defun make-open-door-goal (door-id-name
                            door-handle-name
                            angle)
  "Creates the open-action-goal. Does not send it to the action server though."
  (actionlib:make-action-goal
      (get-open-door-client)
    :object_name door-id-name ;;knowrob_object_id
    :object_link_name door-handle-name ;;knowrob_object_id
    :angle_goal angle
    ))
                                                 
(defun ensure-open-door-goal-reached (status
                                      door-id-name
                                      door-handle-name
                                      angle)
  (roslisp:ros-warn (open-action) "Status ~a" status)
  status
  door-id-name
  door-handle-name
  angle
  T)

;;@author Jan Schimpf
;;Calls the server using the door id, door handle name and 
;;the angle to which degree the door should be open. 
(defun call-open-action (door-id-name
                         door-handle-name
                         angle)
  "object-id' object-id of frame-id of object to grasp"
  ;;  (format t "grasp called with state: ~a" state)
   (multiple-value-bind (result status)
  (actionlib:call-goal (get-open-door-client)
                       (make-open-door-goal door-id-name
                                            door-handle-name
                                            angle))
     (roslisp:ros-info (grasp-action) "open door action finished")
     (ensure-open-door-goal-reached
                                status 
                                door-id-name
                                door-handle-name
                                angle)
     (values result status)))


