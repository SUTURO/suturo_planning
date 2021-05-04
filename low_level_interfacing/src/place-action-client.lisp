(in-package :llif)

(defparameter *place-action-timeout* 30.0 "in seconds")
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
        (actionlib:make-action-client "/place_server"
                                      "manipulation_msgs/PlaceAction"))
  (loop until (actionlib:wait-for-server *place-action-client*
                                         *place-action-timeout*))

  (roslisp:ros-info (gripper-action) "gripper action client created"))

;; NOTE most of these params have to be (vector ...)s 
(defun make-place-action-goal (point-x-object 
                               point-y-object 
                               point-z-object 
                               quaterion-value-1 
                               quaterion-value-2 
                               quaterion-value-3 
                               quaterion-value-4 
                               object-id
                               grasp-mode)
  "Creates the place-action-goal. Does not send it to the action server though."
  (actionlib:make-action-goal
      (get-place-action-client)
      ;; (cram-simple-actionlib-client::get-simple-action-client 'place-action)   
      :place_mode grasp-mode  
      :object_frame_id object-id
      :goal_pose (cl-transforms-stamped:to-msg (cl-tf::make-pose-stamped  
                                   "map"
                                   0.0 
                                   (cl-tf:make-3d-vector point-x-object
                                                         point-y-object
                                                         point-z-object ) 
                                   (cl-tf:make-quaternion quaterion-value-1
                                                          quaterion-value-2
                                                          quaterion-value-3
                                                          quaterion-value-4)))))
                                                 
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
  (roslisp:ros-warn (place-action) "Status ~a" status)
  status
  point-x-object point-y-object point-z-object
  quaterion-value-1 quaterion-value-2 quaterion-value-3 quaterion-value-4
  object-id grasp-mode
  T)


(defun call-place-action (point-x-object 
                          point-y-object 
                          point-z-object 
                          quaterion-value-1
                          quaterion-value-2
                          quaterion-value-3
                          quaterion-value-4
                          object-id
                          grasp-mode)
  "Moves the gripper to the given position.
   `pos' a vector of two values.
   First value describes +left or -right.
   Second value describes +up and -down.
   (vector 0.0 0.0) looks straight ahead."
  ;; (format t "move gripper called with pos: ~a" pos)
  (print "test")
  (multiple-value-bind (result status)
  (actionlib:call-goal (get-place-action-client)
                       (make-place-action-goal point-x-object
                                               point-y-object
                                               point-z-object 
                                               quaterion-value-1
                                               quaterion-value-2
                                               quaterion-value-3
                                               quaterion-value-4
                                               object-id grasp-mode
                                               ))
    
     (roslisp:ros-info (place-action) "place action finished")
    (ensure-place-action-goal-reached status
                                      point-x-object point-y-object point-z-object
                                      quaterion-value-1 quaterion-value-2
                                      quaterion-value-3 quaterion-value-4
                                      object-id
                                      grasp-mode)
     (values result status)))

;;NOTE 0 0 is the deafault lookig straight position.
(defun test-place-action ()
  "A function to test the place action movement."
  (call-place-action 1 1 1 5 5 5 5 "Test" 1))
