;; The navigation client which uses the HSR navigation action server.

(in-package :lli)

(defvar *nav-client* nil)

(defun init-nav-client ()
  "Initialize the navigation client"
  (unless (eq roslisp::*node-status* :running)
    (roslisp:start-ros-node "nav-action-client"))
  (setf *nav-client* (actionlib:make-action-client
                      "/move_base/move"
                      "move_base_msgs/MoveBaseAction"))
  
  (roslisp:ros-info (nav-action-client) "waiting for Navigation Action server...")

  (loop until
        (actionlib:wait-for-server *nav-client*))
  (roslisp:ros-info (nav-action-client) "Navigation action client created."))

(defun get-nav-action-client ()
  "returns the navigation action client. If none exists yet, one will be created."
  (when (null *nav-client*)
    (init-nav-client))
  *nav-client*)

(defun make-nav-action-goal (pose-stamped-goal)
  "creates a navigation action goal. Expects a `pose-stamped' as goal."
  ;; make sure a node is already up and running, if not, one is initialized here.
  (roslisp:ros-info (navigation-action-client) "make navigation action goal")
  (unless (eq roslisp::*node-status* :running)
    (roslisp:start-ros-node "navigation-action-lisp-client"))

  (actionlib:make-action-goal (get-nav-action-client)
    target_pose pose-stamped-goal))

(defun call-nav-action (x y euler-z &optional (frame-id "map"))
  "Calls the navigation action. Expected: x y coordinates within map, and
euler-z gives the rotation around the z axis."
  (unless (eq roslisp::*node-status* :running)
    (roslisp:start-ros-node "nav-action-lisp-client"))

  (multiple-value-bind (result status)
      (let* ((actionlib:*action-server-timeout* 10.0)
             (pose-stamped (cl-tf:make-pose-stamped
                        frame-id
                        (roslisp::ros-time)
                        (cl-tf:make-3d-vector x y 0.0)
                        (cl-tf:euler->quaternion :ax 0.0 :ay 0.0 :az euler-z)))
             
            (the-goal (cl-tf:to-msg pose-stamped)))
        
        (publish-marker-pose pose-stamped :g 1.0)

        (actionlib:call-goal 
         (get-nav-action-client)
         (make-nav-action-goal the-goal)))
    (roslisp:ros-info (nav-action-client) "Navigation action finished.")
    (values result status)))

(defun call-nav-action-ps (pose-stamped)
  "calls the navigation client and passes the given `pose-stamped' to it."
  (setf pose-stamped (cl-tf:copy-pose-stamped pose-stamped :origin
                                              (cl-tf:copy-3d-vector
                                               (cl-tf:origin pose-stamped)
                                               :z 0.0)))
  
  (unless (eq roslisp::*node-status* :running)
    (roslisp:start-ros-node "nav-action-lisp-client"))
  (format t "Pose: ~a " pose-stamped)
  (multiple-value-bind (result status)
      (let ((actionlib:*action-server-timeout* 20.0)
            (the-goal (cl-tf:to-msg
                       pose-stamped)))
        ;; (format t "my POSE: ~a" the-goal)

        ;;publish the pose the robot will navigate to
        (publish-marker-pose pose-stamped :g 1.0)

        (actionlib:call-goal
         (get-nav-action-client)
         (make-nav-action-goal the-goal)))
    (roslisp:ros-info (nav-action-client) "Navigation action finished.")
    (format t "result : ~a" status)
    (values result status)))

;;; NOTE !!!  DON'T USE THIS WITHOUT CONSULTING TUTORS FIRST !!!
(defun smash-into-appartment (&optional (lin 100))
  "Function to send velocity commands. Be VERY CAREFUL if you want to use this.
   Please consult tutors before usage."
  (let ((pub (roslisp:advertise "/hsrb/command_velocity" "geometry_msgs/Twist"))
        (vel-msg (roslisp:make-message "geometry_msgs/Twist" (:x :linear) lin)))
    (dotimes (i 20) 
      (publish pub
               vel-msg)
      (sleep 0.2))))
