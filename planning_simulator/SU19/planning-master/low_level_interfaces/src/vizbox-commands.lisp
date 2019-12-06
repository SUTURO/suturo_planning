;;; interface to the required VizBox
(in-package :lli)

(defvar *challenge-step* nil)
(defvar *robot-text* nil)
(defvar *operator-text* nil)

(defun viz-box-init ()
  "initializes the VizBox"
  (setf *challenge-step* (roslisp:advertise "/challenge_step" "std_msgs/UInt32"))
  (setf *robot-text* (roslisp:advertise "/robot_text" "std_msgs/String"))
  (setf *operator-text* (roslisp:advertise "/operator_text" "std_msgs/String")))
        
;; 0 Initial pose
;; 1 Perceiving Shelf
;; 2 Navigate to table
;; 3 Perceiving table
;; 4 Grasp the object
;; 5 Navigate to shelf
;; 6 Place the object
(defun publish-challenge-step (step)
  "Publishes the current challange step. To see which step is what check the
   comment above"
  (roslisp:publish *challenge-step*
                   (roslisp:make-message "std_msgs/UInt32" 
                                         :data step)))

(defun publish-robot-text (text)
  "Publish the text that the robot says also on VizBox"
  (roslisp:publish *robot-text*
                   (roslisp:make-message "std_msgs/String" 
                                         :data text)))

(defun publish-operator-text (text)
  "Publish the text the operator said to the robot via VizBox"
  (roslisp:publish *operator-text*
                   (roslisp:make-message "std_msgs/String" 
                                         :data text)))
	
;; rostopic pub /challenge_step std_msgs/UInt32 "data: 0" --once
;; rostopic pub /robot_text std_msgs/String "data: 'Hello operator'" --once
;; rostopic pub /operator_text std_msgs/String "data: 'Robot, follow me'" --once
;; rostopic pub /robot_text std_msgs/String "data: 'OK, I will follow you'" --once;
;; rostopic pub /challenge_step std_msgs/UInt32 "data: 1" --once
