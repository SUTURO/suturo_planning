;;; Torso movement action client. Based on the HSR action server.
(in-package :lli)

(defparameter *move-torso-action-timeout* 300.0 "in seconds")

(defun init-move-torso-action-client ()
  "Initializes the Torso action client."
  (cram-simple-actionlib-client:make-simple-action-client
   'move-torso-action
   "hsrb/arm_trajectory_controller/follow_joint_trajectory"
   "control_msgs/FollowJointTrajectoryAction"
   *move-torso-action-timeout*
   :initialize-now T)
  (roslisp:ros-info (torso-action) "torso action client created"))

;; NOTE most of these params have to be (vector ...)s 
(defun make-move-torso-action-goal (&key pos vel acc eff time)
  "Creates an action goal for the torso action client."
  (actionlib:make-action-goal
      (cram-simple-actionlib-client::get-simple-action-client 'move-torso-action)
    trajectory
    (roslisp:make-message
     "control_msgs/FollowJointTrajectoryGoal"
     trajectory (roslisp:make-message
                 "trajectory_msgs/JointTrajectory"
                 joint_names (vector "arm_lift_joint"
                                     "arm_flex_joint"
                                     "arm_roll_joint"
                                     "wrist_flex_joint"
                                     "wrist_roll_joint")
                 points (vector
                         (roslisp:make-message
                          "trajectory_msgs/JointTrajectoryPoint"
                          positions pos
                          velocities vel
                          accelerations acc
                          effort eff
                          time_from_start  time))))))

(defun ensure-move-torso-goal-reached (status pos)
  (roslisp:ros-warn (move-torso) "Status ~a" status)
  status
  pos
  T)


(defun call-move-torso-action (pos vel)
  "Calls the move-torso action. Expects a `pos' pose and `vel' velocity.
   Both are vectors. See the test-move-torso function below for an example."
  (multiple-value-bind (result status)
      (cram-simple-actionlib-client::call-simple-action-client
       'move-torso-action
       :action-goal (make-move-torso-action-goal
                     :pos pos
                     :vel vel
                     :acc (vector 0.1 0.1 0.0 0.0 0.0) ;; acceleration
                     :eff (vector 0.1 0.1 0.0 0.0 0.0) ;; effort
                     :time 3.0) ;; time
       :action-timeout *move-torso-action-timeout*)
    (roslisp:ros-info (move-torso) "move torso action finished")
    (ensure-move-torso-goal-reached status pos)
    (values result status)))

;;NOTE 0 0 is the deafault lookig straight position.
;; This is a good perception hight for the table for the moment
;; up/down torso | lower arm joint | rotate arm | arm joint | wrist roll
(defun test-move-torso ()
  (call-move-torso-action (vector 0.1 -0.1 1.5 -1.5 0.0)
                                (vector 0.0 0.0 0.0 0.0 0.0)))

