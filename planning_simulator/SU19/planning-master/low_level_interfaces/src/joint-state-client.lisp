;;; Joint state client for the wrist flex joint.
;;; Can be used as a "start the demo" sign/button.
;;; Is currently not included in the demo. (It's commented out)


(in-package :lli)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; START SIGNAL FOR THE DEMO ;;
(defparameter *start-signal-fluent* (cpl:make-fluent :name :gripper-joint-tilt :value NIL)
  "Fluent for detecting the start signal")
(defparameter *start-signal-sub* NIL)

(defun start-signal-callback (msg)
  (setf (cpl:value *start-signal-fluent*)
        (aref
            (sensor_msgs-msg:position msg)
            (position "wrist_flex_joint"
                      (coerce (sensor_msgs-msg:name msg) 'list)
                      :test #'string=))))

(defun init-gripper-tilt-fluent ()
  (setf *start-signal-sub*
        (subscribe "hsrb/robot_state/joint_states"
                   "sensor_msgs/JointState" 
                   #'start-signal-callback
                   :max-queue-length 1))
  (cpl:wait-for *start-signal-fluent*))
;; START SIGNAL FOR THE DEMO ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defvar *robot-joints* (cpl:make-fluent :name :robot-joints) "Current joint states of the robot")
(defparameter *joints-sub* nil)

(defun joints-callback (msg)
  (setf (cpl:value *robot-joints*) msg))

(defun init-joint-fluent ()
  (setf *joints-sub*
        (subscribe "hsrb/robot_state/joint_states"
                   "sensor_msgs/JointState" 
                   #'joints-callback
                   :max-queue-length 1)))

(defun get-current-joint-state (joint-name)
  (flet ((fl-joint-state (fluent)
           (aref
            (sensor_msgs-msg:position fluent)
            (position joint-name
                      (coerce (sensor_msgs-msg:name fluent) 'list)
                      :test #'string=))))
    (cpl:value (cpl:fl-funcall #'fl-joint-state *robot-joints*))))


