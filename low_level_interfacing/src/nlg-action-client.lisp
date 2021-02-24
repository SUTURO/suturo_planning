(in-package :llif)

(defparameter *nlg-action-timeout* 30.0 "in seconds")
(defparameter *nlg-client* NIL)

(defun get-nlg-action-client ()
  "returns the currently used nlg action client. If none yet exists,
   creates one."
  (or *nlg-client*
      (init-nlg-action-client)))

(defun init-nlg-action-client ()
  "initializes the nlg-action-client and makes sure it is connected to the
action server."
  (setf *nlg-client*
        (actionlib:make-action-client "nlg_requests" ;;TODO
                                      "nlg_msgs/LanguageGenerationAction"))
  (loop until (actionlib:wait-for-server *nlg-client*
                                         *nlg-action-timeout*))

  (roslisp:ros-info (nlg-action-client) "make plan action client created"))


(defun make-nlg-action-goal (key-value-pairs)
  "Creates the nlg-action-goal."
  (actionlib:make-action-goal
      (get-nlg-action-client)
    :key_value_pairs key-value-pairs))

(defun call-nlg-action (key-value-pairs)
  ""
   (multiple-value-bind (result status)
    (actionlib:call-goal (get-nlg-action-client)
                         (make-nlg-action-goal key-value-pairs))
    (roslisp:ros-info (nlg-action-client) "make plan action finished")
    (values result status)))


(defun make-value-msg (key value)
    (roslisp:make-msg "nlg_msgs/KeyValuePair" :key key :value value))
