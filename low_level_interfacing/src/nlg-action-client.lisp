(in-package :llif)

(defparameter *nlg-action-timeout* 30.0 "in seconds")
(defparameter *nlg-action-client* nil)

;;@autho Torge Olliges
(defun get-nlg-action-client ()
"returns the currently used nlg action client. If none yet exists, creates one."
    (or *nlg-action-client*
        (init-nlg-action-client)))

;;@autho Torge Olliges
(defun init-nlg-action-client ()
"initializes the nlg-action-client and makes sure it is connected to the action server."
    (setf *nlg-action-client*
        (actionlib:make-action-client "nlg_requests"
                                      "nlg_msgs/LanguageGenerationAction"))
    (loop until (actionlib:wait-for-server *nlg-action-client*
                                           *nlg-action-timeout*))
    (roslisp:ros-info (nlg-action-client) "nlg action client created"))

;;@autho Torge Olliges
(defun make-nlg-action-goal (key-value-pairs)
    "Creates the make-plan-action-goal."  
    (actionlib:make-action-goal
        (get-nlg-action-client)
        :key_value_pairs (make-array (length key-value-pairs)
        :initial-contents key-value-pairs)))

;;@autho Torge Olliges
(defun key-value-list->key-value-array (list)
  (make-array (length list) :initial-contents list))

;;@autho Torge Olliges
(defun call-nlg-action (key-value-pairs)
""
    (multiple-value-bind (result status)
    (actionlib:call-goal (get-nlg-action-client)
                         (make-nlg-action-goal key-value-pairs))
    (roslisp:ros-info (nlg-action-client) "language generation action finished")
    (values result status)))

;;@autho Torge Olliges
(defun call-nlg-action-with-list (key-value-pair-lists-list)
    (call-nlg-action (mapcar 
        (lambda (key-value-list) 
            (make-key-value-msg 
                (nth 0 key-value-list) 
                (nth 1 key-value-list))) 
        key-value-pair-lists-list)))

;;@autho Torge Olliges
(defun make-key-value-msg (key value)
    (roslisp:make-msg "nlg_msgs/KeyValuePair" :key key :value value))

;;@autho Torge Olliges
(defun get-string-from-nlg-result (nlg-result)
    (roslisp:with-fields 
        (data) 
        (roslisp:with-fields 
            (generated_sentence) 
            nlg-result
            generated_sentence)
        data))

;;@autho Torge Olliges
(defun call-nlg-action-simple (key value)
    (get-string-from-nlg-result
        (call-nlg-action 
            (key-value-list->key-value-array 
                (list 
                    (make-key-value-msg key value))))))
