(in-package :llif)

(defvar *state-fluent* (cram-language:make-fluent :name :state-fluent) nil)

(defun static-command-listener ()
    (with-ros-node ("command_listener" :spin t)
    (subscribe "/suturo_speech_recognition/hard_commands"  "nlp_msgs/StaticCommand" #'set-state-fluent)))

(defun dynamic-command-listener ()
    (with-ros-node ("command_listener" :spin t)
    (subscribe "/suturo_speech_recognition/dynamic_commands"  "nlp_msgs/DynamicCommand" #'print)))

(defun set-state-fluent (msg)
    "Callback for static-commands, called by static-command-listener."
    (if (eq msg 1)
        (setf (cram-language:value *state-fluent*) nil)
        (setf (cram-language:value *state-fluent*) msg)))