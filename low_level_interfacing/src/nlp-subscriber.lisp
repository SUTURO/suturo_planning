(in-package :llif)

(defun static-command-listener ()
    (with-ros-node ("command_listener" :spin t)
    (subscribe "/suturo_speech_recognition/hard_commands"  "nlp_msgs/StaticCommand" #'print)))

(defun dynamic-command-listener ()
    (with-ros-node ("command_listener" :spin t)
    (subscribe "/suturo_speech_recognition/dynamic_commands"  "nlp_msgs/DynamicCommand" #'print)))