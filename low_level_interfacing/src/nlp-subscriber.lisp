(in-package :llif)

(defun static-command-listener ()
    (with-ros-node ("command_listener" :spin t)
    (subscribe "static_command"  "nlp_msgs/StaticCommand" #'print)))

(defun dynamic-command-listener ()
    (with-ros-node ("command_listener" :spin t)
    (subscribe "dynamic_command"  "nlp_msgs/DynamicCommand" #'print)))