(in-package :llif)

(defun listener ()
  (with-ros-node ("listener" :spin t)
    (subscribe "/perception_output/planning" "std_msgs/String" #'print)))
