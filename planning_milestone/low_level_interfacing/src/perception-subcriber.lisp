(in-package :llif)

(defun listener ()
  (with-ros-node ("listener" :spin t)
    (subscribe "/perception_pipeline/result_advertiser" "perception_msgs/RSObjectDescriptions" #'print)))
