(in-package :llif)

(defun listener ()
  (with-ros-node ("listener" :spin t)
    (subscribe "/perception_pipeline/result_advertiser" "robosherlock_msgs/RSObjectDescriptions" #'print)))
