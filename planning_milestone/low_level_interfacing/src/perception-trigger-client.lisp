(in-package :llif)

(defun trigger-perception-pipeline-main()
  "Trigger perception pipeline"
  (with-ros-node ("perception_client")
		 (if (not (wait-for-service "perception_pipeline" 10))
		     (ros-warn nil "Timed out out waiing for service perception_pipeline")))
		   (if (call-service "perception_pipeline" 'TriggerRequest)
		       (ros-info nil "Perception pipeline was triggered...")))
