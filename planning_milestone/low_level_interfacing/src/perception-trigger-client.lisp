(in-package :llif)

(defun trigger-perception-pipeline-main()
  "Trigger perception pipeline"
  (with-ros-node ("perception_client")
		 (if (not (wait-for-service "perception_pipeline/trigger" 20))
		     (ros-warn nil "Timed out out waiing for service perception_pipeline")
		   (if (call-service "perception_pipeline/trigger" 'std_srvs-srv:Trigger)
		       (ros-info nil "Perception pipeline was triggered...")))))
