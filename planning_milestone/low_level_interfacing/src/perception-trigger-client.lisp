(in-package : llif)

(defun trigger-perception-pipeline()
  "Triggers perception pipeline"
  (with-ros-node ("perception_triggerer")
    (if (not (wait-for-service ("perception_pipeline") 10))
	(ros-warn nil "Timed out waiting for service perception_pipeline")
      (if (call-service "perception_pipeline" 'TriggerRequest))
        (ros-info nil "Perception Pipeline Triggered: True" )))
