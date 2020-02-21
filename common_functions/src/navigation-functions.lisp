(in-package :comf)

(defparameter *poiDistance* 0.5)

;;(cpl:def-cram-function move-to-poi ()
(defun move-to-poi ()
        ;;Point to go is: goal + (poiDistance/distance)*(currentpose - goal)
        (setf *currentOrigin* (cl-tf::origin (cl-tf::transform-stamped->pose-stamped (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))))
        (setf *goalOrigin* (cl-tf::origin(llif::closestPoi (cl-tf::transform-stamped->pose-stamped (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint")))))
        (roslisp:ros-info (poi-subscriber) "going to: ~a" *goalOrigin*)
	(setf *newgoalOrigin* (cl-tf::v+ *goalOrigin* 
		(cl-tf::v* 
		  (cl-tf::v- *currentOrigin* *goalOrigin*) 
		  (/ *poiDistance* (cl-tf::v-dist *currentOrigin* *goalOrigin*)))))

	
	(setf *orientation* (+ 1.57 (atan (/ (cl-tf::y *goalOrigin*) (cl-tf::x *goalOrigin*)))))

	(llif::call-nav-action-ps(cl-tf:make-pose-stamped
		                "map"
		                (roslisp::ros-time)
		                *newgoalOrigin*
		                (cl-tf:euler->quaternion :ax 0.0 :ay 0.0 :az *orientation*)))

	;;going designator call + rotation calculation
	;;(llif::call-nav-action-ps)
)

(cpl:def-cram-function scan-object ()
	(llif::insert-knowledge-objects(get-confident-objects))
	)

(cpl:def-cram-function move-to-poi-and-scan ()
	(move-to-poi)
	(scan-object)
)
