(in-package :comf)

(defparameter *poiDistance* 1)

(cpl:def-cram-function move-to-poi ()
        ;;Point to go is: goal + (poiDistance/distance)*(currentpose - goal)
        (setf *currentOrigin* (cl-tf::origin (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint")))
        (setf *goalOrigin* (cl-tf::origin(llif::closestPoi current)))
	(setf *newgoalOrigin* (cl-tf::v+ *goalOrigin* 
		(cl-tf::v* 
		  (cl-tf::v- *currentOrigin* *goalOrigin*) 
		  (/ *poiDistance* (cl-tf::v-dist *currentOrigin* *goalOrigin*)))))

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
