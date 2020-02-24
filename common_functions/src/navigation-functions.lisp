(in-package :comf)

(defparameter *poiDistance* 0.5)
(defparameter *currentOrigin* Nil)
(defparameter *goalOrigin* Nil)
(defparameter *newgoalOrigin* Nil)
(defparameter *orientation* Nil)
(defparameter *newgoalstamped* Nil)

;;(cpl:def-cram-function move-to-poi ()
(defun move-to-poi ()
        ;;Point to go is: goal + (poiDistance/distance)*(currentpose - goal)
        (setf *currentOrigin* (cl-tf::origin (cl-tf::transform-stamped->pose-stamped (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))))
        (setf *goalOrigin* (cl-tf::origin(llif::closestPoi (cl-tf::transform-stamped->pose-stamped (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint")))))
	(setf *newgoalOrigin* (cl-tf::v+ *goalOrigin* 
		(cl-tf::v* 
		  (cl-tf::v- *currentOrigin* *goalOrigin*) 
		  (/ *poiDistance* (cl-tf::v-dist *currentOrigin* *goalOrigin*)))))

	
	(setf *orientation* (+ 1.57 (atan (/ (cl-tf::y *goalOrigin*) (cl-tf::x *goalOrigin*)))))
        (setf *newgoalstamped* (cl-tf:make-pose-stamped
		                "map"
		                (roslisp::ros-time)
		                *newgoalOrigin*
		                (cl-tf:euler->quaternion :ax 0.0 :ay 0.0 :az *orientation*)))

        (roslisp:ros-info (poi-subscriber) "going to: ~a" *newgoalstamped*)

	(llif::call-nav-action-ps *newgoalstamped*)

	(llif::call-take-pose-action 2)
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
