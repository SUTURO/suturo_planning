(in-package :comf)

(defparameter *poiDistance* 0.5)
(defvar *pose*)

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

(defun create-move-position-list(object-id)
    (setq *pose* (llif:prolog-object-pose object-id))
    (let 
  	((?nav-pose (list (cl-tf::make-pose-stamped "map" 0 
                                                (cl-tf:make-3d-vector
                                                 (+ (nth 0 (nth 2 *pose*)) 0.5) ;;x-cordinate
                                                 (nth 1 (nth 2 *pose*))  ;;y-cordinate
                                                 0) 
                                               (cl-tf::make-quaternion 0 0 0 1)) 
                     (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector
                                                (- (nth 0 (nth 2 *pose*)) 0.5) 
                                                (nth 1 (nth 2 *pose*))
                                                1) 
                                                (cl-tf::make-quaternion 0 0 0 1))
                     (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector
                                                (nth 0 (nth 2 *pose*))
                                                (+ (nth 1 (nth 2 *pose*)) 0.5)
                                                3)
                                               (cl-tf::make-quaternion 0 0 0 1))
                     (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector
                                                (nth 0 (nth 2 *pose*))
                                                (- (nth 1 (nth 2 *pose*)) 0.5)
                                                4)
                                               (cl-tf::make-quaternion 0 0 0 1))
                     )))
        ?nav-pose
      ))
