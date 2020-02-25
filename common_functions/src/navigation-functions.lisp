(in-package :comf)

(defparameter *poiDistance* 0.5)
(defparameter *currentOrigin* Nil)
(defparameter *goalOrigin* Nil)
(defparameter *newgoalOrigin* Nil)
(defparameter *orientation* Nil)
(defparameter *newgoalstamped* Nil)
;;difference between defvar and defparemeter do you really need defvar?
(defvar *pose*)


;;(cpl:def-cram-function move-to-poi ()
(defun move-to-poi ()
        ;;Point to go is: goal + (poiDistance/distance)*(currentpose - goal)
	;;please indent region...
	
        (move-with-distance-to-point *poiDistance* (llif::closestPoi
					  (cl-tf::transform-stamped->pose-stamped
					   (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))))
        (llif::call-take-pose-action 2)
)


(defun move-with-distance-to-point (distance point)

        (setf *currentOrigin* (cl-tf::origin (cl-tf::transform-stamped->pose-stamped ;;new line ..
					      (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))))
        (setf *goalOrigin* (cl-tf::origin point))
	(setf *newgoalOrigin* (cl-tf::v+ *goalOrigin* 
		(cl-tf::v* 
		  (cl-tf::v- *currentOrigin* *goalOrigin*) 
		  (/ distance (cl-tf::v-dist *currentOrigin* *goalOrigin*)))))

	;;what is 1.57 where does it come from? its the half of PI
	(setf *orientation* (+ 1.57 (atan (/ (cl-tf::y *goalOrigin*) (cl-tf::x *goalOrigin*)))))
        (setf *newgoalstamped* (cl-tf:make-pose-stamped
		                "map"
		                (roslisp::ros-time)
		                *newgoalOrigin*
		                (cl-tf:euler->quaternion :ax 0.0 :ay 0.0 :az *orientation*)))

        (roslisp:ros-info (poi-subscriber) "going to: ~a" *newgoalstamped*)

	(llif::call-nav-action-ps *newgoalstamped*))

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
                     ))) ;;bracket :(
        ?nav-pose
      ))  ;;bracket :(

(defun move-to-table ()
        (setf *currentOrigin* (cl-tf::origin (cl-tf::transform-stamped->pose-stamped
					      (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))))
        (setf *tablePose* Nil) ;; insert knowledge function for getting table pose
        ;; add table-width to goal to insert distance (-x)
        (llif::call-nav-action-ps *tablePose*))

(defun move-to-shelf()
        (setf *currentOrigin* (cl-tf::origin (cl-tf::transform-stamped->pose-stamped
					      (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))))
        (setf *shelfPose* Nil) ;; insert knowledge function for getting shelf pose
        ;; add shelf-depth to goal to insert distance (+y)
        (llif::call-nav-action-ps *shelfPose*))
