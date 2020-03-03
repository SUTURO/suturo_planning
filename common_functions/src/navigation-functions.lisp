(in-package :comf)

(defparameter *poiDistance* 1)
(defparameter *currentOrigin* Nil)
(defparameter *goalOrigin* Nil)
(defparameter *newgoalOrigin* Nil)
(defparameter *orientation* Nil)
(defparameter *newgoalstamped* Nil)
(defparameter *radians* Nil)
(defparameter *alternatePositions* Nil)
;;difference between defvar and defparemeter do you really need defvar?
(defvar *pose*)


;;(cpl:def-cram-function move-to-poi ()
;;moved to high level


(defun pose-with-distance-to-point (distance point amountAlternatePositions)

        (setf *currentOrigin* (cl-tf::origin (cl-tf::transform-stamped->pose-stamped ;;new line ..
					      (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))))
        (setf *goalOrigin* (cl-tf::origin point))
	(setf *newgoalOrigin* (cl-tf::v+ *goalOrigin* 
		(cl-tf::v* 
		  (cl-tf::v- *currentOrigin* *goalOrigin*) 
		  (/ distance (cl-tf::v-dist *currentOrigin* *goalOrigin*)))))

	;;what is 1.57 where does it come from? its the half of PI
	(setf *orientation* (+ 1.57 (atan (/ (cl-tf::y *goalOrigin*) (cl-tf::x *goalOrigin*)))))
        
        ;;;;; Direct Position ;;;;;;;;;;;
        (setf *newgoalstamped* (cl-tf:make-pose-stamped
		                "map"
		                (roslisp::ros-time)
		                *newgoalOrigin*
		                (cl-tf:euler->quaternion :ax 0.0 :ay 0.0 :az *orientation*)))

        (roslisp:ros-info (poi-subscriber) "going to: ~a" *newgoalstamped*)

        ;;;;;; Alternative Positions in Circle around Point ;;;;;;;;;

	;;(setf *radians* (mapcar (lambda (listelem) (* (/ 6.28319 amountAlternatePositions) listelem)) 
	;;	                         (number-sequence 1 amountAlternatePositions)))


	;;(setf *alternatePositions* (mapcar (lambda (listelem) (
	;;	
	;;		(cl-tf:make-pose-stamped
	;;	                "map"
	;;	                (roslisp::ros-time)
	;;	                (cl-tf::make-3d-vector 
        ;;                          (+ (* distance (cos listelem)) (cl-tf::x *goalOrigin*)) 
        ;;                          (+ (* distance (cos listelem)) (cl-tf::x *goalOrigin*))
        ;;                          0)
	;;	                (cl-tf:euler->quaternion :ax 0.0 :ay 0.0 :az *orientation*))
	;;	        )) 
	;;*radians*))

       ;;(number-sequence 1 amountAlternatePositions)
       ;;(/ 6.28319 amountAlternatePositions)

	*newgoalstamped*)

(cpl:def-cram-function scan-object ()
	(llif::insert-knowledge-objects(get-confident-objects))
	)

(cpl:def-cram-function move-to-poi-and-scan ()
	(move-to-poi)
	(scan-object)
)

;;@Author Jan Schimpf
(defun create-move-position-list(object-id)
    (setq *pose* (llif:prolog-object-pose object-id))
    (let 
  	((?nav-pose (list (cl-tf::make-pose-stamped "map" 0 
                                                (cl-tf:make-3d-vector
                                                 (- (nth 0 (nth 2 *pose*)) 0.5) ;;x-cordinate
                                                 (nth 1 (nth 2 *pose*))  ;;y-cordinate
                                                 0) 
                                               (cl-tf::make-quaternion 0 0 0 1)) 
                     (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector
                                                (+ (nth 0 (nth 2 *pose*)) 0.5) 
                                                (nth 1 (nth 2 *pose*))
                                                3.14) 
                                                (cl-tf::make-quaternion 0 0 0 1))
                     (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector
                                                (nth 0 (nth 2 *pose*))
                                                (+ (nth 1 (nth 2 *pose*)) 0.5)
                                                1.57)
                                               (cl-tf::make-quaternion 0 0 0 1))
                     (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector
                                                (nth 0 (nth 2 *pose*))
                                                (- (nth 1 (nth 2 *pose*)) 0.5)
                                                -1.57)
                                               (cl-tf::make-quaternion 0 0 0 1))
                     ))) ;;bracket :(
        ?nav-pose
      ))  ;;bracket :(
