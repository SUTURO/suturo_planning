(in-package :comf)

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


(defun points-around-point (distance point amountAlternatePositions turn)
(setf *currentOrigin* (cl-tf::origin (cl-tf::transform-stamped->pose-stamped ;;new line ..
					      (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))))
        (setf *goalOrigin* (cl-tf::origin point))
	(setf *newgoalOrigin* (cl-tf::v+ *goalOrigin* 
		(cl-tf::v* 
		  (cl-tf::v- *currentOrigin* *goalOrigin*) 
		  (/ distance (cl-tf::v-dist *currentOrigin* *goalOrigin*)))))

        ;;;;;; Alternative Positions in Circle around Point ;;;;;;;;;

	(setf *radians* (mapcar (lambda (listelem) (* (/ 6.28319 amountAlternatePositions) listelem)) 
		                         (loop :for n :from 1 :below amountAlternatePositions :collect n)))


        (roslisp:ros-info (poi-subscriber) "radians: ~a" *radians*)

	(setf *alternatePositions* (mapcar (lambda (listelem) (
		
			cl-tf:make-pose-stamped
		                "map"
		                (roslisp::ros-time)
		                (cl-tf::make-3d-vector 
                                  (+ (* distance (cos listelem)) (cl-tf::x *goalOrigin*)) 
                                  (+ (* distance (sin listelem)) (cl-tf::y *goalOrigin*))
                                  0)
		                (cl-tf:euler->quaternion :ax 0.0 :ay 0.0 :az (
                                      let* (
                                           (y (- (cl-tf::y *goalOrigin*) (+ (* distance (sin listelem)) (cl-tf::y *goalOrigin*)))) 
                                           (x (- (cl-tf::x *goalOrigin*) (+ (* distance (cos listelem)) (cl-tf::x *goalOrigin*))))
                                           (atanValue (atan (/ y x)))
                                          )
                                         
                                       (if turn (if (< x 0) (- atanValue 1.57) (+ atanValue 1.57) ) atanValue) 
                                     ))
		        )) 
	*radians*))

       ;;(number-sequence 1 amountAlternatePositions)
       ;;(/ 6.28319 amountAlternatePositions)

       ;;(append (list *newgoalstamped* point) *alternatePositions*))

       (setf *alternatePositions* (llif::sortedStampedByDistance (cl-tf::transform-stamped->pose-stamped ;;new line ..
				(cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))
                                *alternatePositions*))
)


(defun pose-with-distance-to-points (distance points amountAlternatePositions &optional
                                     (turn (Boolean Nil)))

        (setf *Positions* (mapcar (lambda (listelem) (points-around-point distance listelem amountAlternatePositions turn)) points ))

        ;;filter points that dont work, because of the obstacle map
	(setf *Positions* (mapcar (lambda (listelem) (remove-if-not #'llif::robot-in-obstacle-stamped listelem)) *Positions* ))

	;;remove empty lists
	(setf *Positions* (mapcar (lambda (listelem) (remove-if #'null listelem)) *Positions* ))

        (setf *Positions* (flatten *Positions* ))

        (publish-msg (advertise "poi_debug" "geometry_msgs/PoseArray")
               :header (roslisp:make-msg "std_msgs/Header" (frame_id) "map" (stamp) (roslisp:ros-time) )
               :poses (make-array (length *Positions*)
                                  :initial-contents (mapcar #'cl-tf::to-msg (mapcar #'cl-tf::pose-stamped->pose
                                                            *Positions*))) )



  	(let* ((?successfull-pose (try-movement-stampedList *Positions*))
		(?desig (desig:a motion
		                (type going) 
		                (target (desig:a location
		                                 (pose ?successfull-pose))))))
        (cpl:with-failure-handling
     	 (((or common-fail:low-level-failure 
                cl::simple-error
         cl::simple-type-error)
       (e)
       (setf ?successfull-pose (try-movement-stampedList *Positions*))
       (cpl:do-retry going-retry
         (roslisp:ros-warn (move-fail)
                                 "~%Failed to go to Point~%")
         (cpl:retry))))

	    
	    (with-hsr-process-modules
	      (exe:perform ?desig))))
        (if turn (llif::call-take-pose-action 2)))



(cpl:def-cram-function scan-object ()
	(llif::insert-knowledge-objects(get-confident-objects))
	)

(cpl:def-cram-function move-to-poi-and-scan ()
	(move-to-poi)
	(scan-object)
)


(defun flatten (l)
  (cond ((null l) nil)
        ((atom l) (list l))
        (t (loop for a in l appending (flatten a))))
)


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
