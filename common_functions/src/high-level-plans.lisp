(in-package :comf)
(defvar *place-list* nil)
(defparameter *listOfPoi* Nil)

;;;; Navigation ;;;;
(defun try-movement () (let 
	((?nav-pose (list (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector 2 3 0) 
                                               (cl-tf::make-quaternion 0 0 0 1)) 
                     (cl-tf::make-pose-stamped "map" 0 
                                                (cl-tf:make-3d-vector 2 3 0) 
                                                (cl-tf::make-quaternion 0 0 0 1))
                     (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector 1.097 0.556 0)
                                               (cl-tf::make-quaternion 0 0 0 1)))))

(urdf-proj:with-simulated-robot (cpl:with-retry-counters ((going-retry 3))
      (cpl:with-failure-handling
          (((or common-fail:low-level-failure 
                cl::simple-error
                cl::simple-type-error)
               (e)
             
             (setf ?nav-pose (cdr ?nav-pose))
             
             (cpl:do-retry going-retry
               (roslisp:ros-warn (going-demo movement-fail)
                                 "~%Failed to move to given position~%")
               (cpl:retry))
             
             (roslisp:ros-warn (going-demo movement-fail)
                               "~%No more retries~%")))
        
          (let ((?actual-nav-pose (car ?nav-pose))) 
          (cram-executive:perform
           (desig:an action
                     (type going)
                     (target (desig:a location (pose ?actual-nav-pose)))))
            ?actual-nav-pose))))))


;;;;;;;;;;;;;;;;;;;;Try Movement with List ;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun try-movement-stampedList (listStamped)
  (let ((?nav-pose listStamped))

    (urdf-proj:with-simulated-robot
      (cpl:with-retry-counters ((going-retry 3))
        (cpl:with-failure-handling
            (((or common-fail:low-level-failure 
                  cl::simple-error
                  cl::simple-type-error)
                 (e)
               
               (setf ?nav-pose (cdr ?nav-pose))
             
               (cpl:do-retry going-retry
                 (roslisp:ros-warn (going-demo movement-fail)
                                   "~%Failed to move to given position~%")
                 (cpl:retry))
             
               (roslisp:ros-warn (going-demo movement-fail)
                                 "~%No more retries~%")))
        
          (let ((?actual-nav-pose (car ?nav-pose))) 
          (cram-executive:perform
           (desig:an action
                     (type going)
                     (target (desig:a location (pose ?actual-nav-pose)))))
            ?actual-nav-pose))))))

(defun try-movement-with-points-around-robot-list (listStamped)   )
;;Für jede Position werden positionen davon im umkreis von der breite vom roboter berechnet wenn alle ereichbar sind, wird die position zurück gebeben



          
(defun move-hsr ()
  (let* ((?successfull-pose (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector 1.097 0.556 0)
                                               (cl-tf::make-quaternion 0 0 0 1)))
        (?desig (desig:a motion
                        (type going) 
                        (target (desig:a location
                                         (pose ?successfull-pose))))))
    
    (with-hsr-process-modules
      (exe:perform ?desig))
))

;;;; Grasp ;;;;;
;;@author Jan Schimpf
(defun grasp-hsr (object-id grasp-pose)    
  (cpl:with-retry-counters ((grasping-retry 3))
   (cpl:with-failure-handling
      (((or common-fail:low-level-failure 
                cl::simple-error
         cl::simple-type-error)
       (e)
         (comf::scan-object
          ;;closet function i currently know of that allows me
          ;;to trigger perception for this
          )
       (cpl:do-retry grasping-retry
         (roslisp:ros-warn (grasp-fail)
                                 "~%Failed to grasp the object~%")
         (cpl:retry))

         (roslisp:ros-warn (going-demo movement-fail)
                           "~%No more retries~%")))
     (comf::grasp-object object-id grasp-pose))))

;;;; Place ;;;;
;;@author Jan Schimpf

(defun place-hsr (object-id grasp-pose)
  (let ((?place-position (comf:create-place-list object-id grasp-pose)))
                         
 (cpl:with-retry-counters ((grasping-retry 4))
   (cpl:with-failure-handling
      (((or common-fail:low-level-failure 
                cl::simple-error
         cl::simple-type-error)
       (e)
       (setf ?place-position (cdr ?place-position))
       (cpl:do-retry grasping-retry
         (roslisp:ros-warn (grasp-fail)
                                 "~%Failed to grasp the object~%")
         (cpl:retry))
         (let ((?actual-place-position (car ?place-position)))
         (comf::place-object ?actual-place-position)
                       )))))))
   

;;;;Move-to-Grasp;;;;
;;@author Jan Schimpf
(defvar *obj-pos*)

(defun move-to-grasp (object-id grasp-mode)
  (let ((?move-positions (create-move-position-list object-id)))
   (cpl:with-retry-counters ((grasping-retry 4))
   (cpl:with-failure-handling
      (((or common-fail:low-level-failure 
                cl::simple-error
         cl::simple-type-error)
       (e)
       (setf ?move-positions (cdr ?move-positions))
       (cpl:do-retry grasping-retry
         (roslisp:ros-warn (grasp-fail)
                                 "~%Failed to grasp the object~%")
         (cpl:retry))
         (let ((?successfull-pose (try-movement-stampedList ?move-positions)))
           
           (comf::looking *obj-pos*)
           (comf::detecting)
           (llif::with-hsr-process-modules (exe:perform
                                            (desig:a motion (type going)
                                                     (target (desig:a location
                                                                      (pose ?successfull-pose))))))
           (comf::grasp-hsr object-id grasp-mode)
         )))))))

;;@author Philipp Klein
(defun move-to-poi ()
        ;;Point to go is: goal + (poiDistance/distance)*(currentpose - goal)
	;;please indent region...
	(roslisp:ros-info (move-poi) "Move to POI started")
        (setf *listOfPoi* (mapcar (lambda (listelem) (comf::pose-with-distance-to-point *poiDistance* listelem)) 
                                 (llif::sortedPoiByDistance
					(cl-tf::transform-stamped->pose-stamped
					   (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint")))))
        ;;(roslisp:ros-info (poi-subscriber) "Gefundene POI sortiert nach entfernung: ~a" *listOfPoi*)
	(roslisp:ros-info (move-poi) "Move to POIs: ~a"  *listOfPoi*)
	(let* ((?successfull-pose (try-movement-stampedList *listOfPoi*))
		(?desig (desig:a motion
		                (type going) 
		                (target (desig:a location
		                                 (pose ?successfull-pose))))))
	    
	    (with-hsr-process-modules
	      (exe:perform ?desig)))
        (llif::call-take-pose-action 2)
)


;;@author Tom-Eric Lehmkuhl
(defun move-to-table ()
        (roslisp:ros-info (move-poi) "Move to table started")
        ;;(defparameter *goalPose* nil)  
        (defparameter *postion* nil)                                            
        (defparameter *tablePose* (llif::prolog-table-pose)) ;; insert knowledge function for getting table pose
        ;;(roslisp::with-fields (x y z) *tablePose* (setf *postion* (cl-tf::make-3d-vector (x y z))))

        (let* ((?goal-pose (cl-tf::make-pose-stamped "map" 0
                            (cl-tf::make-3d-vector (+ (first *tablePose*) 0.95) 
                              (second *tablePose*) (third *tablePose*)) (cl-tf::make-quaternion 0 0 1 0)))
         (?desig (desig:a motion
		                  (type going) 
		                  (target (desig:a location
		                                   (pose ?goal-pose))))))
	    
	       (with-hsr-process-modules
	        (exe:perform ?desig))))

;;@author Tom-Eric Lehmkuhl
(defun move-to-shelf()
        (roslisp:ros-info (move-poi) "Move to shelf started")  
        (defparameter *postion* nil)                                            
        (defparameter *shelfPose* (llif::prolog-shelf-pose)) ;; insert knowledge function for getting shelf pose
        
        ;; add shelf-depth to goal to insert distance (+y)
        (let* ((?goal-pose (cl-tf::make-pose-stamped "map" 0
                            (cl-tf::make-3d-vector (first *tablePose*) 
                              (- (second *tablePose*) 0.36) (third *tablePose*)) (cl-tf::make-quaternion 0 0 -0.7 0.7)))
         (?desig (desig:a motion
		                  (type going) 
		                  (target (desig:a location
		                                   (pose ?goal-pose))))))
	    
<<<<<<< HEAD
	       (with-hsr-process-modules
	        (exe:perform ?desig))))
 
=======
	      (with-hsr-process-modules
	      (exe:perform ?desig))) 
>>>>>>> cf28a8bf02af43bbe7099ea49b1c25a76114cb0b
