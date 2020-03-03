(in-package :comf)
(defvar *place-list* nil)
(defparameter *listOfPoi* Nil)
(defparameter *poiDistance* 0.75)

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
(defun try-movement-stampedlist (listStamped)
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
(defvar *obj-class*)
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
        (setf *listOfPoi* (mapcar (lambda (listelem) (comf::pose-with-distance-to-point *poiDistance* listelem 10)) 
                                 (llif::sortedPoiByDistance
					(cl-tf::transform-stamped->pose-stamped
					   (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint")))))

	;;(roslisp:ros-info (move-poi) "Move to POIs: ~a"  *listOfPoi*)




	(publish-msg (advertise "removed" "geometry_msgs/PoseArray")
               :header (roslisp:make-msg "std_msgs/Header" (frame_id) "map" (stamp) (roslisp:ros-time) )
               :poses (make-array (length (flatten (mapcar (lambda (listelem) (remove-if #'llif::robot-in-obstacle-stamped listelem)) *listOfPoi* )))
                                  :initial-contents (mapcar #'cl-tf::to-msg (mapcar #'cl-tf::pose-stamped->pose
                                                            (flatten (mapcar (lambda (listelem) (remove-if #'llif::robot-in-obstacle-stamped listelem)) *listOfPoi* ))))) )


        ;;filter points that dont work, because of the obstacle map
	(setf *listOfPoi* (mapcar (lambda (listelem) (remove-if-not #'llif::robot-in-obstacle-stamped listelem)) *listOfPoi* ))

	(mapcar (lambda (listelem) ( roslisp:ros-info (move-poi) "Poi with all Alternative Points: ~a"  listelem)) *listOfPoi*)


	(publish-msg (advertise "origins" "geometry_msgs/PoseArray")
               :header (roslisp:make-msg "std_msgs/Header" (frame_id) "map")
               :poses (make-array (length (llif::sortedPoiByDistance
					(cl-tf::transform-stamped->pose-stamped
					   (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))))
                                  :initial-contents (mapcar #'cl-tf::to-msg (mapcar #'cl-tf::pose-stamped->pose
                                                            (llif::sortedPoiByDistance
					(cl-tf::transform-stamped->pose-stamped
					   (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint")))))) )


        (publish-msg (advertise "poi_debug" "geometry_msgs/PoseArray")
               :header (roslisp:make-msg "std_msgs/Header" (frame_id) "map" (stamp) (roslisp:ros-time) )
               :poses (make-array (length (flatten *listOfPoi*))
                                  :initial-contents (mapcar #'cl-tf::to-msg (mapcar #'cl-tf::pose-stamped->pose
                                                            (flatten *listOfPoi*)))) )

	;;(let* ((?successfull-pose (try-movement-stampedList (flatten *listOfPoi*)))
	;;	(?desig (desig:a motion
	;;	                (type going) 
	;;	                (target (desig:a location
	;;	                                 (pose ?successfull-pose))))))
	;;    
	;;    (with-hsr-process-modules
	;;      (exe:perform ?desig)))
        ;;(llif::call-take-pose-action 2)
)


;;@author Tom-Eric Lehmkuhl
(defun move-to-table ()
        (roslisp:ros-info (move-poi) "Move to table started")
        (defparameter *goalPose* nil)  
        (defparameter *postion* nil)                                            
        (defparameter *tablePose* (llif::prolog-table-pose)) ;; insert knowledge function for getting table pose
        (roslisp::with-fields (x y z) *tablePose* (setf *postion* (cl-tf::make-3d-vector (x y z))))

        ;; add table-width to goal to insert distance (-x)
        (setf *goalPose* (cl-tf::make-pose-stamped "map" 0
                                        (cl-tf::make-3d-vector (- (cl-tf::x *postion*) 0.95) (cl-tf::y *postion*) (cl-tf::z *postion*))))

        (desig:a motion
		                (type going) 
		                (target (desig:a location
		                                 (pose *goalPose*))))
	    
	      (with-hsr-process-modules
	      (exe:perform ?desig))) 

;;@author Tom-Eric Lehmkuhl
(defun move-to-shelf()
        (roslisp:ros-info (move-poi) "Move to shelf started")
        (defparameter *goalPose* nil)  
        (defparameter *postion* nil)                                            
        (defparameter *shelfPose* (llif::prolog-table-pose)) ;; insert knowledge function for getting shelf pose
        (roslisp::with-fields (x y z) *shelfPose* (setf *postion* (cl-tf::make-3d-vector (x y z))))
        
        ;; add shelf-depth to goal to insert distance (+y)
        (setf *goalPose* (cl-tf::make-pose-stamped "map" 0
                                        (cl-tf::make-3d-vector (cl-tf::x *postion*) (+ (cl-tf::y *postion*) 0.36) (cl-tf::z *postion*))))

        (desig:a motion
		                (type going) 
		                (target (desig:a location
		                                 (pose *goalPose*))))
	    
	      (with-hsr-process-modules
	      (exe:perform ?desig))) 
