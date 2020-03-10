(in-package :comf)
(defvar *place-list* nil)
(defparameter *listOfPoi* Nil)
(defparameter *poiDistance* 0.75)
(defparameter *perception-objects* NIL)

;;;; Navigation ;;;;
;;@author Torge Olliges 
(defun try-movement () 
(print "Try Movement startet")
(let ((?nav-pose (list (cl-tf::make-pose-stamped "map" 0 
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
             (print "Try movement Failed")
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
;;@author Torge Olliges, Phillip Klein
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
               (setf listStamped (cdr ?nav-pose))
             
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
            (setf listStamped (cdr ?nav-pose))
            ?actual-nav-pose))))))


(defun try-move-list (listStamped)
    (remove-if #'null (mapcar (lambda (listelem) (try-movement-stamped listelem)) listStamped))
)

;;Für jede Position werden positionen davon im umkreis von der breite vom roboter berechnet wenn alle ereichbar sind, wird die position zurück gebeben



;;@author Torge Olliges      
(defun move-hsr (nav-goal-pose-stamped)
  (let* ((?successfull-pose (try-movement-stampedList 
				(list (cl-tf::make-pose-stamped "map" 0 
                                      	(cl-tf:make-3d-vector 2 3 0) 
                                      	(cl-tf::make-quaternion 0 0 0 1)) 
                     		      (cl-tf::make-pose-stamped "map" 0 
                                   	(cl-tf:make-3d-vector 2 3 0) 
                                        (cl-tf::make-quaternion 0 0 0 1))
		      		      nav-goal-pose-stamped))) ;;@Jan fixed? das war nav-goal-pose zuvor, also ein paar klammer probleme behoben
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
         (failure-grasp)
       (cpl:do-retry grasping-retry
         (roslisp:ros-warn (grasp-fail)
                                 "~%Failed to grasp the object~%")
         (cpl:retry))

         (roslisp:ros-warn (going-demo movement-fail)
                           "~%No more retries~%")))
    (comf::grasp-object object-id grasp-pose))))

(defun failure-grasp ()
            (comf::move-to-table t)
            (llif::call-take-pose-action 2) 
            (setf *perception-objects* (llif::call-robosherlock-object-pipeline (vector "robocup_table") t))
            (llif::insert-knowledge-objects *perception-objects*)
            (llif::call-take-pose-action 1)
  )

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
         (comf::place-object-test ?actual-place-position)
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

(defun reachable-in-simulator (stamp) 
    (null (try-movement-stampedlist (list stamp)))
)

;;@author Philipp Klein
(defun move-to-poi ()
        ;;Point to go is: goal + (poiDistance/distance)*(currentpose - goal)
	;;please indent region...
	(roslisp:ros-info (move-poi) "Move to POI started")
        (setf *listOfPoi* 
                          (llif::sortedPoiByDistance
					(roslisp::with-fields (translation) (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint") translation)))

        (pose-with-distance-to-points *poiDistance* *listOfPoi* 10 t) 

	;;(roslisp:ros-info (move-poi) "Move to POIs: ~a"  *listOfPoi*)


	;;(setf *rangetest* (loop for x from -2 to 3 by 0.1
	;;      collect (loop for y from -2 to 3 by 0.1
        ;;              collect (
        ;;                       cl-tf::make-pose
        ;;                       (cl-tf::make-3d-vector x y 0.1)
        ;;                      (cl-tf::make-quaternion 0 0.7 0 0.7 ))) ))

        ;;(setf *li* (remove-if-not #'llif::robot-in-obstacle-stamped (flatten (copy-list *rangetest*))))
	;;(publish-msg (advertise "range" "geometry_msgs/PoseArray")
        ;;       :header (roslisp:make-msg "std_msgs/Header" (frame_id) "map" (stamp) (roslisp:ros-time) )
        ;;       :poses (make-array (length *li*)
        ;;                          :initial-contents (mapcar #'cl-tf::to-msg *li*)))

        ;;(setf *li1* (flatten (mapcar (lambda (listelem) (remove-if #'llif::robot-in-obstacle-stamped listelem)) *listOfPoi* )))
	;;(publish-msg (advertise "removed" "geometry_msgs/PoseArray")
        ;;       :header (roslisp:make-msg "std_msgs/Header" (frame_id) "map" (stamp) (roslisp:ros-time) )
        ;;       :poses (make-array (length *li1*)
        ;;                          :initial-contents (mapcar #'cl-tf::to-msg (mapcar #'cl-tf::pose-stamped->pose *li1*))) )


        ;;filter points that dont work, because of the obstacle map
	;;(setf *listOfPoi* (mapcar (lambda (listelem) (remove-if-not #'llif::robot-in-obstacle-stamped listelem)) *listOfPoi* ))


	;;remove empty lists
	;;(setf *listOfPoi* (mapcar (lambda (listelem) (remove-if #'null listelem)) *listOfPoi* ))


        ;;(publish-msg (advertise "poi_debug" "geometry_msgs/PoseArray")
        ;;       :header (roslisp:make-msg "std_msgs/Header" (frame_id) "map" (stamp) (roslisp:ros-time) )
        ;;       :poses (make-array (length (flatten *listOfPoi*))
        ;;                          :initial-contents (mapcar #'cl-tf::to-msg (mapcar #'cl-tf::pose-stamped->pose
        ;;                                                    (flatten *listOfPoi*)))) )

	;;liste druchgehen, zuerst im simulator testen und dann im echten, wenn beides erfolgreich => beenden sonst weiter

	;;filter each point that isnt reachable in the simulator
	;;(setf *listOfPoi* (mapcar (lambda (listelem) (try-move-list listelem)) *listOfPoi* ))

	;;remove empty lists
	;;(setf *listOfPoi* (mapcar (lambda (listelem) (remove-if #'null listelem)) *listOfPoi* ))

        ;;Limit Point per Poi to 2
        ;;(setf *listOfPoi* (mapcar (lambda (listelem) (
	;;	if (> (length listelem) 2) (subseq listelem 0 2) listelem
        ;;             )) *listOfPoi* ))

        ;;(setf *listOfPoi* (flatten *listOfPoi* ))



  	;;(let* ((?successfull-pose (try-movement-stampedList *listOfPoi*))
	;;	(?desig (desig:a motion
	;;	                (type going) 
	;;	                (target (desig:a location
	;;	                                 (pose ?successfull-pose))))))
        ;;(cpl:with-failure-handling
     	;; (((or common-fail:low-level-failure 
        ;;        cl::simple-error
        ;; cl::simple-type-error)
        ;;(e)
        ;;(setf ?successfull-pose (try-movement-stampedList *listOfPoi*))
        ;;(cpl:do-retry going-retry
        ;; (roslisp:ros-warn (move-fail)
        ;;                         "~%Failed to go to Point~%")
        ;; (cpl:retry)))))

	    
	;;    (with-hsr-process-modules
	;;      (exe:perform ?desig)))
        ;;(llif::call-take-pose-action 2)
)


;;@author Tom-Eric Lehmkuhl
(defun move-to-table (turn)
     
        (roslisp:ros-info (move-poi) "Move to table started")
        ;;(defparameter *goalPose* nil)  
        (defparameter *postion* nil)                                            
        (defparameter *tablePose* (llif::prolog-table-pose)) ;; insert knowledge function for getting table pose
        ;;(roslisp::with-fields (x y z) *tablePose* (setf *postion* (cl-tf::make-3d-vector (x y z))))

        (let* ((?goal-pose (cl-tf::make-pose-stamped "map" 0
                            (cl-tf::make-3d-vector (+ (first *tablePose*) 0.9) ;;0.7 was previously 0.95
                              (- (second *tablePose* ) 0.15) 0) (if turn (cl-tf::make-quaternion 0 0 -0.7 0.7) (cl-tf::make-quaternion 0 0 1 0))))
         ;;(?goal-pose (try-movement-stampedList (list ?goal-pose)))
         (?desig (desig:a motion
		                  (type going) 
		                  (target (desig:a location
		                                   (pose ?goal-pose))))))
	    	        (exe:perform ?desig)))

;;@author Tom-Eric Lehmkuhl
(defun move-to-shelf (turn)
        (roslisp:ros-info (move-poi) "Move to shelf started")  
        (defparameter *postion* nil)                                            
        (defparameter *shelfPose* (llif::prolog-shelf-pose)) ;; insert knowledge function for getting shelf pose
        
        ;; add shelf-depth to goal to insert distance (+y)
        (let* ((?goal-pose (cl-tf::make-pose-stamped "map" 0
                            (cl-tf::make-3d-vector (+ (first *shelfPose*) 0.1) ;;was previously 0.225
                              (+ (second *shelfPose*) 0.77) 0) (if turn (cl-tf::make-quaternion 0 0 0 1) (cl-tf::make-quaternion 0 0 -0.7 0.7))))
         ;;(?goal-pose (try-movement-stampedList (list ?goal-pose)))
         (?desig (desig:a motion
		                  (type going) 
		                  (target (desig:a location
		                                   (pose ?goal-pose))))))
	    
	        (exe:perform ?desig)))
 


