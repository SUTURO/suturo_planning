(in-package :comf)
(defvar *place-list* NIL)
(defparameter *listOfPoi* NIL)
(defparameter *poiDistance* 0.75)
(defparameter *perception-objects* NIL)

;;;; Navigation ;;;;

;;@author Torge Olliges, Phillip Klein
;;Tries a list of stamped poses in the bulletworld simulation
;;returns a possible pose.
(defun try-movement-stampedList (listStamped)
    (let ((?nav-pose listStamped))
        (urdf-proj::without-top-level-simulated-robot
        (cpl:with-retry-counters ((going-retry 3))
            (cpl:with-failure-handling
                (((or common-fail:low-level-failure 
                      cl::simple-error
                      cl::simple-type-error)
                (e)
                (setf ?nav-pose (cdr ?nav-pose))
                (setf listStamped (cdr ?nav-pose))
                (cpl:do-retry going-retry
                    (roslisp:ros-warn 
                        (going-demo movement-fail)
                        "~%Failed to move to given position~%")
                    (cpl:retry))
            (roslisp:ros-warn 
                (going-demo movement-fail)
                "~%No more retries~%")))
            (let ((?actual-nav-pose (car ?nav-pose))) 
                (cram-executive:perform
                (desig:an action
                    (type going)
                    (target 
                        (desig:a location 
                            (pose ?actual-nav-pose)))))
                (setf listStamped (cdr ?nav-pose))
                ?actual-nav-pose))))))


;;@author Torge Olliges
;;Lets the robot move to the given position with a motion designator    
(defun move-hsr (nav-goal-pose-stamped)
    (let* ((?successfull-pose 
        (try-movement-stampedList 
				    (list ;;TODO: make this a list from a parameter?
                (cl-tf::make-pose-stamped "map" 0 
                    (cl-tf:make-3d-vector 2 3 0) 
                    (cl-tf::make-quaternion 0 0 0 1)) 
                    (cl-tf::make-pose-stamped "map" 0 
                        (cl-tf:make-3d-vector 2 3 0) 
                        (cl-tf::make-quaternion 0 0 0 1))
		            nav-goal-pose-stamped)))
        (?desig 
            (desig:a motion
                (type going) 
                (target 
                    (desig:a location
                        (pose ?successfull-pose))))))
        (exe:perform ?desig)))


;;;; Grasp ;;;;;
;;@author Jan Schimpf
;; gets object id and the grasp-pose takes care of some of the failure-handling for grasp
;; if grasp fails, toya will get into a position to trigger perception again to update
;; the position of objects and then retries with the new information
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
        (roslisp:ros-warn 
            (going-demo movement-fail)
            "~%No more retries~%")))
    (comf::grasp-object object-id grasp-pose))))


;;@author Jan Schimpf
;; the failure handling for the grasp-hsr function
(defun failure-grasp ()
    (comf::move-to-table t)
    (llif::call-take-pose-action 2) 
        (setf *perception-objects* (llif::call-robosherlock-object-pipeline (vector "robocup_table") t))
        (llif::insert-knowledge-objects *perception-objects*)
        (llif::call-take-pose-action 1))


;;;; Place ;;;;
;;@author Jan Schimpf
;; gets object id and the grasp-pose takes care of some of the failure-handling for place
;; currently retries with different position in case of a failure.
;;TODO: more comments inline pls
(defun place-hsr (object-id grasp-pose)
    (let ((?place-position (comf:create-place-list object-id grasp-pose)))
    (cpl:with-retry-counters ((place-retry 4))
        (cpl:with-failure-handling
            (((or common-fail:low-level-failure 
                  cl::simple-error
                  cl::simple-type-error)
        (e)
        (setf ?place-position (cdr ?place-position))
        (cpl:do-retry place-retry
            (roslisp:ros-warn (place-fail)
                                  "~%Failed to grasp the object~%")
            (cpl:retry))
        (let ((?actual-place-position (car ?place-position)))
        (comf::place-object-list ?actual-place-position))))))))
   

;;@author Philipp Klein
(defun move-to-poi ()
    ;;Point to go is: goal + (poiDistance/distance)*(currentpose - goal)
	  ;;please indent region...
	  (roslisp:ros-info (move-poi) "Move to POI started")
        (setf *listOfPoi* 
            (llif::sortedPoiByDistance
            (roslisp::with-fields (translation)
                (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint")
                translation)))
    (pose-with-distance-to-points *poiDistance* *listOfPoi* 10 t))

;;@author Philipp Klein
;;TODO: scan-objects as name? why do we need this as a function?
(defun scan-object ()
	  (llif::insert-knowledge-objects(get-confident-objects)))


;;@author Philipp Klein
(defun move-to-poi-and-scan ()
	  (move-to-poi)
	  (scan-object)
	  (llif::call-take-pose-action 1)
	  (roslisp::with-fields (translation rotation) (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint")
	      (llif::call-nav-action-ps 
            (cl-tf::make-pose-stamped "map" 0 translation 
                (cl-tf::q* rotation 
                    (cl-tf::euler->quaternion :ax 0 :ay 0 :az 1.57))))))


;;@author Tom-Eric Lehmkuhl
(defun move-to-table (turn)
    (roslisp:ros-info (move-poi) "Move to table started")
    ;;(defparameter *goalPose* nil)  
    (defparameter *postion* nil)                                            
    (let* ((*tablePose* (llif::prolog-table-pose))) 
    (let* ((?goal-pose (cl-tf::make-pose-stamped "map" 0
                (cl-tf::make-3d-vector
                    (- (first *tablePose*) 1.2) ;;0.7 was previously 0.95
                    (+ (second *tablePose* ) 0.1)
                    0)
                (if turn
                    (cl-tf::make-quaternion 0 0 0.39 0.91)
                    (cl-tf::make-quaternion 0 0 -0.3 0.93))))
        ;;(?goal-pose (try-movement-stampedList (list ?goal-pose)))
        (?desig (desig:a motion
                    (type going) 
                    (target (desig:a location 
                    (pose ?goal-pose))))))
        (exe:perform ?desig))))


;;@author Tom-Eric Lehmkuhl
(defun move-to-shelf (turn)
    (roslisp:ros-info (move-poi) "Move to shelf started")  
    (defparameter *postion* nil)                                            
    (let* ((*shelfPose* (first (first (llif::prolog-shelf-pose))))) 
    ;; add shelf-depth to goal to insert distance (+y)
    (let* ((?goal-pose (cl-tf::make-pose-stamped "map" 0
        (cl-tf::make-3d-vector
            (- (first *shelfPose*) 0.03) ;;was previously 0.225
            (- (second *shelfPose*) 0.7) 0)
        (if turn
            (cl-tf::make-quaternion 0 0 -1 0)
            (cl-tf::make-quaternion 0 0 0.7 0.7))))
        ;;(?goal-pose (try-movement-stampedList (list ?goal-pose)))
        (?desig (desig:a motion
                    (type going) 
                    (target (desig:a location
                               (pose ?goal-pose))))))  
      (exe:perform ?desig))))

;;@author Tom-Eric Lehmkuhl
(defun move-to-bucket ()
    (roslisp:ros-info (move-poi) "Move to bucket started")  
    (defparameter *postion* nil)                                            
  (let* ((*bucketPose* (first (first (llif::prolog-target-pose))))) 
    ;; add shelf-depth to goal to insert distance (+y)
    (let* ((?goal-pose (cl-tf::make-pose-stamped "map" 0
        (cl-tf::make-3d-vector
            (+ (first *bucketPose*) 0.343) ;;was previously 0.225
            (- (second *bucketPose*) 0.05) 0)
            (cl-tf::make-quaternion 0 0 -1 0)))
        ;;(?goal-pose (try-movement-stampedList (list ?goal-pose)))
        (?desig (desig:a motion
                    (type going) 
                    (target (desig:a location
                               (pose ?goal-pose))))))  
	      (exe:perform ?desig))))
 


