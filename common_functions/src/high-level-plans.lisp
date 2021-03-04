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
    (car listStamped))
;;   (let (?nav-pose listStamped)
;;     (print ?nav-pose)
;;         (cpl:with-retry-counters ((going-retry 3))
;;             (cpl:with-failure-handling
;;                 (((or common-fail:low-level-failure 
;;                       cl::simple-error
;;                       cl::simple-type-error)
;;                   (e)
;;                   (setf ?nav-pose (cdr ?nav-pose))
;;                   (setf listStamped (cdr ?nav-pose))
;;                   (cpl:do-retry going-retry
;;                     (roslisp:ros-warn 
;;                         (going-demo movement-fail) "~%Failed to move to given position~%")
;;                     (cpl:retry))
;;                   (roslisp:ros-warn  (going-demo movement-fail) "~%No more retries~%")))
;;                 (let ((?actual-nav-pose (car ?nav-pose))) 
;;                      (exe:perform
;;                        (desig:an action
;;                          (type going)
;;                          (pose ?actual-nav-pose)))
;;                      (print ?actual-nav-pose)
;;                      (setf listStamped (cdr ?nav-pose))
;;                  ?actual-nav-pose)))))


;;@author Torge Olliges
;;Lets the robot move to the given position with a motion designator    
(defun move-hsr (nav-goal-pose-stamped)
    (let* ((?successfull-pose 
        (try-movement-stampedList 
				    (list 
             nav-goal-pose-stamped))))
      (exe:perform 
            (desig:a motion
                (type going) 
                    (pose ?successfull-pose)))))


;;;; Grasp ;;;;;
;;@author Jan Schimpf
;; gets object id and the grasp-pose takes care of some of the failure-handling for grasp
;; if grasp fails, toya will get into a position to trigger perception again to update
;; the position of objects and then retries with the new information
;; needs to be refactored so it can take over the failurehandling for grasping
;; from the table for both plans needs to be updated with the failure handling for
;; grasping from the table that is currently in the grocery plan
(defun grasp-hsr (object-id grasp-pose)    
    (cpl:with-retry-counters ((grasping-retry 3))
    (cpl:with-failure-handling
        (((or common-fail:low-level-failure 
              cl::simple-error
              cl::simple-type-error)
        (e)
        (comf::move-to-table t)
        (llif::call-take-pose-action 2) 
        (setf *perception-objects* (llif::call-robosherlock-object-pipeline (vector "robocup_table") t))
            (llif::insert-knowledge-objects *perception-objects*)
            (llif::call-take-pose-action 1)
        (cpl:do-retry grasping-retry
            (roslisp:ros-warn (grasp-fail)
                                  "~%Failed to grasp the object~%")
            (cpl:retry))
        (roslisp:ros-warn 
            (going-demo movement-fail)
            "~%No more retries~%")))
    (comf::grasp-object object-id grasp-pose))))



;;;; Place ;;;;
;;@author Jan Schimpf
;; Gets object id and the grasp-pose takes care of some of the failure-handling for place
;; currently retries with different position in case of a failure.
;; Still untested and not in use in any of the plans.
(defun place-hsr (object-id grasp-pose)
    ;;create the the list with contatains a number of place we can place if
    ;;the first one doesn't work
    (let ((?place-position (comf:create-place-list object-id grasp-pose)))     
    (cpl:with-retry-counters ((place-retry 4))
        (cpl:with-failure-handling
            (((or common-fail:low-level-failure 
                  cl::simple-error
                  cl::simple-type-error)
        (e)
        (setf ?place-position (cdr ?place-position))
        ;;starts iterating over the list if after a failed try        
        (cpl:do-retry place-retry
            (roslisp:ros-warn (place-fail)
                                  "~%Failed to grasp the object~%")
            (cpl:retry))
        ;;First element in the list is used to make a designator from it
        (let ((?actual-place-position (car ?place-position)))
        (comf::place-object-list ?actual-place-position))))))))
   

;;@author Philipp Klein
(defun move-to-poi ()
  "moves the robot to the closest poi point"
  (roslisp:ros-info (move-poi) "Move to POI started")
  (setf *listOfPoi* 
        (llif::sorted-poi-by-distance
         (roslisp::with-fields (translation)
             (cl-tf::lookup-transform
              cram-tf::*transformer*
              "map"
              "base_footprint")
           translation)))
  (print "before poi")
  (if (not *listOfPoi*) (return-from move-to-poi Nil))
  (pose-with-distance-to-points *poiDistance* *listOfPoi* 10 t)
  (print "after poi"))

;;@author Philipp Klein
(defun move-to-poi-and-scan ()
  "moves the robot to a poi and perceive it"
  (move-to-poi)
  (llif::call-take-pose-action 1)
  (roslisp::with-fields
      (translation rotation)
      (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint")
    (llif::call-nav-action-ps 
     (cl-tf::make-pose-stamped
      "map"
      0
      translation 
      (cl-tf::q* rotation 
                 (cl-tf::euler->quaternion :ax 0 :ay 0 :az 1.57))))))

;;@author Tom-Eric Lehmkuhl
(defun move-to-table (turn)
  "moves the robot to the table. if turn is true,
   then the robot will move sideways to the table"
    (roslisp:ros-info (move-poi) "Move to table started")
    ;;(defparameter *goalPose* nil)  
    (defparameter *postion* nil)                                            
    (let* ((*tablePose* (llif::prolog-table-pose)) ;;TODO fix!
      (?goal-pose (cl-tf::make-pose-stamped "map" 0
                (cl-tf::make-3d-vector
                    (- (first *tablePose*) 0.7) ;;0.7 was previously 0.95
                    (+ (second *tablePose* ) 0.1)
                    0)
                (if turn
                    (cl-tf::make-quaternion 0 0 0.39 0.91)
                    (cl-tf::make-quaternion 0 0 -0.3 0.93)))))
        ;;(?goal-pose (try-movement-stampedList (list ?goal-pose)))
        (exe::perform (desig:a motion
                    (type going) 
                    (pose ?goal-pose)))))


;;@author Tom-Eric Lehmkuhl
(defun move-to-shelf (turn)
   "moves the robot to the shelf. if turn is true,
   then the robot will move sideways to the shelf"
    (roslisp:ros-info (move-poi) "Move to shelf started")  
    (defparameter *postion* nil)                                            
    ;; add shelf-depth to goal to insert distance (+y)
      (let* ((*shelfPose* (first (first (llif::prolog-shelf-pose))))
             (?goal-pose (cl-tf::make-pose-stamped "map" 0
        (cl-tf::make-3d-vector
            (- (first *shelfPose*) 0.03) ;;was previously 0.225
            (- (second *shelfPose*) 0.9) 0)
        (if turn
            (cl-tf::make-quaternion 0 0 -1 0)
            (cl-tf::make-quaternion 0 0 0.7 0.7)))))
        ;;(?goal-pose (try-movement-stampedList (list ?goal-pose)))
        (exe::perform (desig:a motion
                    (type going) 
                         (pose ?goal-pose)))))

;;@author Tom-Eric Lehmkuhl
(defun move-to-bucket ()
   "moves the robot to the bucket."
    (roslisp:ros-info (move-poi) "Move to bucket started")  
    (defparameter *postion* nil)                                            
    ;; add shelf-depth to goal to insert distance (+y)
    (let* ((*bucketPose* (first (first (llif::prolog-target-pose))))
           (?goal-pose (cl-tf::make-pose-stamped "map" 0
        (cl-tf::make-3d-vector
            (+ (first *bucketPose*) 0.7) ;;was previously 0.225
            (- (second *bucketPose*) 0.05) 0)
            (cl-tf::make-quaternion 0 0 -1 0))))
        ;;(?goal-pose (try-movement-stampedList (list ?goal-pose)))
        (exe::perform (desig:a motion
                    (type going) 
                              (pose ?goal-pose)))))
 
;;@author Jan Schimpf
;;asumption is that the robot is already standing in position
(defun open-door ()
 (cpl:with-retry-counters ((grasping-retry 2))
    (cpl:with-failure-handling
        (((or common-fail:low-level-failure 
              cl::simple-error
              cl::simple-type-error)
        (e)
        (roslisp:ros-info (open-door) "Retry if opening the door failed")
        ;;insert here failure handling new position / retry / perception retry
        (cpl:do-retry grasping-retry
            (roslisp:ros-warn (grasp-fail)
                                  "~%Failed to grasp the object~%")
            (cpl:retry))
        (roslisp:ros-warn 
            (going-demo movement-fail)
            "~%No more retries~%")))
    (roslisp:ros-info (open-door) "Open the door")
      ;; go into percieve position
      ;; call perception client
      ;; go back into normal position
      ;; insert into knowledge
      ;; query knowledge for ID
      ;; call manipulation with ID
      )))



