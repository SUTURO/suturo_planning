(in-package :comf)
(defparameter *poi-distance-threshold* 0.75)
(defparameter *graspmode* nil)

;;;; Navigation ;;;;

;;@author Torge Olliges, Phillip Klein
;;Tries a list of stamped poses in the bulletworld simulation
;;returns a possible pose.
(defun try-movement-stampedList (listStamped)
 ;;  (car listStamped))
   (let ((?nav-pose listStamped))
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
                 (going-demo movement-fail) "~%Failed to move to given position~%")
                (cpl:retry))
              (roslisp:ros-warn  (going-demo movement-fail) "~%No more retries~%")))
         (let ((?actual-nav-pose (car ?nav-pose))) 
           (exe:perform
            (desig:a motion
                      (type going)
                      (pose ?actual-nav-pose)))
           (setf listStamped (cdr ?nav-pose))
           ?actual-nav-pose)))))


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
            (roslisp:ros-warn (place-fail) "~%Failed to grasp the object~%")
            (cpl:retry))
        ;;First element in the list is used to make a designator from it
        (let ((?actual-place-position (car ?place-position)))
        (comf::place-object-list ?actual-place-position))))))))

;;@author Torge Olliges
(defun move-to-room (room-id)
  (roslisp::ros-info (move-to-room) "Starting movement to room ~a" room-id)
  (announce-movement "present")
    (let ((shortest-path (llif::prolog-shortest-path-between-rooms
                           (llif::prolog-current-room) room-id)))
        (print shortest-path)
    ))

;;@author Torge Olliges
(defun move-to-surface (surface-id turn)
  (roslisp:ros-info (move-too-room) "Moving to surface ~a" surface-id)
  (let* ((perceive-pose (llif::prolog-pose-to-perceive-surface surface-id))
        (?goal-pose
          (cl-tf::make-pose-stamped
            "map" 0
            (cl-tf::make-3d-vector
             (first (first perceive-pose))
             (second (first perceive-pose))
             (third (first perceive-pose)))
            (if turn
                (cl-transforms:q* 
                 (cl-tf::make-quaternion
                  (first (second perceive-pose))
                  (second (second perceive-pose))
                  (third (second perceive-pose))
                  (fourth (second perceive-pose)))
                 (cl-transforms:euler->quaternion :ax 0 :ay 0 :az (/ pi 2)))
                (cl-tf::make-quaternion
                 (first (second perceive-pose))
                 (second (second perceive-pose))
                 (third (second perceive-pose))
                 (fourth (second perceive-pose)))))))
    (exe::perform (desig:a motion
                           (type going)
                           (pose ?goal-pose)))))

;;@author Philipp Klein
(defun move-to-poi ()
  "moves the robot to the closest poi point"
  (roslisp:ros-info (move-to-poi) "Move to point of interest started")
  (let ((poi-list (llif::sorted-poi-by-distance
                   (roslisp::with-fields (translation)
                       (cl-tf::lookup-transform
                        cram-tf::*transformer*
                        "map"
                        "base_footprint")
                     translation))))
    (when 
        (not poi-list) 
      (return-from move-to-poi Nil))
    (roslisp:ros-info (move-to-poi) "Poi with positions called ~a" poi-list)
    (pose-with-distance-to-points *poi-distance-threshold* poi-list 10 t)))

;;@author Philipp Klein
(defun move-to-poi-and-scan ()
  "moves the robot to a poi and perceives it"
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
                 (cl-tf::euler->quaternion :ax 0 :ay 0 :az (/ pi 2)))))))

;;@author Jan Schimpf
;;Gets the door id as input and move into position to open it, currently only for one door as the position query isn't done yet
(defun open-room-door (door-id)
 (cpl:with-retry-counters ((grasping-retry 2))
    (cpl:with-failure-handling
        (((or common-fail:low-level-failure 
              cl::simple-error
              cl::simple-type-error)
        (e)
        (roslisp:ros-info (open-door) "Retry if opening the door failed")
        ;;insert here failure handling new position / retry / perception retry
        (cpl:do-retry grasping-retry
            (roslisp:ros-warn (grasp-fail) "~%Failed to grasp the object~%")
          (cpl:retry))
          (comf::get-nav-pose-for-doors (llif::prolog-manipulating-pose-of-door door-id) t)
        (roslisp:ros-warn 
            (going-demo movement-fail)
            "~%No more retries~%")))

      (let ((knowledge-pose-manipulation (llif::prolog-manipulating-pose-of-door door-id))) ;;get position to move the robot to
             (comf::get-motion-des-going-for-doors knowledge-pose-manipulation t) ;;execution of the move 
           
        (let ((knowledge-doorhandle-id (concatenate 'string "iai_kitchen/"
                                                    (llif::prolog-knowrob-name-to-urdf-link ;;changes the knowrob id to the urdf link
                                                        (car (cdr (llif::prolog-perceiving-pose-of-door door-id)))))) ;; get the knowrob id for the door handle
            (knowledge-pose-perceiving   (car (llif::prolog-perceiving-pose-of-door door-id))) 
            (knowledge-open-door-angle (llif::prolog-get-angle-to-open-door door-id)))
            (llif::call-open-action knowledge-doorhandle-id
                                    knowledge-doorhandle-id
                                    knowledge-open-door-angle)
            (llif::prolog-update-door-state door-id knowledge-open-door-angle) ;; update the door state
            (comf::get-motion-des-going-for-doors knowledge-pose-manipulation t)))))) ;; get into a position from which the robot can move through the open door
           

;;@author Jan Schimpf
(defun open-door-on-the-path (start-room target-room)
  (loop for door-id in (llif::prolog-shortest-path-between-rooms start-room target-room)
        do
           (comf::open-room-door door-id)
        ))


;;@author Torge Olliges
(defun perceive-surface (surface-id)
  (comf::announce-perceive-action-surface "present" surface-id)
  (let ((surface-pose (first (llif::prolog-surface-pose surface-id))))
    (llif::call-take-gaze-pose-action
                                 :px (first surface-pose)
                                 :py (second surface-pose)
                                 :pz (third surface-pose)))
  (let* ((perceived-objects
           (llif::call-robosherlock-object-pipeline
            (vector (llif::prolog-surface-region surface-id)) t))
         (confident-objects (comf::get-confident-objects perceived-objects)))

    (roslisp::with-fields (detectiondata)
          confident-objects
        (progn
          (roslisp::ros-info (poi-search) "Number of objects detected: ~a" (length detectiondata))
          (when (> (length detectiondata) 0)
              (llif::insert-knowledge-objects confident-objects)))))
  (llif::prolog-set-surfaces-visit-state surface-id)
  ;;(clean::spawn-btr-objects confident-objects))
  (llif::call-take-pose-action 1))

;;@author Torge Olliges
(defun grasp-handling (next-object)
  (roslisp::ros-info (grasp-handling) "Starting grasp handling for object ~a" next-object)
  (let ((grasp-action-result (comf::grasp-object next-object 1)))
    (roslisp::with-fields (error_code)
        grasp-action-result
      (if (eq error_code 0)
          (comf::announce-grasp-action "past" next-object)
          (cpl:with-retry-counters ((grasp-retries 3))
            (cpl:with-failure-handling
                (((or 
                   common-fail:low-level-failure 
                   cl::simple-error
                   cl::simple-type-error)
                     (e)
                   (comf::announce-grasp-action "failed"  next-object)
                   (cpl:do-retry grasp-retries
                     (roslisp:ros-warn (grasp-handling) "~%Failed to grasp the object~%")
                     (cpl:retry))
                   (roslisp:ros-warn (grasp-handling) "~%No more retries~%")))
              (dynamic-grasp next-object)  ;;sets the graspmode should be replaced with the function from knowledge when that is finished
              (comf::grasp-object next-object *grasp-mode*))))))
  (comf::announce-grasp-action "past" next-object)
  (llif::call-take-pose-action 1))

;;@author Torge Olliges
(defun place-handling (next-object)
  (let ((place-action-result (comf::place-object next-object 1)))
    (roslisp::with-fields (error_code)
        place-action-result
      (if (eq error_code 0)
          (comf::announce-place-action "past" next-object)
          (cpl:with-retry-counters ((place-retries 1))
            (cpl:with-failure-handling
                (((or 
                   common-fail:low-level-failure 
                   cl::simple-error
                   cl::simple-type-error)
                     (e)
                   (comf::announce-place-action "failed"  next-object)
                   (cpl:do-retry place-retries
                     (roslisp:ros-warn (place-handling) "~%Failed to place the object~%")
                     (cpl:retry))
                   (roslisp:ros-warn (place-action) "~%No more retries~%")))
              (dynamic-grasp next-object)  ;;sets the graspmode should be replaces with the function from knowledge when with-hash-table-iterator is finished
              (if (eq (comf::reachability-check-place next-object *grasp-mode*) 1)
                  (throw common-fail:low-level-failure "Not Reachable")
                  (comf::place-object next-object *grasp-mode*))
              (llif::call-take-pose-action 1)
              (comf::announce-place-action "past" next-object)))))))





(defun dynamic-grasp (object-id)

  (if (< 0.06 (nth 2  (llif:prolog-object-dimensions object-id)))
      (setf *graspmode* 1)
      (setf *graspmode* 2)))
