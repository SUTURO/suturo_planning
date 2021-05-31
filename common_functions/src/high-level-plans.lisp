(in-package :comf)
(defparameter *poi-distance-threshold* 0.75)

;;;; Navigation ;;;;

;;@author Torge Olliges, Phillip Klein
;;Tries a list of stamped poses in the bulletworld simulation
;;returns a possible pose.
(defun try-movement-stampedList (listStamped)
   (car listStamped))
 ;;  (let (?nav-pose listStamped)
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
  (let* ((surface-pose (llif::prolog-surface-pose surface-id))
        (?goal-pose
          (cl-tf::make-pose-stamped
            "map" 0
            (roslisp::with-fields (origin)
                (get-nav-pose-for-surface surface-id) origin)
            (if turn
                (cl-transforms:q* 
                 (cl-tf::make-quaternion
                  (first (second surface-pose))
                  (second (second surface-pose))
                  (third (second surface-pose))
                  (fourth (second surface-pose)))
                 (cl-transforms:euler->quaternion :ax 0 :ay 0 :az (/ pi 2)))
                (cl-tf::make-quaternion
                 (first (second surface-pose))
                 (second (second surface-pose))
                 (third (second surface-pose))
                 (fourth (second surface-pose)))))))
    (exe::perform (desig:a motion
                           (type going)
                           (pose ?goal-pose)))))

;;@author Philipp Klein
(defun move-to-poi ()
  "moves the robot to the closest poi point"
  (roslisp:ros-info (move-poi) "Move to point of interest started")
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
                 (cl-tf::euler->quaternion :ax 0 :ay 0 :az (/ pi 2)))))))

;;@author Jan Schimpf
;;Gets the door id as input and move into position to open it, currently only for one door as the position query isn't done yet
(defun open-roomdoor (door-id)
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

      ;;this is here for testing purposes only / until it is replaced with the proper querry
      (comf::get-nav-pose-for-doors (llif::prolog-manipulating-pose-of-door door-id) t)
      ;; go into percieve position (as manipulation works with its own angle this isn't needed yet)

     
      ;; insert into knowledge ...
      
      ;; query knowledge for ID (manipulation doesn't use this currently ...
      (let ((knowledge-doorhandle-id (concatenate 'string "iai_kitchen/" (llif::prolog-knowrob-name-to-urdf-link
                                                                          (car (cdr (llif::prolog-perceiving-pose-of-door door-id))))))
            (knowledge-pose   (car (llif::prolog-perceiving-pose-of-door door-id)))
            (knowledge-open-door-angle (llif::prolog-get-angle-to-open-door door-id)))
        (llif::call-open-action knowledge-doorhandle-id
                                knowledge-doorhandle-id
                                1.35)
        (llif::prolog-update-door-state door-id "1.35")
        (comf::get-nav-pose-for-doors knowledge-pose t)))))


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
    (llif::insert-knowledge-objects confident-objects))
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
              (setf *grasp-mode* 1)  ;;sets the graspmode should be replaces with the function from knowledge when that is finished
              (comf::grasp-object next-object *grasp-mode*)
              (comf::announce-grasp-action "past" next-object)))))))

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
                     (roslisp:ros-warn (place-handling) "~%Failed to grasp the object~%")
                     (cpl:retry))
                   (roslisp:ros-warn (place-action) "~%No more retries~%")))
              (setf *grasp-mode* 1)  ;;sets the graspmode should be replaces with the function from knowledge when with-hash-table-iterator is finished
              (comf::place-object next-object *grasp-mode*)
              (if (eq (comf::reachability-check-place next-object *grasp-mode*) 1)
                  (throw common-fail:low-level-failure "Not Reachable")
                  (comf::place-object next-object *grasp-mode*))
              (comf::announce-place-action "past" next-object)))))))





