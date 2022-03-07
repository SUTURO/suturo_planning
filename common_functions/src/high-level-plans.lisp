(in-package :comf)
(defparameter *poi-distance-threshold* 0.75)
(defparameter *grasp-mode* nil)
(defparameter *object-list* nil)

;;;; Navigation ;;;;

;;@author Torge Olliges, Phillip Klein
;;Tries a list of stamped poses in the bulletworld simulation
;;returns a possible pose.
(defun try-movement-stamped-list (listStamped)
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
        (try-movement-stamped-list 
				    (list 
             nav-goal-pose-stamped))))
      (exe:perform 
            (desig:a motion
                (type going) 
                    (pose ?successfull-pose)))))


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
    ;;(roslisp:ros-info (move-to-poi) "Poi with positions called ~a" poi-list)
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
           (roslisp:ros-warn  (going-demo movement-fail) "~%No more retries~%")))
      (let ((knowledge-pose-manipulation (llif::prolog-manipulating-pose-of-door door-id))) ;;get position to move the robot to
        (comf::get-motion-des-going-for-doors knowledge-pose-manipulation t) ;;execution of the move   
        (let ((knowledge-doorhandle-id 
                (concatenate 'string
                             "iai_kitchen/"
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
;; Function that is work in progress and is intended to navigate the robot between rooms
;; and open door to reach the goal if needed.
(defun open-door-on-path (start-room target-room)
  (loop for door-id in (llif::prolog-shortest-path-between-rooms start-room target-room)
        do
           (comf::open-room-door door-id)))


;;@author Torge Olliges
(defun perceive-surface (surface-id)
  ;;(comf::announce-perceive-action-surface "present" surface-id)
  (let ((surface-pose (first (llif::prolog-surface-pose surface-id))))
    ;;(llif::call-take-gaze-pose-action
    ;;                             :px (first surface-pose)
    ;;                             :py (second surface-pose)
    ;;                             :pz (third surface-pose))
    (llif::call-take-pose-action 2))
  
  (let* ((surface-region (llif::prolog-surface-region surface-id))
         (perceived-objects
           (llif::call-robosherlock-object-pipeline
            (vector surface-region) t))
         (confident-objects (comf::get-confident-objects perceived-objects)))
    ;;(print perceived-objects)
    (print confident-objects)
    (setf *object-list* confident-objects)
    (roslisp::with-fields (detectiondata)
          confident-objects
        (progn
          (roslisp::ros-info (poi-search) "Number of objects detected: ~a" (length detectiondata))
          (when (> (length detectiondata) 0)
              (llif::insert-knowledge-objects confident-objects)))))
  (llif::prolog-set-surface-visited surface-id)
  ;;(clean::spawn-btr-objects confident-objects))
  (llif::call-take-pose-action 1))

;;@author Torge Olliges
(defun grasp-handling (next-object)
  (dynamic-grasp next-object)
  (roslisp::ros-info (grasp-handling) "Starting grasp handling for object ~a" next-object)
  ;;(setf clean::*last-inner-timestamp* (get-universal-time))
  (let ((grasp-action-result (comf::grasp-object next-object *grasp-mode*)))
    ;;(setf clean::*total-manipulation-time*
    ;;      (+
    ;;       (- (get-universal-time) clean::*last-inner-timestamp*)
    ;;       clean::*total-manipulation-time*))
    (print grasp-action-result)
    (roslisp::with-fields (error_code)
        grasp-action-result
      (if (eq error_code 1)
          ;;(comf::announce-grasp-action "past" next-object)
          (cpl:with-retry-counters ((grasp-retries 3))
            (cpl:with-failure-handling
                (((or 
                   common-fail:low-level-failure 
                   cl::simple-error
                   cl::simple-type-error)
                     (e)
                   ;;(comf::announce-grasp-action "failed"  next-object)
                   (cpl:do-retry grasp-retries
                     (roslisp:ros-warn (grasp-handling) "~%Failed to grasp the object~%")
                     (cpl:retry))
                   (roslisp:ros-warn (grasp-handling) "~%No more retries~%")))
              ;;(dynamic-grasp next-object)  ;;sets the graspmode should be replaced with the function from knowledge when that is finished
              ;;(setf clean::*last-inner-timestamp* (get-universal-time))
              (comf::grasp-object next-object *grasp-mode*)
              ;;(setf clean::*total-manipulation-time*
              ;;      (+
              ;;       (- (get-universal-time) clean::*last-inner-timestamp*)
              ;;clean::*total-manipulation-time*))
              )))))
  (llif::call-take-pose-action 1)
  ;;(comf::announce-grasp-action "past" next-object))
)


;;Author Jan Schimpf
;; Quick solution making use of the second Graspmode
;; Graspmode 1 in case the Object is larger than is higher than 6 cm
;; and Graspmode 2 in case the Object 6cm high or less 
(defun dynamic-grasp (object-id)
  (third (llif:prolog-object-dimensions object-id))
  (if (< 0.06 (third (llif:prolog-object-dimensions object-id)))
      (setf *grasp-mode* 1)
      (setf *grasp-mode* 2)))
