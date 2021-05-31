(in-package :clean)

(defun execute-cleanup ()
  ;;(init-interfaces)
  (comf::with-hsr-process-modules
    (comf::announce-plan-start "clean up")
    ;;(move-to-start-position)
    ;;TODO: move to start position -> move to first room
    (setf distances-from-current-position
          (llif::sort-surfaces-by-distance
           (llif::prolog-room-surfaces
            (llif::prolog-current-room))))
    (loop for surface-info in distances-from-current-position
          do
             (when (eq (search "Shelf" (car surface-info)) nil)
               (comf::announce-movement-to-surface "future" (car surface-info))
               (comf::move-to-surface (car surface-info) t)
               (comf::perceive-surface (car surface-info))
               (handle-found-objects))
             (setf distances-from-current-position
                   (llif::sort-surfaces-by-distance
                    (llif::prolog-room-surfaces
                     (llif::prolog-current-room)))))
      (poi-search-new)))

;;@author Torge Olliges
(defun move-to-start-position()
    )

;;@author Torge Olliges
(defun handle-found-objects ()
  (let ((next-object (llif::prolog-next-object)))
    (when (eq next-object 1) (return-from handle-found-objects nil))
    (let ((source-surface (llif::prolog-object-source next-object))
          (target-surface (llif::prolog-object-goal next-object)))
      (print source-surface)
      
      (progn 
        (comf::announce-movement-to-surface "future" source-surface)
        (comf::move-to-surface source-surface nil))

      ;;(comf::reachability-check-grasp next-object 1)
      ;;TODO is the next-object still valid?
      (progn
        (comf::announce-grasp-action "future" next-object)
        (comf::grasp-handling next-object))

      (progn
        (comf::announce-movement-to-surface "future" target-surface)
        (comf::move-to-surface target-surface nil))

      (progn
        (comf::announce-place-action "future" next-object)
        (comf::place-handling next-object)))))

(defun poi-search-new ()
  (loop do
    (block continue
    (llif::call-text-to-speech-action "I have found a point of interest to search.") ;;replace with NLG command

  (if (not (comf::move-to-poi))
      (progn 
              (exe:perform (desig:a motion
                         (type going)
                         (pose (llif::find-biggest-notsearched-space T))))
             (return-from continue)))

    (comf::announce-perceive-action "future")
    (setf *perception-objects* (llif::call-robosherlock-object-pipeline (vector "robocup_default") t))
    ;;(comf:reachability-check *perception-objects*)
    (llif::insert-knowledge-objects *perception-objects*)
    ;;(comf:reachability-check (llif::prolog-next-graspable-objects))
    (clean::spawn-btr-objects *perception-objects*)
    ;;percieve -> filter -> insert into knowledge
    (llif::call-take-pose-action 1)
    (setf *next-object* (llif::prolog-next-object))
    (when (eq *next-object* 1) (return-from continue))

    (setf *object-goal-pose* (llif::prolog-object-pose *next-object*))

    (comf::announce-grasp-action "future" *next-object*)
    (llif::call-take-pose-action 1)

    ;; turn to face the object
    ;;(roslisp::with-fields (translation rotation)
    ;;    (cl-tf::lookup-transform cram-tf::*transformer* "map" "base_footprint")
    ;;    (llif::call-nav-action-ps 
    ;;        (cl-tf::make-pose-stamped "map" 0 translation
    ;;            (cl-tf::q* rotation
    ;;            (cl-tf::euler->quaternion :ax 0 :ay 0 :az -1.57)))))
    
    ;; grasp the object from the floor
    (hsr-failure-handling-grasp)
    (comf::announce-movement  "future")
    ;;move to bucket
    (comf::move-to-bucket)

    ;;place object in bucket
    (comf::announce-place-action "present" *next-object*)

    (comf::place-object *next-object* *graspmode*)
    (comf::announce-place-action "past" *next-object*)
    ;;back to base position
    (llif::call-take-pose-action 1))
  ))