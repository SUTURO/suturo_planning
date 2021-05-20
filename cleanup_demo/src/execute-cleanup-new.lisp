(in-package :clean)

(defun execute-cleanup()
    (init-interfaces)
    (comf::with-hsr-process-modules
        (comf::announce-plan-start "clean up")
        (move-to-start-position)
        ;;TODO: move to start position -> move to first room
        ;;(loop for room-name in (llif::prolog-rooms) do (what follows...))
        ;; TODO: loop over tables in room...
        (loop for table-id in (llif::sort-surfaces-by-distance (llif::prolog-tables))
            do (
                (comf::announce-movement-to-surface "future" table-id)
                (comf::move-to-surface table-id t)
                (perceive-table table-id)
                (handle-found-objects)))
        (poi-search);;POI stuff missing
        ))

;;@author Torge Olliges
(defun move-to-start-position()
    )

;;@author Torge Olliges
(defun perceive-table (table-id)
    (comf::announce-perceive-action-surface "present" table-id)
    (llif::call-take-pose-action 2)
    (let ((perceived-objects (llif::call-robosherlock-object-pipeline (vector table-id) t))
          (confident-objects (comf::get-confident-objects perceived-objects)))
        (llif::insert-knowledge-objects confident-objects)
        (clean::spawn-btr-objects confident-objects))
    (llif::call-take-pose-action 1))

;;@author Torge Olliges
(defun handle-found-objects ()
    (let ((next-object (llif::prolog-next-object))
          (source-surface (llif::prolog-object-source next-object))
          (target-surface (llif::prolog-object-goal next-object)))
        (when (eq next-object 1) (return-from handle-found-objects nil))
        (comf::reachability-check-grasp next-object 1)
        ;;TODO is the next-object still valid?

        (comf::announce-movement-to-surface "future" source-surface)
        (comf::move-to-surface source-surface nil)

        (comf::announce-grasp-action "future" next-object)
        (grasp-handling next-object)

        (comf::announce-movement-to-surface "future" target-surface)
        (comf::move-to-surface target-surface nil)
        
        (comf::announce-place-action "future" next-object)
        (place-handling next-object)))

;;@author Torge Olliges
(defun grasp-handling (next-object)
    (let ((grasp-action-result (comf::grasp-object next-object 1)))
        (roslisp::with-fields (error_code)
                grasp-action-result
                (if (eq error_code 0)
                    (comf::announce-grasp-action "past" next-object)
                    (cpl:with-retry-counters ((grasp-retries 1))
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
                                (roslisp:ros-warn (going-demo movement-fail) "~%No more retries~%")))
                          (setf *grasp-mode* 1)  ;;sets the graspmode should be replaces with the function from knowledge when that is finished
                          (comf::grasp-object next-object *grasp-mode*)
                          (comf::announce-grasp-action "past" next-object)))))))


;;@author Torge Olliges
(defun place-handling (next-object)
    (let ((place-action-result (comf::place-object next-object 1)))
        (roslisp::with-fields (error_code)
            place-action-result
                (if (eq place-action-result 0)
                    (comf::announce-place-action "past" next-object)
                    ((cpl:with-retry-counters ((place-retries 1))
                        (cpl:with-failure-handling
                            (((or 
                                common-fail:low-level-failure 
                                cl::simple-error
                                cl::simple-type-error)
                                (e)
                                (comf::announce-place-action "failed"  nex-object)
                                (cpl:do-retry place-retries
                                    (roslisp:ros-warn (place-handling) "~%Failed to grasp the object~%")
                                    (cpl:retry))
                                (roslisp:ros-warn (going-demo movement-fail) "~%No more retries~%")))
                            (setf *grasp-mode* 1)  ;;sets the graspmode should be replaces with the function from knowledge when that is finished
                            (comf::place-object next-object *graspmode*)
                            (if (eq (comf::reachability-check-place next-object *grasp-mode*) 1)
                                (throw common-fail:low-level-failure)
                                (comf::place-object next-object *grasp-mode*))
                            (comf::announce-place-action "past" next-object))))))))

(defun poi-search-new ()
  (loop do
    (block continue
    (llif::call-text-to-speech-action "I have found a point of interest to search.") ;;replace with NLG command

  (if (not (comf::move-to-poi))
      (progn (
              (exe:perform (desig:a motion
                         (type going)
                         (pose (llif::find-biggest-notsearched-space T))))
             (return-from continue))))

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
