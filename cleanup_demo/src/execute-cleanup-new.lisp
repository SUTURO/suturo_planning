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
        ;;POI stuff missing
        ))

;;@author Torge Olliges
(defun move-to-start-position()
    )

;;@author Torge Olliges
(defun perceive-table (table-id)
    (comf::announce-perceive-action-surface "present" table-id)
    (llif::call-take-pose-action 3)
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
