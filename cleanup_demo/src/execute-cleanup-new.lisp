(in-package :clean)

(defun execute-cleanup-new()
    ;;(init-interfaces)
    (comf::with-hsr-process-modules
        (comf::announce-plan-start "clean up")
        ;;(move-to-start-position)
        ;;TODO: move to start position -> move to first room
        ;;(loop for room-name in (llif::prolog-rooms) do (what follows...))
        (loop for table-id in (llif::sort-surfaces-by-distance (llif::prolog-room-surfaces (llif::prolog-current-room)))
              do
                (when (eq (search "Shelf" (car table-id)) nil)
                (comf::announce-movement-to-surface "future" (car table-id))
                (comf::move-to-surface (car table-id) t)
                (perceive-table (car table-id))
                (handle-found-objects))
        ;;POI stuff missing
        )))

;;@author Torge Olliges
(defun move-to-start-position()
    )

;;@author Torge Olliges
(defun perceive-surface (surface-id)
  (comf::announce-perceive-action-surface "present" surface-id)
  (let ((surface-pose (first (llif::prolog-surface-pose surface-id))))
    (llif::call-take-pose-action 5 0.0 0.0 0.0 0.0 0.0 0.0 0.0
                                 (first surface-pose)
                                 (second surface-pose)
                                 (third surface-pose)))
  (let* ((perceived-objects
           (llif::call-robosherlock-object-pipeline
            (vector (llif::prolog-surface-region surface-id)) t))
         (confident-objects (comf::get-confident-objects perceived-objects)))
    (llif::insert-knowledge-objects confident-objects))
  ;;(clean::spawn-btr-objects confident-objects))
  (llif::call-take-pose-action 1))

;;@author Torge Olliges
(defun handle-found-objects ()
  (let ((next-object (llif::prolog-next-object)))
        (when (eq next-object 1) (return-from handle-found-objects nil))
      (let ((source-surface (llif::prolog-object-source next-object))
            (target-surface (llif::prolog-object-goal next-object)))
        (print source-surface)
        ;;(comf::reachability-check-grasp next-object 1)
        ;;TODO is the next-object still valid?
        
        (comf::announce-movement-to-surface "future" source-surface)
        (comf::move-to-surface source-surface nil)
        
        (comf::announce-grasp-action "future" next-object)
        (grasp-handling next-object)
        
        (comf::announce-movement-to-surface "future" target-surface)
        (comf::move-to-surface target-surface nil)
        
        (comf::announce-place-action "future" next-object)
        (place-handling next-object))))

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
                                (roslisp:ros-warn (going-demo movement-fail) "~%No more retries~%")))
                            (setf *grasp-mode* 1)  ;;sets the graspmode should be replaces with the function from knowledge when that is finished
                            (comf::place-object next-object *graspmode*)
                            (if (eq (comf::reachability-check-place next-object *grasp-mode*) 1)
                                (throw common-fail:low-level-failure "Not Reachable")
                                (comf::place-object next-object *grasp-mode*))
                            (comf::announce-place-action "past" next-object)))))))
