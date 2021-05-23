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
                     (llif::prolog-current-room))))
             ;;POI stuff missing
          )))

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





