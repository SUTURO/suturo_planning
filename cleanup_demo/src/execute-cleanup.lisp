(in-package :clean)

(defun execute-cleanup ()
  ;;(init-interfaces)
  (comf::with-hsr-process-modules
    (llif::call-take-pose-action 1)
    (comf::announce-plan-start "clean up")
    ;;(move-to-start-position)
    ;;TODO: move to start position -> move to first room
    ;;(setf surfaces-with-distances-from-current-position
    ;;      (llif::sort-surfaces-by-distance
    ;;       (llif::prolog-room-surfaces
    ;;        (llif::prolog-current-room))))
    (loop for surface in (llif::prolog-cleanup-surfaces)
          do
             (llif::prolog-set-surface-not-visited surface))
    
    (loop for room in (llif::prolog-all-rooms)
          do
             ;;(setf surfaces-with-distances-from-current-position
             ;;      (llif::sort-surfaces-by-distance
             ;;       (llif::prolog-room-surfaces
             ;;        room)))
             (loop for surface-info in (llif::sort-surfaces-by-distance
                                        (llif::prolog-surfaces-not-visited-in-room
                                         room))
                   do
                      ;;(when (and
                      ;;       (eq (search "Shelf" (car surface-info)) nil)
                      ;;       (eq (search "bucket"
                      ;;                   (llif::prolog-surface-region (car surface-info))) nil)
                      ;;       (eq (search "chair"
                      ;;                   (llif::prolog-surface-region (car surface-info))) nil))
                        (comf::announce-movement-to-surface "future" (car surface-info))
                        (comf::move-to-surface (car surface-info) t)
                        (comf::perceive-surface (car surface-info))
                        (handle-found-objects);;)
                      ;;(setf surfaces-with-distances-from-current-position
                      ;;      (llif::sort-surfaces-by-distance
                      ;;       (llif::prolog-room-surfaces
                      ;;        (llif::prolog-current-room)))))
                   ))
    ;;(poi-search)
    ))

;;@author Torge Olliges
(defun move-to-start-position()
    )

;;@author Torge Olliges
(defun handle-found-objects ()
  (loop
    do
       (let ((next-object (llif::prolog-next-object)))
         (when (eq next-object 1) (return-from handle-found-objects nil))
         (let ((source-surface (llif::prolog-object-source next-object))
               (target-surface (llif::prolog-object-goal next-object)))
           (print source-surface)
           
           (progn 
             (comf::announce-movement-to-surface "future" source-surface)
             (comf::move-to-surface source-surface nil))
           
           (comf::reachability-check-grasp next-object 1)
           ;;TODO is the next-object still valid?
           (progn
             (comf::announce-grasp-action "future" next-object)
             (comf::grasp-handling next-object))

           (progn
             (comf::announce-movement-to-surface "future" target-surface)
             (comf::move-to-surface target-surface nil))
           
           (progn
             (comf::announce-place-action "future" next-object)
             (comf::place-object next-object 1))
           (llif::call-take-pose-action 1)))))

;;@author Philipp Klein
(defun poi-search ()
  "approaching the next poi and bring the found object to the basket. When no poi is found, start the poi-search and loop the behavior."
  (loop do
    (block continue
      (llif::call-text-to-speech-action "I have found a point of interest to search.") ;;replace with NLG command

      (if (not (comf::move-to-poi))
          (progn 
            (comf::move-hsr (list (llif::find-biggest-unsearched-space T)))
            (return-from continue)))

      (comf::announce-perceive-action "future")

      (setf confident-objects
            (comf::get-confident-objects
             (llif::call-robosherlock-object-pipeline (vector "robocup_default") t)
             0.8))


      ;;percieve -> filter -> insert into knowledge
      (llif::call-take-pose-action 1)

      ;; turn to face the object
      (roslisp::with-fields (translation rotation)
        (cl-tf::lookup-transform cram-tf::*transformer* "map" "base_footprint")
        (llif::call-nav-action-ps 
            (cl-tf::make-pose-stamped "map" 0 translation
                (cl-tf::q* rotation
                           (cl-tf::euler->quaternion :ax 0 :ay 0 :az (- (/ pi 2)))))))
      
      (roslisp::with-fields (detectiondata)
          confident-objects
        (progn
          (roslisp::ros-info (poi-search) "Number of objects detected: ~a" (length detectiondata))
          (if (> (length detectiondata) 0)
              (llif::insert-knowledge-objects confident-objects)
              (return-from continue))))
      
      (let ((next-object (llif::prolog-next-object)))
        (when (eq next-object 1) (return-from continue))
        (let ((object-goal (llif::prolog-object-goal next-object)))
          (comf::announce-grasp-action "future" next-object)
          (llif::call-take-pose-action 1)
          
          (comf::grasp-handling next-object)
          (comf::announce-movement  "future")
          (comf::move-to-surface object-goal nil)
          
          ;;place object at goal surface
          (comf::announce-place-action "present" next-object)
          
          (comf::place-object next-object 1)
          (comf::announce-place-action "past" next-object)

          (llif::call-take-pose-action 1))))))
