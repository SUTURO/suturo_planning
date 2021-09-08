(in-package :clean)

(defun execute-cleanup ()
  (init-interfaces)
  (comf::with-hsr-process-modules
    (llif::call-take-pose-action 1)
    ;;(comf::announce-plan-start "clean up")
    ;;(move-to-start-position)
    ;;TODO: move to start position -> move to first room
    ;;(setf surfaces-with-distances-from-current-position
    ;;      (llif::sort-surfaces-by-distance
    ;;       (llif::prolog-room-surfaces
    ;;        (llif::prolog-current-room))))
    ;; (robocup-floor-check)
    ;; (loop for surface in (llif::prolog-cleanup-surfaces)
    ;;      do
    ;;         (llif::prolog-set-surface-not-visited surface))
    
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
                        ;;(comf::announce-movement-to-surface "future" (car surface-info))
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

(defun robocup-floor-check ()
  (let ((nav-pose (cl-tf2::make-pose-stamped
                   "map"
                   0
                   (cl-tf2::make-3d-vector 0.3567 0.744 0)
                   (cl-tf2::q* (cl-tf2::make-quaternion 0 0 0 1)
                      (cl-tf2::euler->quaternion
                       :ax 0
                       :ay 0
                       :az (/ pi 1.5)))))
        (nav-pose-grasp (cl-tf2::make-pose-stamped
                         "map"
                         0
                         (cl-tf2::make-3d-vector 0.3567 0.744 0)
                         (cl-tf2::q* (cl-tf2::make-quaternion 0 0 0 1)
                                     (cl-tf2::euler->quaternion
                                      :ax 0
                                      :ay 0
                                      :az 0.75))))
        (look-at-pose (llif::prolog-surface-pose (first (llif::prolog-cleanup-surfaces)))))
    (comf::move-hsr nav-pose)
    (llif::call-take-gaze-pose-action
     :px (first (first look-at-pose))
     :py (second (first look-at-pose))
     :pz 0)
    (let ((detected-objects (llif::call-robosherlock-object-pipeline (vector "robocup_default") t)))
      (llif::insert-knowledge-objects (comf::get-confident-objects detected-objects))
      (llif::call-take-pose-action 1)
      (comf::move-hsr nav-pose-grasp)
      (handle-found-objects))))
    

;;@author Torge Olliges
(defun handle-found-objects ()
  (loop
    do
       (let ((next-object (llif::prolog-next-object)))
         (when (eq next-object 1) (return-from handle-found-objects nil))
         (let ((source-surface (llif::prolog-object-source next-object))
               (target-surface (llif::prolog-object-goal next-object)))
           (when (or (eq source-surface nil) (search "Floor" source-surface))
             (comf::move-hsr
              (cl-tf2::make-pose-stamped
               "map"
               0
               (cl-tf2::make-3d-vector 0.3567 0.744 0)
               (cl-tf2::q* (cl-tf2::make-quaternion 0 0 0 1)
                           (cl-tf2::euler->quaternion
                            :ax 0
                            :ay 0
                            :az 0.75))))
             (deliver-object next-object target-surface)
             (return-from handle-found-objects))
           (progn 
           ;;(comf::announce-movement-to-surface "future" source-surface)
             (roslisp::ros-info (handle-found-objects)
                                "Moving to source surface of ~a namely ~a" next-object source-surface)
             (comf::move-to-surface source-surface nil))
           (deliver-object next-object target-surface source-surface)))))

(defun deliver-object (next-object target-surface &optional source-surface)
  (progn
      ;;(comf::announce-grasp-action "future" next-object)
      (roslisp::ros-info (handle-found-objects) "Trying to grasp ~a" next-object)
      (comf::grasp-handling next-object))
  (let ((objects-in-gripper (llif::prolog-object-in-gripper)))
    (roslisp::ros-info (deliver-object) "Objects in gripper ~a" objects-in-gripper)
    (when (eq (length objects-in-gripper) 0)
      (llif::prolog-set-object-handled next-object)
      (return-from deliver-object)))
  (progn
    ;;(comf::announce-movement-to-surface "future" target-surface)
    (comf::ros-info (handle-found-objects)
                    "Moving to target surface of ~a namely ~a" next-object target-surface)
    (comf::move-to-surface target-surface nil))
  
  (progn
    ;;(comf::announce-place-action "future" next-object)
    (roslisp::ros-info (handle-found-objects)
                       "Trying to place ~a from ~a on ~a" next-object source-surface target-surface)
    (llif::call-take-pose-action 6)
    (comf::place-object next-object 1))

  (llif::call-take-pose-action 1)
  (llif::call-take-pose-action 1))

;;@author Philipp Klein
(defun poi-search ()
  "approaching the next poi and bring the found object to the basket. When no poi is found, start the poi-search and loop the behavior."
  (loop do
    (block continue
      ;;(llif::call-text-to-speech-action "I have found a point of interest to search.") ;;replace with NLG command
      (let ((poi-pos (comf::move-to-poi)))
      (if (not poi-pos)
          (progn 
            (comf::move-hsr (llif::find-biggest-unsearched-space T))
            (return-from continue)))

      ;;(comf::announce-perceive-action "future")

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
              (progn
                (llif::poi-remover (cl-tf::origin poi-pos) 0.1)
                (return-from continue)))))
      
      (let ((next-object (llif::prolog-next-object)))
        (when (eq next-object 1)
          (progn(llif::poi-remover (cl-tf::origin poi-pos) 0.1) (return-from continue)))
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

          (llif::call-take-pose-action 1)))))))
