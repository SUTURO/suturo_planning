(in-package :clean)
(defparameter *start-time* 0)
(defparameter *last-timestamp* 0)
(defparameter *last-inner-timestamp* 0)
(defparameter *total-init-time* 0)
(defparameter *total-movement-time* 0)
(defparameter *total-detection-time* 0)
(defparameter *total-manipulation-time* 0)


(defun execute-cleanup ()
  (reset-timestamps)
  (setf *start-time* (get-universal-time))
  (setf *last-timestamp* (get-universal-time))
  (init-interfaces)
  (roslisp::ros-info (execute-cleanup) "Initialisation took ~a seconds"
                 (- (get-universal-time) *last-timestamp*))
  (setf *total-init-time* (- (get-universal-time) *last-timestamp*))
  
  (comf::with-hsr-process-modules
    (setf *last-timestamp* (get-universal-time))
    (llif::call-take-pose-action 1)
    (setf *total-manipulation-time*
          (+ *total-manipulation-time* (- (get-universal-time) *last-timestamp*)))
    (comf::announce-plan-start "clean up")
    
    
    (loop for room in (llif::prolog-all-rooms)
          do
             (loop for surface-info in (comf::sort-surfaces-by-distance
                                        (llif::prolog-room-surfaces
                                         (llif::prolog-current-room)))
                   do
                      (let* ((surface (car surface-info))
                             (surface-region-name (llif::prolog-surface-region surface)))
                        (when (and
                               (eq (search "Shelf"
                                           surface) nil)
                               (eq (search "bin"
                                           surface-region-name) nil)
                               (eq (search "tray"
                                           surface-region-name) nil)
                               (eq (search "Floor"
                                           surface) nil)
                               (eq (search "chair"
                                           surface-region-name) nil)
                               (eq (search "container"
                                           surface-region-name) nil))
                          (roslisp::ros-info (execute-cleanup)
                                             "Surface set not visited, surface: ~a region name: ~a"
                                             surface
                                             surface-region-name)
                          (llif::prolog-set-surface-not-visited (car surface-info)))))

             (loop for surface-info in (comf::sort-surfaces-by-distance
                                        (llif::prolog-surfaces-not-visited-in-room
                                         room))
                   do
                        (setf *last-timestamp* (get-universal-time))
                        (comf::move-to-surface (car surface-info) t)
                        (setf *total-movement-time*
                              (+
                               (- (get-universal-time) *last-timestamp*)
                               *total-movement-time*))
                        
                        (comf::perceive-surface (car surface-info))
                        (handle-detected-objects)))
  (poi-search))

  (roslisp::ros-info (execute-cleanup) "Plan execution took ~a seconds"
                     (- (get-universal-time) *start-time*))
  (roslisp::ros-info (execute-cleanup) "Of that ~a seconds were used for initialisation, ~a seconds were used for detection ~a seconds for movement and ~a seconds for manipulation actions the remaining ~a seconds were used for querying for information from the database"
                     *total-init-time*
                     *total-detection-time*
                     *total-movement-time*
                     *total-manipulation-time*
                     (- (- (get-universal-time) *start-time*)
                        *total-detection-time*
                        *total-movement-time*
                        *total-manipulation-time*
                        *total-init-time*)))


;;(defun robocup-floor-check ()
;;  (let ((nav-pose (cl-tf2::make-pose-stamped
;;                   "map"
;;                   0
;;                   (cl-tf2::make-3d-vector 0.3567 0.744 0)
;;                   (cl-tf2::q* (cl-tf2::make-quaternion 0 0 0 1)
;;                      (cl-tf2::euler->quaternion
;;                       :ax 0
;;                       :ay 0
;;                       :az (/ pi 1.5)))))
;;        (nav-pose-grasp (cl-tf2::make-pose-stamped
;;                         "map"
;;                         0
;;                         (cl-tf2::make-3d-vector 0.3567 0.744 0)
;;                         (cl-tf2::q* (cl-tf2::make-quaternion 0 0 0 1)
;;                                     (cl-tf2::euler->quaternion
;;                                      :ax 0
;;                                      :ay 0
;;                                      :az 0.75))))
;;        (look-at-pose (llif::prolog-surface-pose (first (llif::prolog-cleanup-surfaces)))))
;;    (comf::move-hsr nav-pose)
;;    (llif::call-take-gaze-pose-action
;;     :px (first (first look-at-pose))
;;     :py (second (first look-at-pose))
;;     :pz 0)
;;    (let ((detected-objects (llif::call-robosherlock-object-pipeline (vector "robocup_default") t)))
;;      (llif::insert-knowledge-objects (comf::get-confident-objects detected-objects))
;;      (llif::call-take-pose-action 1)
;;      (comf::move-hsr nav-pose-grasp)
;;      (handle-detected-objects))))
    

;;@author Torge Olliges
(defun handle-detected-objects ()
  (loop
    do
       (let ((next-object (llif::prolog-next-object)))
         (when (or (eq next-object nil)
                   (eq next-object 1)) (return))
         (let ((source-surface (llif::prolog-object-source next-object))
               (target-surface (llif::prolog-object-goal next-object)))
           (when (or (eq source-surface nil) (search "Floor" source-surface))
             (setf *last-inner-timestamp* (get-universal-time))
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
             (setf *total-movement-time*
                            (+
                             (- (get-universal-time) *last-inner-timestamp*)
                             *total-movement-time*))
             (deliver-object next-object target-surface)
             (return-from handle-detected-objects))
           (progn 
             ;;(comf::announce-movement-to-surface "future" source-surface)
             (roslisp::ros-info (handle-found-objects)
                                "Moving to source surface of ~a namely ~a" next-object source-surface)
             (setf *last-inner-timestamp* (get-universal-time))
             (comf::move-to-surface source-surface nil)
             (setf *total-movement-time*
                            (+
                             (- (get-universal-time) *last-inner-timestamp*)
                             *total-movement-time*)))
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
    (setf *last-inner-timestamp* (get-universal-time))
    (comf::move-to-surface target-surface nil)
    (setf *total-movement-time*
          (+
           (- (get-universal-time) *last-inner-timestamp*)
           *total-movement-time*)))
  (progn
    ;;(comf::announce-place-action "future" next-object)
    (roslisp::ros-info (handle-found-objects)
                       "Trying to place ~a from ~a on ~a" next-object source-surface target-surface)
    (setf *last-inner-timestamp* (get-universal-time))
    (llif::call-take-pose-action 6)
    (comf::place-object next-object 1)
    (setf *total-manipulation-time*
          (+
           (- (get-universal-time) *last-inner-timestamp*)
           *total-manipulation-time*)))
  
  (setf *last-inner-timestamp* (get-universal-time))
  (llif::call-take-pose-action 1)
  (llif::call-take-pose-action 1)
  (setf *total-manipulation-time*
          (+
           (- (get-universal-time) *last-inner-timestamp*)
           *total-manipulation-time*)))

;;@author Philipp Klein
(defun poi-search ()
  "approaching the next poi and bring the found object to the basket. When no poi is found, start the poi-search and loop the behavior."
  (comf::move-hsr (cl-tf::make-pose-stamped "map" 0 (cl-tf::make-3d-vector 1 0.5 0) (cl-tf::make-quaternion 0 0 0 1)))
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

        (roslisp::with-fields (detectiondata)
          confident-objects
        (progn
          (roslisp::ros-info (poi-search) "Number of objects detected: ~a" (length detectiondata))
          (if (> (length detectiondata) 0)
              (llif::insert-knowledge-objects confident-objects)
              (progn
                (llif::poi-remover (cl-tf::origin poi-pos) 0.4)
                (return-from continue)))))


        (let ((next-object (llif::prolog-next-object)))
          (when (eq next-object 1)
            (progn(llif::poi-remover (cl-tf::origin poi-pos) 0.4) (return-from continue)))
          
      ;; turn to face the object
      (roslisp::with-fields (translation rotation)
        (cl-tf::lookup-transform cram-tf::*transformer* "map" "base_footprint")
        (llif::call-nav-action-ps 
            (cl-tf::make-pose-stamped "map" 0 translation
                (cl-tf::q* rotation
                           (cl-tf::euler->quaternion :ax 0 :ay 0 :az (- (/ pi 2)))))))
      
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

(defun reset-timestamps ()
  (setf *start-time* 0)
  (setf *last-timestamp* 0)
  (setf *last-inner-timestamp* 0)
  (setf *total-init-time* 0)
  (setf *total-movement-time* 0)
  (setf *total-detection-time* 0)
  (setf *total-manipulation-time* 0))
