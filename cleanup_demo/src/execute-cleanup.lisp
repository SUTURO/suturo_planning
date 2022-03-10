(in-package :clean)
(defparameter *start-time* 0)
(defparameter *last-timestamp* 0)
(defparameter *last-inner-timestamp* 0)
(defparameter *total-init-time* 0)
(defparameter *total-movement-time* 0)
(defparameter *total-detection-time* 0)
(defparameter *total-manipulation-time* 0)

;; used in cleanup
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
    ;;(poi-search)
    )

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



    
;; used in cleanup
;;@author Torge Olliges
(defun handle-detected-objects ()
  (loop
    do
       (let ((next-object (llif::prolog-next-object)))
         (when (or (eq next-object nil)
                   (eq next-object 1)) (return))
         (print next-object)
         (let ((source-surface (llif::prolog-object-source next-object))
               (target-surface (llif::prolog-object-goal next-object)))
           (print "Source-surface:")
           (print source-surface)
           (print "Target-surface:")
           (print target-surface)
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

;; used in cleanup
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



;; used in cleanup
(defun reset-timestamps ()
  (setf *start-time* 0)
  (setf *last-timestamp* 0)
  (setf *last-inner-timestamp* 0)
  (setf *total-init-time* 0)
  (setf *total-movement-time* 0)
  (setf *total-detection-time* 0)
  (setf *total-manipulation-time* 0))
