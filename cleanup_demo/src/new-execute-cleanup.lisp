(in-package :clean)

;; used in cleanup
;; author Luca Krohm
(defun new-execute-cleanup ()
  "Resets timestamps, measures the time the whole plan takes and announces everything measured at the end"
  (comf::reset-timestamps)
  (comf::measure-time :start
    (start-cleanup))
  (comf::announce-final-time))

;; used in cleanup
;; author Luca Krohm
(defun start-cleanup ()
  "Robot goes into the standart position and officially starts the plan"
  (init-cleanup)  
  (comf::with-hsr-process-modules
    (comf::measure-time :manipulation
      (llif::call-take-pose-action 1))
    (comf::announce-plan-start "clean up")
    (mapc #'cleanup-room (llif::prolog-all-rooms))))
    ;;(poi-search)))

;; used in cleanup
;;@author Luca Krohm
(defun init-cleanup ()
  "Initialize the interfaces, and measures time it takes to initialize them"
  (comf::measure-time :init
    (init-interfaces)))
;; generalize and put into time-measurement
;; (roslisp::ros-info (execute-cleanup) "Initialisation took ~a seconds"
;;                    *total-init-time*))

;; used in cleanup
;;@author Luca Krohm
(defun cleanup-room (room)
  "Receives the current room `room'. Gets the surfaces in that room, sort them by distance and set them as not visited"
  (mapc #'set-surface-not-visited (comf::sort-surfaces-by-distance
                                   (llif::prolog-room-surfaces
                                    (llif::prolog-current-room))))
  ;; sort all surfaces marked as not visited by distance
  ;; and start cleaning them up one after another
  (mapc #'next-cleanup (comf::sort-surfaces-by-distance
                        (llif::prolog-surfaces-not-visited-in-room
                         room))))

;; used in cleanup
;;@author Luca Krohm
(defun set-surface-not-visited (surface-info)
  "Receives the surface info `surface-info'. Makes sure it has not been visited and lets knowledge know that the surface not visited"
  (let* ((surface (car surface-info))
         (surface-region-name (llif::prolog-surface-region surface))
         (surface-names '("bin" "tray" "chair" "container" "Shelf" "Floor")))
    ;; if non of the surface-names are being found in surface or surface-region-name
    ;; set all surfaces as "not visited"
    (when (and (notany (lambda (n)                       
                         (search n surface))                                
                       surface-names)
               (notany (lambda (n)
                         (search n surface-region-name))
                       surface-names))
      (roslisp::ros-info (execute-cleanup)
                         "Surface set not visited, surface: ~a region name: ~a"
                         surface
                         surface-region-name)
      ;; set all surfaces as "not visited"
      (llif::prolog-set-surface-not-visited (car surface-info)))))

;; used in cleanup
;;@author Luca Krohm
(defun next-cleanup (surface-info)
  "Receives the surface info `surface-info'. Initiates the cleaning process for the next surface `surface-info'"
  ;; move robot to surface
  (comf::measure-time :movement
    (comf::move-to-surface (car surface-info) t))
  ;; perceive the surface
  (comf::perceive-surface (car surface-info))
  ;; if objects were detected, handle them accordingly
  (handle-detected-objects))

;; used in cleanup
;;@author Torge Olliges, Luca Krohm
(defun handle-detected-objects ()
  "Gets the next object `next-object', target surface `target-surface' and source surface `source-surface' from Knowledge, and gets ready to pick up the next object" 
  (let ((next-object (llif::prolog-next-object)))
    ;; if there is not next object, exit "handle-detected-objects
    (unless (or (eq next-object nil)
                (eq next-object 1))
      (print next-object)
      (let ((source-surface (llif::prolog-object-source next-object))
            (target-surface (llif::prolog-object-goal next-object)))
        (print "Source-surface:")
        (print source-surface)
        (print "Target-surface:")
        (print target-surface)
        (cond
          ;; if there is no surface being found, or the surface is floor
          ;; handle the object according to handle-floor-surface
          ((or (eq source-surface nil)
               (search "Floor" source-surface))
           (handle-floor-surface next-object target-surface))
          ;; otherwise handle it according to handle-surfaces
          (t (handle-surfaces next-object target-surface source-surface)))
        (handle-detected-objects)))))

;; used in cleanup
;;@author Torge Olliges, Luca Krohm
(defun handle-surfaces (next-object target-surface source-surface)
  "Receives the next object `next-object', target surface `target-surface' and source surface `source-surface'. Moves to source surface, and handles the next object."
  ;;(comf::announce-movement-to-surface "future" source-surface)
  (roslisp::ros-info (handle-found-objects)
                     "Moving to source surface of ~a namely ~a"
                     next-object
                     source-surface)
  ;; move robot to surface where the object was detected
  (comf::measure-time :movement
    (comf::move-to-surface source-surface nil))
  ;; pick up next-object and deliver it to the target-surface
  (deliver-object next-object target-surface source-surface))

;; used in cleanup
;;@author Torge Olliges, Luca Krohm
(defun handle-floor-surface (next-object target-surface)
  "Receives the next object `next-object', target surface `target-surface'. Gets called by HANDLE-DETECTED-OBJECTS if the source surface is 'Floor', and handles the objects detected there"
  (comf::measure-time :movement
    (comf::move-hsr
     ;; apparently someone hardcoded the position
     ;; where the robot should go when handling the
     ;; floor as a surface
     (cl-tf2::make-pose-stamped
      "map" 0
      (cl-tf2::make-3d-vector 0.3567 0.744 0)
      (cl-tf2::q* (cl-tf2::make-quaternion 0 0 0 1)
                  (cl-tf2::euler->quaternion :ax 0 :ay 0 :az 0.75)))))
  (deliver-object next-object target-surface))

;; used in cleanup
;; @author Luca Krohm
(defun deliver-object (next-object target-surface &optional source-surface)
  "Receives the next object `next-object', target surface `target-surface' and an optional source surface `source-surface'. Tries to grasp the next object and, if it succeeded delivers it to the target surface"
  ;;(comf::announce-grasp-action "future" next-object)
  (roslisp::ros-info (handle-found-objects)
                     "Trying to grasp ~a"
                     next-object)
  (comf::grasp-handling next-object)
  (let ((objects-in-gripper (llif::prolog-object-in-gripper)))
    (roslisp::ros-info (deliver-object)
                       "Objects in gripper ~a"
                       objects-in-gripper)
    (cond
      ((eq (length objects-in-gripper) 0)
       (llif::prolog-set-object-handled next-object))
      (t (handle-object-in-gripper next-object target-surface source-surface)))))

;; used in cleanup
;; @author Luca Krohm
(defun handle-object-in-gripper (next-object target-surface &optional source-surface)
  "Receives the next object `next-object', target surface `target-surface' and an optional source surface `source-surface'. Moves to target surface and places next-object"
  ;;(comf::announce-movement-to-surface "future" target-surface)
  (comf::ros-info (handle-found-objects)
                  "Moving to target surface of ~a namely ~a"
                  next-object
                  target-surface)
  (comf::measure-time :movement
    (comf::move-to-surface target-surface nil))
  ;;(comf::announce-place-action "future" next-object)
  (roslisp::ros-info (handle-found-objects)
                     "Trying to place ~a from ~a on ~a"
                     next-object
                     source-surface
                     target-surface)
  (comf::measure-time :manipulation
    (llif::call-take-pose-action 6)
    (comf::place-object next-object 1)
    (llif::call-take-pose-action 1)))




