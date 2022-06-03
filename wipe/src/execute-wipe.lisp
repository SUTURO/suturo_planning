

(in-package :wipe)

(defparameter *surface-to-clean* "tall_table:table:table_center")
(defparameter *drawer* "drawer:drawer:drawer_center")
(defparameter *drawer-knob* "iai_kitchen/drawer:drawer:drawer_knob")
(defparameter *tall-table* "long_table:table:table_center")
(defparameter *knows-object* nil)
(defparameter *drawer-name* nil)
(defparameter *surface-name* nil)
(defparameter *starting-point* nil)
(defparameter *all-objects* nil)

;;; @author Felix Krause
(defun execute-wipe ()
 "Resets timestamps, measures the time the whole plan takes and announces everything measured at the end"
  (comf::reset-timestamps)
  (comf::measure-time :start
    (start-wipe))
  (comf::announce-final-time))

;; @author Felix Krause
(defun start-wipe ()
"Robot goes into the standart position and officially starts the plan."
  (init-wipe)
  (comf::with-hsr-process-modules
    (llif::call-take-pose-action 1)
    (comf::announce-plan-start "wipe")
    (let ((surface (llif:prolog-surface-from-urdf *surface-to-clean*))
          (drawer (llif:prolog-surface-from-urdf *drawer*)))
      (llif::prolog-add-default-surface "Blegoduplo" drawer)
      (setf *drawer-name* drawer)
      (setf *surface-name* surface)
      (llif::prolog-set-surface-not-visited surface)
      (roslisp::ros-info (execute-wipe) "Moving to source-surface.")
      (comf::move-to-surface surface t)
      (roslisp::ros-info (execute-wipe) "Percieving Objects and Surface.")
      (comf::perceive-surface surface)
      (roslisp::ros-info (execute-wipe) "Handling Objects.")
      (setf *all-objects* (llif::prolog-get-objects))
      (handle-detected-objects)
      ;;This is great for testing purposes. So we are leaving this here.
      ;; (open-drawer drawer)
      ;; (comf::move-to-surface drawer t)
      ;; (comf::find-object "foam_brick" drawer)
      ;; (comf::move-to-surface drawer nil)
      ;; (llif::call-take-pose-action 8)
      ;; (take-object (llif::prolog-get-object "Blegoduplo") drawer)
      ;; (wipe-surface surface)
      ;; (comf::move-to-surface drawer nil)
      ;; (llif::call-take-pose-action 8)
      ;; (comf::place-object (llif::prolog-get-object "Blegoduplo") 1 :wipe-return 0)
      ;;(close-drawer drawer)
      (roslisp::ros-info (execute-wipe) "Wiping Surface.")
      (wipe-surface-with-sponge surface *knows-object*)
      (roslisp::ros-info (execute-wipe) "Returning Objects to original Surface.")
      (mapcar (lambda (x)
                (deliver-object x surface :source-surface
                                (llif::prolog-temporary-storage-surface) :phase :wipe-return))
              (llif::prolog-stored-objects)))
      (roslisp::ros-info (execute-wipe) "Finished Plan.")))

;; @author Luca Krohm
(defun init-wipe ()
  "Initialize the interfaces, and measures time it takes to initialize them"
  (comf::measure-time :init
    (init-interfaces)))

    
;; used in cleanup
;;@author Torge Olliges, Luca Krohm
(defun handle-detected-objects ()
  "Gets the next object `next-object', target surface `target-surface' and source surface `source-surface' from Knowledge, and gets ready to pick up the next object" 
  (let* ((next-object (pop *all-objects*))
         ;; only using "Blegoduplo" since perception doesn't correctly perceive "foam_brick"
         (sponge (type-check next-object "Foambrick")))
    ;; if there is not next object, exit handle-detected-objects
    (unless (or  (eq next-object 1)
                 (eq next-object nil))
      (cond (sponge (setf *knows-object* next-object)
                     (roslisp::ros-info (perception)
                                     "Saw object ~a" *knows-object*))
            (T
             (print next-object)
             (let ((source-surface (llif::prolog-object-source next-object))
                   (target-surface (llif::prolog-temporary-storage-surface)))
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
                 (t (handle-surfaces next-object target-surface source-surface))))))
               (handle-detected-objects))))

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
  (deliver-object next-object target-surface :source-surface source-surface))

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


;; @author Luca Krohm
(defun deliver-object (next-object target-surface &key source-surface (phase :wipe))
  "Receives the next object `next-object', target surface `target-surface' and an optional source surface `source-surface'. Tries to grasp the next object and, if it succeeded delivers it to the target surface"
  ;;(comf::announce-grasp-action "future" next-object)
  (when source-surface
    (comf::move-to-surface source-surface  nil))  
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
      (t (handle-object-in-gripper next-object target-surface :source-surface source-surface :phase phase)))))

;; @author Luca Krohm
(defun handle-object-in-gripper (next-object target-surface &key source-surface phase)
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
    (comf::place-object next-object 1 phase)
    (llif::call-take-pose-action 1)))


  ;;@author Felix Krause
(defun wipe-surface (surface)
  "Receives urdf-surface `surface'. Starts the process of wiping a surface."
  (comf::move-to-surface surface nil)
    (let ((object-length (llif::prolog-object-dimensions (llif::prolog-object-in-gripper))))
      (cond
        ((equal object-length 0)
         (roslisp::ros-warn (wipe-surface)
                            "No object in gripper found, abandoning wiping of surface ~a"
                            surface))
        (t (comf::perform-wipe-motion surface)))
      (llif::call-take-pose-action 1)))


;;@author Felix Krause
(defun open-drawer (drawer)
  "Receives urdf-surface `drawer'. Opens the specified drawer."
  ;;Locating and moving to the Drawer.
  (roslisp::ros-info (open-drawer) "Moving to Drawer")
  (comf::move-to-surface drawer t)
  (roslisp::ros-info (open-drawer) "Percieving Drawer.")
  ;;Grabbing the Handle and Opening the Drawer.
  (roslisp::ros-info (open-drawer) "Opening Drawer")
  (comf::perform-drawer-pose (comf::perceive-surface-drawer drawer) 1 *drawer-knob*)
  ;;This is here to prevent the hsr from bumping into the drawer when taking a neutral position afterwards. This can be fixed by roatating the gripper and arm first. I couldn't do this due to time constraints.
  (comf::move-hsr
    (cl-tf2::make-pose-stamped
     "map" 0
      (cl-tf2::make-3d-vector 0.15 0.7 0.95)
      (cl-tf2::make-quaternion 0 0 -0.7071 0.7071)))
  (llif::call-take-pose-action 1)
  (comf::move-to-surface drawer t))


;;@author Felix Krause
(defun close-drawer (drawer)
  "Receives urdf-surface `drawer'. Closes the specified drawer."
  ;;Locating and Moving to the Drawer.
  (roslisp::ros-info (close-drawer) "Moving to Drawer")
  (comf::move-to-surface drawer t)
  (roslisp::ros-info (close-drawer) "Percieving Drawer.")
  (comf::perceive-surface-drawer drawer)
  (comf::move-to-surface drawer nil)
  ;;Grabbing the handle and closing the Drawer.
  (roslisp::ros-info (close-drawer) "Closing Drawer")
  (comf::perform-drawer-pose (comf::perceive-surface-drawer drawer) 0 *drawer-knob*)
  (llif::call-take-pose-action 1)
  (llif::call-take-pose-action 1))
  

;; @author Luca Krohm
(defun type-check (next-object object-type)
  "Receives next object `next-object' and object type `object-type'. Checks if `next-object' is of type `object-type'."
  (when (stringp next-object)
    (format t "checking ~a for ~a"
            next-object
            object-type)
    (search object-type next-object)))

;; @author Luca Krohm 
(defmacro with-surface-handling (surface-type surface &body body)
  "Receives surface type `surface-type' of surface `surface' and lisp expression `body'.
   Makes sure that `surface' of type surface type is handled correctly before evaluating `body'"
  `(progn
     (print "Starting macro")
     (alexandria::switch (,surface-type :test #'equal)
        ("drawer"  (open-drawer ,surface))
        ("table" (comf::move-to-surface ,surface nil))
        (t  (roslisp::ros-warn (surface-handling)
                               "No special surface-handling found for surfacetype ~a. Evaluating body as given." ,surface-type)))
     ,@body
     (print "Continuing macro")
     (alexandria::switch (,surface-type :test #'equal)
        ("drawer"  (close-drawer ,surface))
        ("table" nil)
        (t  nil))))

;; @author Luca Krohm
(defun wipe-surface-with-sponge (surface knows-object)
  "Receives surface `surface' and string `knows-object' which may contain the sponge name or nil.
  Either uses the sponge `knows-object' to wipe the table or asks knowledge for the default surface of the sponge."
  (cond
    (knows-object (take-object knows-object surface)
                  (wipe-surface surface)
                  (comf::place-object knows-object 1 :wipe))
    (t (let* ((surfaces (get-object-location 'sponge)) ;;"Blegoduplo")) 
              (obj-surface (car surfaces))
              (surface-type (llif::prolog-get-surface-type obj-surface)))
         (roslisp::ros-info (surface-handling)
                            "Handling surface ~a of type ~a"
                            obj-surface
                            surface-type)
         (with-surface-handling surface-type obj-surface(comf::find-object "Blegoduplo" obj-surface)
                                                         (let* ((object (llif::prolog-get-object "Blegoduplo"))
                                                                (amount (take-object object obj-surface)))
                                                           ;;(setf amount 1) ;; for test purposes
                                                           (unless (eq amount 0)
                                                             (wipe-surface surface)
                                                             (comf::move-to-surface obj-surface nil)
                                                             (comf::place-object object 1 :wipe-return 0.3))))))))

;; @author Luca Krohm
(defun take-object (object surface)
  "Receives object `object', and surface `surface'. Takes the object from the specified surface."
  (comf::move-to-surface surface nil)
  (llif::call-take-pose-action 8)
  (roslisp::ros-info (handle-found-objects)
                     "Trying to take ~a"
                     object)
     (comf::grasp-handling object)
     (let* ((objects-in-gripper (llif::prolog-object-in-gripper))
            (amount (length objects-in-gripper)))
       (when (eq amount 0)
         (llif::prolog-set-object-handled object))
       amount))

;; @author Luca Krohm
(defun get-object-location (object-type)
  "Receives object type `object-type'. Asks prolog for the default surface of the object type."
  (roslisp::ros-info (get-object-location)
                     "Getting location of object ~a"
                     object-type)
  ;; (llif::prolog-get-object-surfaces object-type))
  (case object-type
    ('sponge (list *drawer-name*))  
    (t nil)))


