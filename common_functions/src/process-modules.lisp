(in-package :comf)

(defmacro with-hsr-process-modules (&body body)
  "Receives a body of lisp code `body'. Runs the code contained in `body' with all the necessary process modules"
  `(cram-process-modules:with-process-modules-running
       (hsr-navigation
        hsr-arm
        hsr-text-to-speach
        hsr-perceive
        hsr-wipe
        hsr-drawer)
     (cpl-impl::named-top-level (:name :top-level),@body)))

 ;;;;;;;;;;;;;;;;;;;; BASE ;;;;;;;;;;;;;;;;;;;;;;;;
(cram-process-modules:def-process-module hsr-navigation (motion-designator)
  "Receives motion-designator `motion-designator'. Calls the process module HSR-NAVIGATION with the appropriate designator."
  (destructuring-bind (command argument &rest args)
      (desig:reference motion-designator)
    (declare (ignore args))
    (ecase command
      (cram-common-designators:move-base
       (llif::call-nav-action-ps argument)))))



;; (roslisp:ros-info 
;;     (hsr-navigation-process-modules)
;;     "hsr-navigation called with motion designator `~a'."
;;     motion-designator)
;; (destructuring-bind (command target) (desig:reference motion-designator)
;;     (format t "command ~a target ~a" command target)
;;     (ecase command
;;         (going
;;             (llif::call-nav-action-ps (desig:reference target))))))

 ;;;;;;;;;;;;;;;;;;;; TEXT-TO-SPEECH ;;;;;;;;;;;;;;;;;;;;;;;;
(cram-process-modules:def-process-module hsr-text-to-speach (motion-designator)
   "Receives motion-designator `motion-designator'. Calls the process module HSR-TEXT-TO-SPEACH with the appropriate designator."
  (roslisp:ros-info (text-to-speach-process-modules)
                    "text-to-speach called with motion designator `~a'."
                    motion-designator)
  (destructuring-bind (command text) (desig:reference motion-designator)
    (ecase command
      (say
       (llif::call-text-to-speech-action text)))))


 ;;;;;;;;;;;;;;;;;;;; Perceive ;;;;;;;;;;;;;;;;;;;;;;;;
(cram-process-modules:def-process-module hsr-perceive (motion-designator)
   "Receives motion-designator `motion-designator'. Calls the process module HSR-PERCEIVE with the appropriate designator."
  (roslisp:ros-info (perceive-process-modules)
                    "perceive called with motion designator `~a'."
                    motion-designator)
  (destructuring-bind (command plane) (desig:reference motion-designator)
    (ecase command
      (perceive
       (llif::call-robosherlock-pipeline plane)))))


 ;;;;;;;;;;;;;;;;;;;; Arm  ;;;;;;;;;;;;;;;;;;;;;;;;

(cram-process-modules:def-process-module hsr-arm (motion-designator)
   "Receives motion-designator `motion-designator'. Calls the process module HSR-ARM with the appropriate designator."
  (roslisp:ros-info (hsr-arm-process-modules)
                    "hsr-arm called with a motion designator `~a'."
                    motion-designator)
  (destructuring-bind 
      (command
       ?point-x
       ?point-y
       ?point-z
       ?quaterion-value-1
       ?quaterion-value-2
       ?quaterion-value-3
       ?quaterion-value-4
       ?size-x
       ?size-y
       ?size-z
       ?object-id
       ?grasp-mode)
      (desig:reference motion-designator)
    (ecase command
      (grasping
       (llif::call-grasp-action
        ?point-x
        ?point-y
        ?point-z
        ?quaterion-value-1
        ?quaterion-value-2
        ?quaterion-value-3
        ?quaterion-value-4
        ?size-x
        ?size-y
        ?size-z
        ?object-id
        ?grasp-mode))
      (placing
       (llif::call-place-action
        ?point-x
        ?point-y
        ?point-z
        ?quaterion-value-1
        ?quaterion-value-2
        ?quaterion-value-3
        ?quaterion-value-4
        ?object-id
        ?grasp-mode)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;WIPE;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;@author Felix Krause
(cram-process-modules:def-process-module hsr-wipe (motion-designator)
   "Receives motion-designator `motion-designator'. Calls the process module HSR-WIPE with the appropriate designator."
  (roslisp:ros-info (hsr-wipe-process-modules)
                    "hsr-wipe called with a motion designator `~a'."
                    motion-designator)
  (destructuring-bind 
      (command
       ?point-x
       ?point-y
       ?point-z
       ?quaternion-value-1
       ?quaternion-value-2
       ?quaternion-value-3
       ?quaternion-value-4)
      (desig:reference motion-designator)
    (ecase command
      (wiping
       (llif::call-wipe-action
        ?point-x
        ?point-y
        ?point-z
        ?quaternion-value-1
        ?quaternion-value-2
        ?quaternion-value-3
        ?quaternion-value-4)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;DRAWER;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;@author Felix Krause
(cram-process-modules:def-process-module hsr-drawer (motion-designator)
   "Receives motion-designator `motion-designator'. Calls the process module HSR-DRAWER with the appropriate designator."
  (roslisp:ros-info (hsr-open-process-modules)
                    "hsr-drawer called with a motion designator `~a'."
                    motion-designator)
  (destructuring-bind 
      (command
       ?point-x
       ?point-y
       ?point-z
       ?quaternion-value-1
       ?quaternion-value-2
       ?quaternion-value-3
       ?quaternion-value-4
       ?mode
       ?name)
      (desig:reference motion-designator)
    (ecase command
      (openclose
       (llif::call-drawer-action
        ?point-x
        ?point-y
        ?point-z
        ?quaternion-value-1
        ?quaternion-value-2
        ?quaternion-value-3
        ?quaternion-value-4
        ?mode
	?name)))))
