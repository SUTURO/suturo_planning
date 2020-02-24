(in-package :llif)

(defmacro with-hsr-process-modules (&body body)
  `(cram-process-modules:with-process-modules-running
       (llif::hsr-navigation
        llif::hsr-arm
        )
     (cpl-impl::named-top-level (:name :top-level)
       ,@body)))

 ;;;;;;;;;;;;;;;;;;;; BASE ;;;;;;;;;;;;;;;;;;;;;;;;
 (cram-process-modules:def-process-module hsr-navigation (motion-designator)
  (roslisp:ros-info (hsr-navigation-process-modules)
                     "hsr-navigation called with motion designator `~a'."
                     motion-designator)
  (destructuring-bind (command target) (desig:reference motion-designator)
    (ecase command
      (going
       (llif::call-nav-action-ps (desig:reference target)))
    )))

 ;;;;;;;;;;;;;;;;;;;; TEXT-TO-SPEACH ;;;;;;;;;;;;;;;;;;;;;;;;
 (cram-process-modules:def-process-module hsr-text-to-speach (action-designator)
  (roslisp:ros-info (text-to-speach-process-modules)
                     "text-to-speach called with motion designator `~a'."
                     motion-designator)
  (destructuring-bind (command target) (desig:reference motion-designator)
    (ecase command
      (say
       (llif::call-text-to-speech-action (desig:reference target))
       ))))


 ;;;;;;;;;;;;;;;;;;;; Arm  ;;;;;;;;;;;;;;;;;;;;;;;;

(cram-process-modules:def-process-module hsr-arm (motion-designator)
  (roslisp:ros-info (hsr-arm-process-modules)
                   "hsr-arm called with a motion designator `~a'."
                   motion-designator)
 (destructuring-bind (command
                      ?point-x
                      ?point-y
                      ?point-z
                      ?quaterion-value-1
                      ?quaterion-value-2
                      ?quaterion-value-3
                      ?quaterion-value-4
                      ?size_x
                      ?size_y
                      ?size_z
                      ?object_id
                      ?graspmode)
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
       ?size_x
       ?size_y
       ?size_z
       ?object_id
       ?graspmode))
     (placeing
      (llif::call-place-action
       ?point-x
       ?point-y
       ?point-z
       ?quaterion-value-1
       ?quaterion-value-2
       ?quaterion-value-3
       ?quaterion-value-4
       ?object_id
       ?graspmode)))))
