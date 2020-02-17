(in-package :llif)

(defmacro with-hsr-process-modules (&body body)
  `(cram-process-modules:with-process-modules-running
       (llif::hsr-navigation)
       (llif::hsr-arm)
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
       (llif::call-nav-action-ps (desig:reference target))
       ))))

 ;;;;;;;;;;;;;;;;;;;; TEXT-TO-SPEACH ;;;;;;;;;;;;;;;;;;;;;;;;
 (cram-process-modules:def-process-module text-to-speach (action-designator)
  (roslisp:ros-info (text-to-speach-process-modules)
                     "text-to-speach called with action designator `~a'."
                     action-designator)
  (destructuring-bind (command target) (desig:reference action-designator)
    (ecase command
      (saying
       (llif::call-text-to-speech-action (desig:reference target))
       ))))


 ;;;;;;;;;;;;;;;;;;;; Arm  ;;;;;;;;;;;;;;;;;;;;;;;;

(cram-process-modules:def-process-module hsr-arm (motion-designator)
  (roslisp:ros-info (hsr-arm-process-modules)
                    "hsr-arm called with a motion designator `~a'."
                    motion-designator)
  (destructuring-bind (command
                       ?px
                       ?py
                       ?pz
                       ?ox
                       ?oy
                       ?oz
                       ?ow
                       ?size_x
                       ?size_y
                       ?size_z
                       ?object_id
                       ?graspmode)
      (desig:reference motion-designator)
    (ecase command
      (grasping
       (llif::call-grasp-action
        ?px
        ?py
        ?pz
        ?ox
        ?oy
        ?oz
        ?ow
        ?size_x
        ?size_y
        ?size_z
        ?object_id
        ?graspmode))
      (placeing
       (llif::call-place-action
        ?px
        ?py
        ?pz
        ?ox
        ?oy
        ?oz
        ?ow
        ?object_id
        ?graspmode)))))
