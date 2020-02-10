(in-package :comp)

(defmacro with-hsr-process-modules (&body body)
  `(cram-process-modules:with-process-modules-running
       (comp::hsr-navigation)
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

 ;;;;;;;;;;;;;;;;;;;; Arm  ;;;;;;;;;;;;;;;;;;;;;;;;

(cram-process-modules:def-process-module hsr-grasp (motion-designator)
  (roslisp:ros-info (hsr-grasp-process-modules)
                    "hsr-grasp called with a motion designator `~a'."
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
                       ?graspmode
                       )
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
        ?object_id)))))
