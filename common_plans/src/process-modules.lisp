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
