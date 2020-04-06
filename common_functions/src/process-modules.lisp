(in-package :comf)

(defmacro with-hsr-process-modules (&body body)
    `(cram-process-modules:with-process-modules-running
        (hsr-navigation
        hsr-arm
        hsr-text-to-speach
        hsr-perceive)
        (cpl-impl::named-top-level (:name :top-level), @body)))

 ;;;;;;;;;;;;;;;;;;;; BASE ;;;;;;;;;;;;;;;;;;;;;;;;
 (cram-process-modules:def-process-module hsr-navigation (motion-designator)
    (roslisp:ros-info 
        (hsr-navigation-process-modules)
        "hsr-navigation called with motion designator `~a'."
        motion-designator)
    (destructuring-bind (command target) (desig:reference motion-designator)
        (format t "command ~a target ~a" command target)
        (ecase command
            (going
                (llif::call-nav-action-ps (desig:reference target))))))

 ;;;;;;;;;;;;;;;;;;;; TEXT-TO-SPEACH ;;;;;;;;;;;;;;;;;;;;;;;;
 (cram-process-modules:def-process-module hsr-text-to-speach (motion-designator)
    (roslisp:ros-info 
        (text-to-speach-process-modules)
        "text-to-speach called with motion designator `~a'."
        motion-designator)
    (destructuring-bind (command text) (desig:reference motion-designator)
        (ecase command
            (say
                (llif::call-text-to-speech-action text)))))


 ;;;;;;;;;;;;;;;;;;;; Perceive ;;;;;;;;;;;;;;;;;;;;;;;;
 (cram-process-modules:def-process-module hsr-perceive (motion-designator)
    (roslisp:ros-info 
        (perceive-process-modules)
        "perceive called with motion designator `~a'."
        motion-designator)
    (destructuring-bind (command plane) (desig:reference motion-designator)
        (ecase command
            (perceive
                (llif::call-robosherlock-pipeline plane)))))


 ;;;;;;;;;;;;;;;;;;;; Arm  ;;;;;;;;;;;;;;;;;;;;;;;;

(cram-process-modules:def-process-module hsr-arm (motion-designator)
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


