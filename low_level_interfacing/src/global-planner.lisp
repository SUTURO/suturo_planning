;;; global-planner client
(in-package :llif)

(defun global-planner-reachable (from to)
    "initializes the global-planner client"
    (if (not (wait-for-service "planner/make_plan" 10))
      (ros-warn nil "Timed out waiting for service global-planner-make-plan")
      (roslisp:with-fields (plan_found)
          (call-service "planner/make_plan" 'navfn-srv:makenavplan
                        :start (cl-tf:make-pose-stamped-msg from "map" (roslisp:ros-time))
                        :goal (cl-tf:make-pose-stamped-msg to "map" (roslisp:ros-time)))
        (if (= plan_found 1) 1 Nil))))


