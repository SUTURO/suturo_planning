;;; global-planner client
(in-package :llif)

(defun global-planner-reachable (start-pose goal-pose)
    "initializes the global-planner client"
    (if (not (wait-for-service "planner/make_plan" 10))
      (roslisp::ros-warn (global-planner) "Timed out waiting for service global-planner-make-plan")
      (roslisp:with-fields (plan_found)
          (call-service "planner/make_plan" 'navfn-srv:makenavplan
                        :start (cl-tf::to-msg start-pose)
                        :goal (cl-tf::to-msg goal-pose))
        (if (eq plan_found 1) 1 Nil))))


