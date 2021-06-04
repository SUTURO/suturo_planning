(in-package :llif)

;;@author Philipp Klein
(defun global-planner-reachable (start-pose goal-pose)
    "returns if the global-planner can find a path from `start-pose' to `goal-pose'"
    (if (not (wait-for-service "planner/make_plan" 10))
      (roslisp::ros-warn (global-planner) "Timed out waiting for service global-planner-make-plan")
      (roslisp:with-fields (plan_found)
          (call-service "planner/make_plan" 'navfn-srv:makenavplan
                        :start (cl-tf::to-msg start-pose)
                        :goal (cl-tf::to-msg goal-pose))
        (if (eq plan_found 1) 1 Nil))))


