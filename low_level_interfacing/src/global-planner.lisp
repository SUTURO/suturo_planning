(in-package :llif)

;;@author Philipp Klein, @2ndAuthor Luca Krohm
(defun global-planner-reachable (start-pose goal-pose)
  "Receives start pose `start-pose' and goal pose `goal-pose'. Returns if the global-planner can find a path from start pose to goal pose"
  (cond
    ((not (wait-for-service "planner/make_plan" 10))
     (roslisp::ros-warn (global-planner)
                        "Timed out waiting for service global-planner-make-plan"))
    (t (roslisp:with-fields (plan_found)
           (call-service "planner/make_plan" 'navfn-srv:makenavplan
                         :start (cl-tf::to-msg start-pose)
                         :goal (cl-tf::to-msg goal-pose))
         (when (eq plan_found 1) T)))))

;;@author Philipp Klein
(defun global-planner-reachable-from-current-pose (goal-pose)
  "Receives goal pose `goal-pose'. Checks if the robot can reach the goal"
  (global-planner-reachable
   (roslisp::with-fields (translation rotation)
       (cl-tf::lookup-transform cram-tf::*transformer*
                                "map"
                                "base_footprint")
     (cl-tf::make-pose-stamped "map" 0 translation rotation))
   goal-pose))


