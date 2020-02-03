(in-package :comp)
(defvar *perceptionData*)
(defvar *object-id)

(defun execute-m1()
  (init-planning)
	;;call perceive takes torso_height and pose 
	(llif::call-perceive-action 0 2)
	;;call nav action takes x y z as goal position
	(llif::call-nav-action 0.2 1 3)
  ;;calls perception to trigger the pipeline 
  (setq *perceptionData* (llif::call-robosherlock-pipeline))
  ;;inserts the perception data into the knowledge base 
	(llif::insert-knowledge-objects *perceptionData*)
  ;; get the object id of the next object we want to grasp and place in the goal
  (setq *object-id* (comp::next-object table))
  ;;uses the object-id to grasp an object
  (comp::grasp-action *object-id* 1)
  ;;uses the object-id to place an object at its goal
  (comp::place-action *object-id*)
)


