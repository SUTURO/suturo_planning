(in-package :comp)
(defvar *perceptionData*)
(defvar *object-id*)

(defun execute-m1()

  ;;Get into perceive position
  (llif:call-nav-action -0.8 0.7 -1.5)
  (llif::call-perceive-action 0)

  ;;perceive
  (setq *perceptionData* (llif::call-robosherlock-pipeline))

  ;;Get into grasp position
  (llif::call-perceive-action 1)
  (llif::call-nav-action -0.8 0.7 3.15)

  ;;Insert found objects into knowledge base
	(llif::insert-knowledge-objects *perceptionData*)

  ;;Get next object
  (setq *object-id* (comp:next-object "table"))

  ;;Grasping object
  (comp:grasp-object *object-id* 1)
  
  ;;(comp:place-object *object-id*)
)


