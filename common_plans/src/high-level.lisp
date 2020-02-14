(in-package :comp)
(defvar *perceptionData*)
(defvar *object-id*)
(defvar *table-objects* NIL)

(defun execute-m1()

  ;;Get into perceive position
  (llif:call-nav-action -0.8 0.7 -1.5)
  (llif::call-take-pose-action 2)

  ;;perceive
  (setq *perceptionData* (llif::call-robosherlock-pipeline))

  ;;Get into grasp position
  (llif::call-take-pose-action 1)
  (llif::call-nav-action -0.8 0.7 3.15)

  ;;Insert found objects into knowledge base
	(llif::insert-knowledge-objects *perceptionData*)

  ;;Get next object
  (setq *object-id* (comp:next-object))

  ;;Grasping object
  (comp:grasp-object *object-id* 1)
  
  ;;(comp:place-object *object-id*)
  )


(defun next-object ()
    "get the next Object to grasp"
  (setf *table-objects* (llif:prolog-table-objects))
  (first *table-objects*))
