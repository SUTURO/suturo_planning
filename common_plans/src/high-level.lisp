(in-package :comp)
(defvar *perceptionData*)
(defvar *min-confidence* 0.5)
(defvar *color* "red")

(defun execute-m1()
  (init-planning)
	;;call perceive takes torso_height and pose 
	(llif::call-perceive-action 0 2)
	;;call nav action takes x y z as goal position
	(llif::call-nav-action 0.2 1 3)
  (setq *perceptionData* (llif::call-robosherlock-pipeline))
	(llif::insert-knowledge-objects *perceptionData*)
  (sorting *perceptionData*)
	(llif::call-place-action -0.87 0.824 0.28 0 0.7 0 0.7)
)

(defun sorting(msg)
  (let ((px (roslisp:msg-slot-value msg :x))
        (py (roslisp:msg-slot-value msg :y))
        (pz (roslisp:msg-slot-value msg :z))
        (pw (roslisp:msg-slot-value msg :w))
        (ph (roslisp:msg-slot-value msg :h))
        (pd (roslisp:msg-slot-value msg :d))
        (confidence (roslisp:msg-slot-value msg :confidence))
        (color-of-object (roslisp:msg-slot-value msg :color_name)))

    (if (equalp color-of-object *color*)
        (llif::call-grasp-action px py (+ pz (/ pd 2)) 0.0 0.7 0.0 0.7 pw ph pd)
)))

(defun checkConfidence(msg)
  (loop for i from 1 to (length msg) 
  do(checkElementConfidence(aref msg i)
  )))

;;(defun checkElementConfidence(msg)
;;(let ((*px* (roslisp:msg-slot-value msg :x))
;;       (*py* (roslisp:msg-slot-value msg :y))
;;        (*pz* (roslisp:msg-slot-value msg :z))
;;        (*pw* (roslisp:msg-slot-value msg :w))
;;        (*ph* (roslisp:msg-slot-value msg :h))
;;        (*pd* (roslisp:msg-slot-value msg :d))
;;        (*confidence* (roslisp:msg-slot-value msg :confidence))
;;        (*color-of-object* (roslisp:msg-slot-value msg :color_name)))

;;    (if (< *min-confidence* *confidence* )
         
;;)))
