(in-package :comp)
(defvar *goal*)

(defun place-object (object-id)
  (setq *goal* (llif:prolog-object-goal-pose object-id))

  (let ((px (nth 0 (nth 2 *pose*)))
        (py (nth 1 (nth 2 *pose*)))
        (pz (nth 2 (nth 2 *pose*)))
        (qv1 (nth 0 (nth 3 *pose*)))
        (qv2 (nth 1 (nth 3 *pose*)))
        (qv3 (nth 2 (nth 3 *pose*)))
        (qv4 (nth 3 (nth 3 *pose*))))
  (llif:call-place-action px py pz qv1 qv2 qv3 qv4 object-id)
  ))
 
