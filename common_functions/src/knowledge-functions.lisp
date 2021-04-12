(in-package :comf)
;;TODO: fix indentation similar to high-level/grocery-execute
;;TODO: or remove? is it unnecessary?
(defvar *objects* NIL)
(defvar *robot-pose-stamped* NIL)
(defvar *robot-transform* NIL)

(defun next-object (from)
    "get the next Object to grasp"
    (setq *objects* (list nil))
     (case (intern (string-upcase from))
        (table (setq *objects* (llif:prolog-table-objects)))
        (shelf (setq *objects* (llif:prolog-all-objects-in-shelf)))
        ;;(pose (setq *objects* (llif:prolog-objects-around-pose (get-pose) threshold)))
        ;;TODO: use new prolog query to knowledge)
    (if (null *objects*) 
        (error 'no-object)))
    (first *objects*))
       

(defun get-pose ()
  ;;get robot position
  (setf *robot-transform* 
        (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))
  ;;create robot pose
  (setf *robot-pose-stamped* (cl-transforms:make-pose 
                (cl-tf:origin *robot-pose-stamped*) 
                (cl-tf:orientation *robot-pose-stamped*))))

(defun reachability-check-place (object-id object-dimensions grasp-mode)
    (let 
        (try-make-plan-result 
            (llif::try-make-plan-action
                (cl-tf2::stamped-transform->pose-stamped  
                    (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))
                object-id
                (prolog-object-goal-pose->pose-stamped (llif::prolog-object-goal-pose object-id))
                object-dimensions
                grasp-mode
                'Grasp))
        ;; do smth with the result
    ))

(defun reachability-check-place (object-id object-dimensions grasp-mode)
    (let (try-make-plan-result 
            (llif::try-make-plan-action
                (cl-tf2::stamped-transform->pose-stamped  
                    (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))
                object-id
                (prolog-object-goal-pose->pose-stamped (llif::prolog-object-goal-pose object-id))
                object-dimensions
                grasp-mode
                'Place))
        ;; do smth with the result
    ))

(defun prolog-object-goal-pose->pose-stamped (prolog-object-goal-pose)
    (let* ((?x (nth 0 (nth 0 prolog-object-goal-pose)))
            (?y (nth 1 (nth 0 prolog-object-goal-pose)))
            (?z (nth 2 (nth 0 prolog-object-goal-pose)))
            (?qx (nth 0 (nth 1 prolog-object-goal-pose)))
            (?qy (nth 1 (nth 1 prolog-object-goal-pose)))
            (?qz (nth 2 (nth 1 prolog-object-goal-pose)))
            (?qw (nth 3 (nth 1 prolog-object-goal-pose)))
            (stamped-pose 
                (cl-tf2::make-pose-stamped "map" 0 
                    (cl-tf2::make-3d-vector ?x ?y ?z)
                    (cl-tf2::make-quaternion ?qx ?qy ?qz ?qw)))) 
        stamped-pose))