(in-package :comf)

;;@autho Torge Olliges
(defun reachability-check (object-ids)
  (every
   (lambda (x) (eq x T))
   (mapcar 
    (lambda 
     (object-id) 
     (if (eq 0 (reachability-check-grasp object-id 1))
       T  
       (if (eq 0 (reachability-check-grasp object-id 2))
         T
         (llif::set-object-not-graspable object-id 1))))
      object-ids)))

;;@autho Torge Olliges
(defun reachability-check-grasp (object-id grasp-mode)
    (let 
        ((make-plan-result 
            (llif::try-make-plan-action
                (cl-tf2::stamped-transform->pose-stamped  
                    (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))
                object-id
                (prolog-object-goal-pose->pose-stamped (llif::prolog-object-goal-pose object-id))
                (llif::prolog-object-dimensions object-id)
                grasp-mode
                'Grasp)))
        make-plan-result))

;;@autho Torge Olliges
(defun reachability-check-place (object-id grasp-mode)
    (let ((make-plan-result 
            (llif::try-make-plan-action
                (cl-tf2::stamped-transform->pose-stamped  
                    (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))
                object-id
                (prolog-object-goal-pose->pose-stamped (llif::prolog-object-goal-pose object-id))
                (llif::prolog-object-dimensions object-id)
                grasp-mode
                'Place)))
        make-plan-result))

;;@autho Torge Olliges
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
