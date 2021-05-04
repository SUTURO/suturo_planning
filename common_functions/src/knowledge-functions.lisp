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
                (cl-tf2::transform-stamped->pose-stamped  
                    (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))
                object-id
                (prolog-object-goal-pose->pose-stamped (llif::prolog-object-goal-pose object-id))
                (let ((dimensions (llif::prolog-object-dimensions object-id)))
                  (cl-tf2::make-3d-vector
                   (nth 0 dimensions)
                   (nth 1 dimensions)
                   (nth 2 dimensions)))
                grasp-mode
                98)))
      (roslisp:ros-info (reachability-check-grasp) "Reachability check result: ~a" make-plan-result)
        make-plan-result))

;;@autho Torge Olliges
(defun reachability-check-place (object-id grasp-mode)
  (let ((make-plan-result 
            (llif::try-make-plan-action
                (cl-tf2::stamped-transform->pose-stamped  
                    (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))
                object-id
                (prolog-object-goal-pose->pose-stamped (llif::prolog-object-goal-pose object-id))
                (let ((dimensions (llif::prolog-object-dimensions object-id)))
                  (cl-tf2::make-3d-vector (nth 0 dimensions) (nth 1 dimensions) (nth 2 dimensions)))
                grasp-mode
                99)))
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

;;@author Torge Olliges, Jan Schimpf
(defun get-nav-pose-for-surface (surface-id)
  (let ((surface-pose (llif::prolog-surface-pose surface-id))
        (surface-edge-pose (llif::prolog-surface-front-edge-pose surface-id)))

    (cl-tf2::make-pose-stamped "map" 0
     (cl-tf2::make-3d-vector
     ;; adds the x value of the Vector to the edge to create an offset
      (+ (first (first surface-edge-pose))
        ;; creates the x value of the Vector from Center to Edge
        (* (- (first (first surface-edge-pose))
           (first (first surface-pose))) 1.5))

     ;; adds the y value of the Vector to the edge to create an offset
     (+ (second (first surface-edge-pose))
        ;; creates the y value of the Vector from Center to Edge
        (* (- (second (first surface-edge-pose))
           (second (first surface-pose))) 1.5))
     0)
    (cl-tf2::make-quaternion (first (second surface-edge-pose))
                             (second (second surface-edge-pose))
                             (third (second surface-edge-pose))
                             (fourth (second surface-edge-pose)))
    )))
