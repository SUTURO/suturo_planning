(in-package :comf)

;;used in go-get-it
;;@author Torge Olliges
(defun prolog-object-goal-pose->pose-stamped (prolog-object-goal-pose)
  "Receives goal pose `prolog-object-goal-pose'. Transforms a prolog object goal pose into a stamped pose"
  (let* ((?x (nth 0 (nth 0 prolog-object-goal-pose)))
         (?y (nth 1 (nth 0 prolog-object-goal-pose)))
         (?z (nth 2 (nth 0 prolog-object-goal-pose)))
         (?qx (nth 0 (nth 1 prolog-object-goal-pose)))
         (?qy (nth 1 (nth 1 prolog-object-goal-pose)))
         (?qz (nth 2 (nth 1 prolog-object-goal-pose)))
         (?qw (nth 3 (nth 1 prolog-object-goal-pose)))
         (stamped-pose 
           (cl-tf2::make-pose-stamped
            "map" 0 
            (cl-tf2::make-3d-vector ?x ?y ?z)
            (cl-tf2::make-quaternion ?qx ?qy ?qz ?qw)))) 
    stamped-pose))

;; used in cleanup
;; @author Torge Olliges                             
(defun current-pose-prolog-pose->distance (prolog-pose-values)
  "Receives pose `prolog-pose-values'. Returns the robots distance to the pose"
  (let ((current-pose-translation
          (cl-transforms::translation
           (cl-tf::lookup-transform cram-tf::*transformer*
                                    "map"
                                    "base_footprint")))
        (prolog-pose-vector (cl-tf2::make-3d-vector
                             (first prolog-pose-values)
                             (second prolog-pose-values)
                             (third prolog-pose-values))))
    (cl-transforms:v-dist current-pose-translation prolog-pose-vector)))

;; used in cleanup
;; @author Torge Olliges
(defun tuple-compare (left right)
  "Receives values `left' and `right'. Compares those two values"
  (< (nth 1 left) (nth 1 right)))

;; used in cleanup
;; @author Torge Olliges
(defun sort-surfaces-by-distance (surface-names)
  "Receive surface names `surface-names'. Returns surfaces sorted by distance"
  (sort
   (mapcar
    (lambda (surface-name) (list
                            surface-name
                            (current-pose-prolog-pose->distance
                             (first (llif::prolog-surface-pose surface-name)))))
    surface-names)
   #'tuple-compare))



