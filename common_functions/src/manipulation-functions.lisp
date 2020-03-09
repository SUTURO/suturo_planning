(in-package :comf)


(defvar *goal* nil)
(defvar *classgrasp* nil)
(defvar *dimensions* nil)
(defvar *pose* nil)
(defvar *classplace* nil)
(defvar *list*)
(defvar *result*)
(defvar ?place-list)

;;@author Jan Schimpf
;;todo add checks for nil;
;;add the text to speech: what class the object we place has,
;; the name of the goal and if the action is done;
(defun place-object (object-id grasp-pose)
  ;;(setf ?place-list place-list)
  (setf *dimensions* (llif:prolog-object-dimensions object-id))
  (setf *goal* (llif:prolog-object-goal-pose object-id))
  (let* ((?point-x-object (nth 0 (nth 0 *goal*)))
        (?point-y-object (nth 1 (nth 0 *goal*)))
        (?point-z-object (- (nth 2 (nth 0 *goal*)) 0.07))
        (?quaterion-value-1 (nth 0 (nth 1 *goal*)))
        (?quaterion-value-2 (nth 1 (nth 1 *goal*)))
        (?quaterion-value-3 (nth 2 (nth 1 *goal*)))
        (?quaterion-value-4 (nth 3 (nth 1 *goal*)))
        (?size-x (nth 0 *dimensions*))
        (?size-y (nth 1 *dimensions*))
        (?size-z (nth 2 *dimensions*))
        (?object-id object-id)
        (?grasp-mode grasp-pose)
   (place (desig:a motion
                    (:type :placing)
                    (:point-x ?point-x-object)
                    (:point-y ?point-y-object)
                    (:point-z ?point-z-object)
                    (:quaterion-value-1 ?quaterion-value-1)
                    (:quaterion-value-2 ?quaterion-value-2)
                    (:quaterion-value-3 ?quaterion-value-3)
                    (:quaterion-value-4 ?quaterion-value-4)
                    (:size-x ?size-x)
                    (:size-y ?size-y)
                    (:size-z ?size-z)
                    (:object-id ?object-id)
                    (:grasp-mode ?grasp-mode))))
    (print "Was here")
    (comf::with-hsr-process-modules (exe:perform
                                     place))))

(defun place-object-test (place-list)
  (let* ((?point-x-object (nth 0 place-list))
        (?point-y-object (nth 1 place-list))
        (?point-z-object (nth 2 place-list))
        (?quaterion-value-1 (nth 0 place-list))
        (?quaterion-value-2 (nth 1 place-list))
        (?quaterion-value-3 (nth 2 place-list))
        (?quaterion-value-4 (nth 3 place-list))
        (?size-x (nth 0 place-list))
        (?size-y (nth 1 place-list))
        (?size-z (nth 2 place-list))
        (?object-id object-id)
        (?grasp-mode grasp-pose)
   (place (desig:a motion
                    (:type :placing)
                    (:point-x ?point-x-object)
                    (:point-y ?point-y-object)
                    (:point-z ?point-z-object)
                    (:quaterion-value-1 ?quaterion-value-1)
                    (:quaterion-value-2 ?quaterion-value-2)
                    (:quaterion-value-3 ?quaterion-value-3)
                    (:quaterion-value-4 ?quaterion-value-4)
                    (:size-x ?size-x)
                    (:size-y ?size-y)
                    (:size-z ?size-z)
                    (:object-id ?object-id)
                    (:grasp-mode ?grasp-mode))))
    (comf::with-hsr-process-modules (exe:perform
                                     place))))  

;;@author Jan Schimpf
;;todo add checks for nil;
;; what class the object we grasp has,
;; the name of the goal and if the action is done;
;; add ways to handle failure and recoveery
(defun grasp-object (object-id grasp-pose)
  (setq *dimensions* (llif:prolog-object-dimensions object-id))
  (setq *pose* (llif:prolog-object-pose object-id))
  
  ;;say:grasping object of class *class*
  (let* ((?point-x-object (nth 0 (nth 2 *pose*)))
        (?point-y-object (nth 1 (nth 2 *pose*)))
        (?point-z-object (nth 2 (nth 2 *pose*)))
        (?quaterion-value-1 (nth 0 (nth 3 *pose*)))
        (?quaterion-value-2 (nth 1 (nth 3 *pose*)))
        (?quaterion-value-3 (nth 2 (nth 3 *pose*)))
        (?quaterion-value-4 (nth 3 (nth 3 *pose*)))
        (?size-x (nth 0 *dimensions*))
        (?size-y (nth 1 *dimensions*))
        (?size-z (nth 2 *dimensions*))
        (?object-id object-id)
        (?grasp-mode grasp-pose)
        (grasp (desig:a motion
                    (:type :grasping)
                    (:point-x ?point-x-object)
                    (:point-y ?point-y-object)
                    (:point-z ?point-z-object)
                    (:quaterion-value-1 ?quaterion-value-1)
                    (:quaterion-value-2 ?quaterion-value-2)
                    (:quaterion-value-3 ?quaterion-value-3)
                    (:quaterion-value-4 ?quaterion-value-4)
                    (:size-x ?size-x)
                    (:size-y ?size-y)
                    (:size-z ?size-z)
                    (:object-id ?object-id)
                    (:grasp-mode ?grasp-mode))))
    (format t "desig: ~a" grasp)
    (print "war HIER")
    (comf::with-hsr-process-modules (exe:perform
                                     grasp))))
                                          

;;@author Jan Schimpf
(defun create-place-list (object-id grasp-pose)
  (setf *dimensions* (llif:prolog-object-dimensions object-id))
  (setf *goal* (llif:prolog-object-goal-pose object-id))

  (let* ((point-x-object (nth 0 (nth 0 *goal*)))
        (point-y-object (nth 1 (nth 0 *goal*)))
        (point-z-object (nth 2 (nth 0 *goal*)))
        (quaterion-value-1 (nth 0 (nth 1 *goal*)))
        (quaterion-value-2 (nth 1 (nth 1 *goal*)))
        (quaterion-value-3 (nth 2 (nth 1 *goal*)))
        (quaterion-value-4 (nth 3 (nth 1 *goal*)))
        (size-x (nth 0 *dimensions*))
        (size-y (nth 1 *dimensions*))
        (size-z (nth 2 *dimensions*))
        (?list (list (list point-x-object point-y-object point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size-x size-y size-z object-id grasp-pose)
                        (list (+ point-x-object 0.05) point-y-object point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size-x size-y size-z object-id grasp-pose)
                        (list (- point-x-object 0.05) point-y-object point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size-x size-y size-z object-id grasp-pose)
                        (list point-x-object (+ point-y-object 0.05) point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size-x size-y size-z object-id grasp-pose)
                        (list point-x-object (- point-y-object 0.05) point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size-x size-y size-z object-id grasp-pose))))   
    ?list))

