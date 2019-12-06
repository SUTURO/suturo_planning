(in-package :plc)

(defvar *object-dimensions* NIL)
(defvar *width-offset* 0.05)
;; TODO move this to knowledge:
(defvar *height-obj-in-gripper* NIL)
(defparameter *placing-z-offset* 0.05)
(defparameter *placing-x-offset* 0.0)
(defparameter *placing-y-offset* 0.1)


(cpl:def-cram-function go-to (&key (to NIL) (facing NIL) (manipulation NIL))
  "go to a predefined location"
  ;;NOTE the publish-callange-step is done in the dynamic-poses.lisp
    (let* ((?to-say (concatenate 'string "I am going to the " (write-to-string to)))
           (?pose (case to
                    (:SHELF
                     (case facing
                       (:PERCEIVE (plc::pose-infront-shelf :manipulation manipulation :rotation :LEFT))
                       (T (plc::calculate-possible-poses-from-obj "environment/shelf_center"
                                                                  :facing-direction facing
                                                                  :relative-to :SHELF
                                                                  :manipulation manipulation))))
                    
                    (:TABLE
                     (case facing
                       (:PERCEIVE (plc::pose-infront-table :manipulation manipulation :rotation :RIGHT))
                       
                       (T (plc::calculate-possible-poses-from-obj "environment/table_front_edge_center"
                                                                  :facing-direction facing
                                                                  :relative-to :TABLE
                                                                  :manipulation manipulation))))
                    
                    (T (plc::calculate-possible-poses-from-obj to
                                                        :facing-direction :OBJECT
                                                        :relative-to :TABLE
                                                        :manipulation manipulation))))

           
           (move (desig:a motion
                          (:type :going)
                          (:target (desig:a location
                                            (:pose ?pose))))))
      (format t "~% +++ POSE NAV+++ : ~a ~%" ?pose)
      (cpl:seq
        (plc::say ?to-say)
        ;; (cram-executive:perform rotate) ;;TODO debug. calculate direction to face
        (cram-executive:perform move))))

;;; -----
(cpl:def-cram-function perceive-table ()
  "move head, torso and perceive"
  ;;(lli:publish-challenge-step 3)
  ;;(plc::perceive-side)
  ;;(plc::go-to :to :TABLE :facing :PERCEIVE :manipulation NIL)
  (plc::go-to :to :TABLE :facing :TABLE :manipulation NIL)
  (plc::go-to :to :TABLE :facing :PERCEIVE :manipulation NIL)
  ;;(plc::turn :LEFT)
  (plc::perceive-side)
  
   (plc::move-torso (plc::table-head-difference))    
   ;;(plc::go-to :to :TABLE :facing :PERCEIVE :manipulation NIL)

  (plc::move-head :right-down-3)
  (plc::say "I am going to perceive the table now..")
  ;; TODO this is a hack
 ;;(plc::turn :RIGHT)
    
  
  (plc::perceive (vector "robocup_table"))
;;  (plc::go-to :to :TABLE :facing :SHELF :manipulation T)
  )

;;assuming robot is already standing infront of the shelf
;;TODO implement the following
;; (let* ((shelf-T-base (cl-tf2:lookup-transform
;;                           (plc::get-tf-listener)
;;                            "base_footprint"
;;                             "environment/shelf_center"))
;;              (y (cl-tf:y (cl-tf:translation shelf-T-base)))
;;              (left-or-right (if (> y 0.0)
;;                                 "LEFT"
;;                                 "RIGHT")))
;;   (intern (concatenate 'string left-or-right "-1") :keyword))


(cpl:def-cram-function perceive-shelf ()
  "move head, torso and perceive"
  ;;(lli:publish-challenge-step 1)
  (plc::go-to :to :SHELF :facing :PERCEIVE :manipulation T)
  ;;high
  (cpl:par 
    (plc::go-to :to :SHELF :facing :PERCEIVE :manipulation NIL)
    (plc::perceive-side))
  ;;high
  ;;(cpl:par 
  ;;   (plc::move-torso (plc::shelf-head-difference "3"))
  ;;   (plc::move-head :left-down))
  ;; (plc::perceive (vector "robocup_shelf_3"))
  
  ;;middle
   (cpl:par
     (plc::move-torso (plc::shelf-head-difference "2"))
     (plc::move-head :left-down-2))
  (plc::perceive (vector "robocup_shelf_2"))

  ;;low
  ;; (cpl:par
  ;;   (plc::move-torso (plc::shelf-head-difference "0"))
  ;;   (plc::move-head :left-down-3))
  ;; (plc::perceive (vector "robocup_shelf_1"))
  ;; (plc::perceive (vector "robocup_shelf_0"))
  (plc::base-pose))




;; -----
(cpl:def-cram-function grasp-object (&optional (?modus NIL))
  "grasp object"
  ;;(lli:publish-challenge-step 4)
    (let* ((all-table-objects (lli:prolog-table-objects))
           (closest-object (plc:frame-closest-to-robot all-table-objects))
           (closest-object-pose (cl-tf2:lookup-transform (plc:get-tf-listener)
                                                         "map" closest-object :timeout 5))
           (object-class (lli:object-name->class closest-object))
           (?pose (cl-tf:make-pose (cl-tf:translation closest-object-pose)
                                   (cl-tf:rotation closest-object-pose)))
           (dimensions (lli:prolog-object-dimensions closest-object))

           (?weight 1.2)
           (?width (- (first dimensions) *width-offset*))
           (?depth (second dimensions))
           (?height (- (third dimensions) 0.08))
           (?modus (if (equal ?modus NIL)
                       (transform->grasp-side closest-object)))

           (grasp (desig:a motion
                           (:type :grasping)
                           (:pose ?pose)
                           (:weight ?weight)
                           (:width ?width)
                           (:height ?height)
                           (:depth ?depth)
                           (:modus ?modus))))
      ;;vars
      ;;(pc::publish-marker-pose ?pose)
      (setq *height-obj-in-gripper* ?height)
      (format t "Object Class: ~a " object-class)
      (format t "MODUS: ~a " ?modus)

      (if (equal ?modus "TOP")
          (plc::go-to :to :TABLE :facing :TABLE :manipulation T)
          (plc::go-to :to closest-object :facing :OBJECT :manipulation T))

      (format t "grasp mode: ~a" ?modus)
      ;; movement
      (setq *object-dimensions* dimensions)
      ;;(planning-communication::publish-marker-pose ?pose)
      (cpl:par
        (plc::move-head :safe)
        (plc::say (concatenate 'String "I am going to grasp the " object-class " now."))
        (cram-executive:perform grasp))
      
      (cpl:par
        (plc::say "done grasping")
        (plc::perceive-side)
        (plc::go-to :to :SHELF :facing :SHELF :manipulation T))))

;; FRONT TOP
(cpl:def-cram-function place-object (?modus)
  "place object"
  ;;(lli:publish-challenge-step 6)
  (cpl:seq
    (let* ((?context NIL)
           (pose-from-prolog (multiple-value-bind (pose context)
                                 (lli:prolog-object-goal-pose (lli:prolog-object-in-gripper))
                               (setq ?context context)
                               pose))
           

           (?pose (cl-tf:make-pose
                   (cl-tf:make-3d-vector (+ (first (car pose-from-prolog)) *placing-x-offset*)
                                         (+ (second (car pose-from-prolog)) *placing-y-offset*)
                                         (+ (third (car pose-from-prolog)) *placing-z-offset*))
                   (cl-tf:make-quaternion (first (second pose-from-prolog))
                                          (second (second pose-from-prolog))
                                          (third (second pose-from-prolog))
                                          (fourth (second pose-from-prolog)))))
        
           (?weight 1.2)
           (?width (first *object-dimensions*))
           (?depth 0.0)
           (?height *height-obj-in-gripper*)
           
           (place (desig:a motion
                              (:type :placing)
                              (:pose ?pose)
                              (:weight ?weight)
                              (:width ?width)
                              (:height ?height)
                              (:depth ?depth)
                              (:modus ?modus))))

      (if (>= (third *object-dimensions*)
                            (second *object-dimensions*))
                        (progn
                          (setq ?height (+ (third *object-dimensions*)))
                          (setq ?depth (second *object-dimensions*)))
                        (progn
                          (setq ?height (+ (second *object-dimensions*)))
                          (setq ?depth (third *object-dimensions*))))
      
      ;;(lli:publish-marker-pose ?pose)
      (plc::say ?context)
      (cram-executive:perform place))))




;; minor plans /very basic ones

(cpl:def-cram-function move-head (?position)
  "moves head into the desired position. Accepts either a vector with two values,
or one of the following: :perceive :safe :front"  
    (let* ((look-at (desig:a motion
                             (:type :looking)
                             (:direction ?position))))      
      (cram-executive:perform look-at)))

(cpl:def-cram-function say (?text)
  "speaks the given text"
  ;;(lli:publish-robot-text ?text)
    (let* ((say-text (desig:a motion
                             (:type :say)
                             (:text ?text))))     
      (cram-executive:perform say-text))
  )

(cpl:def-cram-function move-torso (?height)
  "moves torso to given height. keeps the arm out of sight." 
    (let* ((move-torso (desig:a motion
                             (:type :moving-torso)
                             (:height ?height))))
      (cram-executive:perform move-torso)))


;; for table call
(cpl:def-cram-function perceive (?surface)
  (let* ((perceive-desig (desig:a motion
                                  (:type :perceive)
                                  (:surface ?surface))))
    
    (cpl:par
      (plc::say "Now, perceiving.")
      (cram-executive:perform perceive-desig))
    (plc::say "Done perceiving.")))
    
(cpl:def-cram-function base-pose ()
  ;;(lli:publish-challenge-step 0)
  ;;(lli:publish-operator-text "Toya, please clean up the table")
  (let* ((?pose (cl-tf:make-identity-transform))
         (?nil NIL)
         (?zero 0.0)
         (perceive (desig:a motion
                         (:type :perceiving)
                         (:pose ?pose)
                         (:weight ?zero)
                         (:width ?zero)
                         (:height ?zero)
                         (:depth ?zero)
                         (:modus ?nil))))
    (cpl:par
      (plc::say "moving into base pose")
      (cram-executive:perform perceive))))

(cpl:def-cram-function perceive-high ()
  (let* ((?pose (cl-tf:make-identity-transform))
         (?nil NIL)
         (?zero 0.0)
         (perceive (desig:a motion
                         (:type :perceiving-high)
                         (:pose ?pose)
                         (:weight ?zero)
                         (:width ?zero)
                         (:height ?zero)
                         (:depth ?zero)
                         (:modus ?nil))))
    (cpl:par
      (plc::say "moving into perceive high pose.")
      (cram-executive:perform perceive))))

(cpl:def-cram-function perceive-side ()
  (let* ((?pose (cl-tf:make-identity-transform))
         (?nil NIL)
         (?zero 0.0)
         (perceive (desig:a motion
                         (:type :perceiving-side)
                         (:pose ?pose)
                         (:weight ?zero)
                         (:width ?zero)
                         (:height ?zero)
                         (:depth ?zero)
                         (:modus ?nil))))
    (cpl:par
      (plc::say "moving into perceive side pose.")
      (cram-executive:perform perceive))))
         

(cpl:def-cram-function turn (direction)
  "turns 90 degrees LEFT or RIGHT"
  (let* ((?rotation (cram-tf:rotate-pose
                    (plc::transform-stamped->pose-stamped
                      (cl-tf2:lookup-transform
                       (plc::get-tf-listener)
                       "map"
                       "base_footprint"))
                    :z (case direction
                         (:LEFT (/ pi 2))
                         (:RIGHT (/ pi -2)))))
         
         (move (desig:a motion
                          (:type :going)
                          (:target (desig:a location
                                            (:pose ?rotation))))))
    (cram-executive:perform move)))
