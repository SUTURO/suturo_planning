;;; Adapted from https://github.com/cram2/cram/blob/master/cram_boxy/cram_boxy_designators/src/motions.lisp
;;; use these instead: https://github.com/cram2/cram/tree/master/cram_pr2/cram_pr2_fetch_deliver_plans/src
(in-package :plc)
;; TODO Adapt to HSR

(cram-prolog:def-fact-group hsr-motion-designators (desig:motion-grounding)
  ;; for each kind of motion define a desig

  ;;;;;;;;;;;;;;;;;;;; BASE ;;;;;;;;;;;;;;;;;;;;;;;;
  (cram-prolog:<- (desig:motion-grounding ?designator (going ?pose))
    (desig:desig-prop ?designator (:type :going))
    (desig:desig-prop ?designator (:target ?pose)))

  (cram-prolog:<- (desig:motion-grounding ?designator (going goal-pose))
    (desig:desig-prop ?designator (:type :going))
    (desig:desig-prop ?designator (:x ?x))
    (desig:desig-prop ?designator (:y ?y))
    (desig:desig-prop ?designator (:angle ?angle)))
   
  ;;;;;;;;;;;;;;;;;;;; TORSO ;;;;;;;;;;;;;;;;;;;;;;;;

  ;; move robot up and down
  (cram-prolog:<- (desig:motion-grounding ?designator (move-torso ?height))
    (desig:desig-prop ?designator (:type :moving-torso))
    (desig:desig-prop ?designator (:height ?height))
    )
  
  ;;;;;;;;;;;;;;;;;;;; NECK ;;;;;;;;;;;;;;;;;;;;;;;;

  ;; looking at?
  (cram-prolog:<- (desig:motion-grounding ?designator (move-neck ?pos-vector))
    (desig:desig-prop ?designator (:type :looking))
    (desig:desig-prop ?designator (:direction ?pos-vector)))

  (cram-prolog:<- (desig:motion-grounding ?designator (look :front))
    (desig:desig-prop ?designator (:type :looking))
    (desig:desig-prop ?designator (:direction :front)))
  
  (cram-prolog:<- (desig:motion-grounding ?designator (look :perceive))
    (desig:desig-prop ?designator (:type :looking))
    (desig:desig-prop ?designator (:direction :perceive)))

  (cram-prolog:<- (desig:motion-grounding ?designator (look :perceive))
    (desig:desig-prop ?designator (:type :looking))
    (desig:desig-prop ?designator (:direction :perceive-down)))

  (cram-prolog:<- (desig:motion-grounding ?designator (look :safe))
    (desig:desig-prop ?designator (:type :looking))
    (desig:desig-prop ?designator (:direction :safe)))
  
  ;;;;;;;;;;;;;;;;;;;; GRIPPER ;;;;;;;;;;;;;;;;;;;;;;;;

  (cram-prolog:<- (desig:motion-grounding ?designator (move-gripper-joint :open))
    (desig:desig-prop ?designator (:type :opening)))

  (cram-prolog:<- (desig:motion-grounding ?designator (move-gripper-joint :close))
    (desig:desig-prop ?designator (:type :closing)))

;;  (cram-prolog:<- (desig:motion-grounding ?designator (move-gripper-joint :grip NIL ?effort))
;;    (desig:desig-prop ?designator (:type :gripping))
;;    (once (or (desig:desig-prop ?designator (:effort ?effort))
;;              (equal ?effort nil))))

  (cram-prolog:<- (desig:motion-grounding ?designator (move-gripper-joint nil ?position NIL))
    (desig:desig-prop ?designator (:type :moving-gripper-joint))
    (desig:desig-prop ?designator (:joint-angle ?position)))

  ;; (cram-prolog:<- (desig:motion-grounding ?designator (grasping ?obj-desig))
  ;;   (desig:desig-prop ?designator (:type :grasp))
  ;;   (desig:desig-prop ?designator (:obj ?obj-desig)))
 

  (cram-prolog:<- (desig:motion-grounding ?designator (move-gripper-joint :grip NIL ?effort))
    (or (desig:desig-prop ?motion-designator (:type :gripping))
        (desig:desig-prop ?motion-designator (:type :moving-gripper-joint))
        (and (desig:desig-prop ?motion-designator (:gripper ?_))
             (or (desig:desig-prop ?motion-designator (:type :opening))
                 (desig:desig-prop ?motion-designator (:type :closing))))))

  ;;;;;;;;;;;;;;;;;;;; ARM ;;;;;;;;;;;;;;;;;;;;;;;;
  (cram-prolog:<- (desig:motion-grounding ?designator (grasping ?pose ?weight
                                                                ?width ?height
                                                                ?depth ?modus))
    (desig:desig-prop ?designator (:type :grasping))
    (desig:desig-prop ?designator (:pose ?pose))
    (desig:desig-prop ?designator (:weight ?weight))
    (desig:desig-prop ?designator (:width ?width))
    (desig:desig-prop ?designator (:height ?height))
    (desig:desig-prop ?designator (:depth ?depth))
    (desig:desig-prop ?designator (:modus ?modus)))

  (cram-prolog:<- (desig:motion-grounding ?designator (placing ?pose ?weight
                                                               ?width ?height
                                                               ?depth ?modus))
    (desig:desig-prop ?designator (:type :placing))
    (desig:desig-prop ?designator (:pose ?pose))
    (desig:desig-prop ?designator (:weight ?weight))
    (desig:desig-prop ?designator (:width ?width))
    (desig:desig-prop ?designator (:height ?height))
    (desig:desig-prop ?designator (:depth ?depth))
    (desig:desig-prop ?designator (:modus ?modus)))

  (cram-prolog:<- (desig:motion-grounding ?designator (perceiving ?pose ?weight
                                                                 ?width ?height
                                                                 ?depth ?modus))
      (desig:desig-prop ?designator (:type :perceiving))
      (desig:desig-prop ?designator (:pose ?pose))
      (desig:desig-prop ?designator (:weight ?weight))
      (desig:desig-prop ?designator (:width ?width))
      (desig:desig-prop ?designator (:height ?height))
      (desig:desig-prop ?designator (:depth ?depth))
      (desig:desig-prop ?designator (:modus ?modus)))

  (cram-prolog:<- (desig:motion-grounding ?designator (perceiving-high ?pose ?weight
                                                                   ?width ?height
                                                                   ?depth ?modus))
    (desig:desig-prop ?designator (:type :perceiving-high))
    (desig:desig-prop ?designator (:pose ?pose))
    (desig:desig-prop ?designator (:weight ?weight))
    (desig:desig-prop ?designator (:width ?width))
    (desig:desig-prop ?designator (:height ?height))
    (desig:desig-prop ?designator (:depth ?depth))
    (desig:desig-prop ?designator (:modus ?modus)))

  (cram-prolog:<- (desig:motion-grounding ?designator (perceiving-side ?pose ?weight
                                                                   ?width ?height
                                                                   ?depth ?modus))
    (desig:desig-prop ?designator (:type :perceiving-side))
    (desig:desig-prop ?designator (:pose ?pose))
    (desig:desig-prop ?designator (:weight ?weight))
    (desig:desig-prop ?designator (:width ?width))
    (desig:desig-prop ?designator (:height ?height))
    (desig:desig-prop ?designator (:depth ?depth))
    (desig:desig-prop ?designator (:modus ?modus)))

  

                                          
  ;;;;;;;;;;;;;;;;;;;; SPEECH ;;;;;;;;;;;;;;;;;;;;;;;;
  (cram-prolog:<- (desig:motion-grounding ?designator (say ?text))
    (desig:desig-prop ?designator (:type :say))
    (desig:desig-prop ?designator (:text ?text)))

  ;;;;;;;;;;;;;;;;;;;; PERCEPTION ;;;;;;;;;;;;;;;;;;;;;;;;
  (cram-prolog:<- (desig:motion-grounding ?designator (perceive ?surface-list))
    (desig:desig-prop ?designator (:type :perceive))
    (desig:desig-prop ?designator (:surface ?surface-list)))
)
