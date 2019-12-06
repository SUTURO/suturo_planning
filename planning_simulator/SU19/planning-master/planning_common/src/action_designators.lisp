(in-package :plc)

(cram-prolog:def-fact-group hsr-action-designators (action-grounding)
  (cram-prolog:<- (desig:action-grounding ?desig (grasping ?obj))
    (desig:desig-prop ?desig (:type :grasping))
    (desig:desig-prop ?desig (:obj ?obj)) ;; Or location-desig
    ) ;; or object desig

    
  (cram-prolog:<- (desig:action-grounding ?desig (perceiving ?base-pose ?head-pose))
    (desig:desig-prop ?desig (:type :perceiving))
    (desig:desig-prop ?desig (:base-pose ?base-pose)) ;;where robot should stand to perceive
    (desig:desig-prop ?desig (:head-pose ?head-pose)))

  (cram-prolog:<- (desig:action-grounding ?desig (perceiving))
    (desig:desig-prop ?desig (:type :perceiving))
    (desig:desig-prop ?desig (:base-pose ?base-pose)))

  ;;;;;;;;;;;;;;;;;;;;;; ARM ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (cram-prolog:<- (desig:action-grounding ?desig (grasping ?obj-desig))
    (desig:desig-prop ?desig (:type :grasping))
    (desig:desig-prop ?desig (:object ?obj-desig)) ;; can be a location desig
    )
 ;;;; OTHER

  (cram-prolog:<- (desig:action-grounding ?desig (say ?text))
    (desig:desig-prop ?desig (:type :say))
    (desig:desig-prop ?desig (:text ?text)))
  
  )

