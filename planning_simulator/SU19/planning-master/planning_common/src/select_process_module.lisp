(in-package :plc)

(cram-prolog:def-fact-group available-hsr-process-modules (cpm:available-process-module
                                                            cpm:matching-process-module)

  (cram-prolog:<- (cpm:available-process-module hsr-navigation))
  (cram-prolog:<- (cpm:available-process-module hsr-motion))
  (cram-prolog:<- (cpm:available-process-module hsr-say))
  (cram-prolog:<- (cpm:available-process-module hsr-arm-motion))
  (cram-prolog:<- (cpm:available-process-module hsr-torso))
  (cram-prolog:<- (cpm:available-process-module hsr-perception))
 
  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-navigation)
    (desig:desig-prop ?desig (:type :going)))

  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-motion)
    (desig:desig-prop ?desig (:type :looking)))

  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-torso)
    (desig:desig-prop ?desig (:type :moving-torso)))

  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-motion)
    (desig:desig-prop ?desig (:type :opening)))
  
  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-motion)
    (desig:desig-prop ?desig (:type :closing)))
  
  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-arm-motion)
    (desig:desig-prop ?desig (:type :grasping)))
  
  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-arm-motion)
    (desig:desig-prop ?desig (:type :placing)))
  
  ;; (cram-prolog:<- (matching-process-module ?desig  hsr-say-process-modules)
  ;;   (desig:desig-prop ?desig (:type :say)))

  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-say)
    (desig:desig-prop ?desig (:type :say)))

  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-perception)
    (desig:desig-prop ?desig (:type :perceive)))

  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-arm-motion)
    (desig:desig-prop ?desig (:type :perceiving-high)))

  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-arm-motion)
    (desig:desig-prop ?desig (:type :perceiving-side)))

  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-arm-motion)
    (desig:desig-prop ?desig (:type :perceiving)))
  )
