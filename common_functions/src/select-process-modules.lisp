(in-package :comf)

(cram-prolog:def-fact-group available-hsr-process-modules (cpm:available-process-module
                                                            cpm:matching-process-module)

  (cram-prolog:<- (cpm:available-process-module hsr-navigation))
  (cram-prolog:<- (cpm:available-process-module hsr-text-to-speach))
  
  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-navigation)
    (desig:desig-prop ?desig (:type :going)))

  (cram-prolog:<- (cpm:available-process-module hsr-arm))

  (cram-prolog:<- (cpm:matching-process-module ?desig hsr-arm)
      (desig:desig-prop ?desig (:type :grasping)))

  (cram-prolog:<- (cpm:matching-process-module ?desig hsr-text-to-speach)
      (desig:desig-prop ?desig (:type :say)))

  (cram-prolog:<- (cpm:matching-process-module ?desig hsr-arm)
      (desig:desig-prop ?desig (:type :placing))))



