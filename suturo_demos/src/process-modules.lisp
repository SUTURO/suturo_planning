(in-package :su-demos)

;;All of the following is Designator/PM stuff.
(defmacro with-hsr-process-modules (&body body)
  "Receives a body of lisp code `body'. Runs the code contained in `body' with all the necessary process modules"
  `(cram-process-modules:with-process-modules-running
       (hsr-navigation
        giskard::giskard-pm common-desig:wait-pm
        rk:robokudo-perception-pm)
     (cpl-impl::named-top-level (:name :top-level),@body)))


;;Process module itself
(cram-process-modules:def-process-module hsr-navigation (motion-designator)
  "Receives motion-designator `motion-designator'. Calls the process module HSR-NAVIGATION with the appropriate designator."
  (destructuring-bind (command argument &rest args)
      (desig:reference motion-designator)
    (declare (ignore args))
    (ecase command
      (cram-common-designators:move-base
       (call-nav-action-ps argument)))))

;;Denotes the PM as avaivailable
(cram-prolog:def-fact-group available-hsr-process-modules (cpm:available-process-module
                                                           cpm:matching-process-module)

  (cram-prolog:<- (cpm:available-process-module hsr-navigation))
  
  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-navigation)
    (desig:desig-prop ?desig (:type :going))))


;;Designator inference rules
(cram-prolog:def-fact-group hsr-motion-designators (desig:motion-grounding)

  (cram-prolog:<- (desig:motion-grounding ?designator (going ?pose))
    (desig:desig-prop ?designator (:type :going))
    (desig:desig-prop ?designator (:target ?pose)))

  (cram-prolog:<- (desig:motion-grounding ?designator (going goal-pose))
    (desig:desig-prop ?designator (:type :going))
    (desig:desig-prop ?designator (:x ?x))
    (desig:desig-prop ?designator (:y ?y))
    (desig:desig-prop ?designator (:angle ?angle))))

;; ------
(prolog:def-fact-group giskard-pm (cpm:matching-process-module
                                   cpm:available-process-module)

  (prolog:<- (cpm:matching-process-module ?motion-designator giskard:giskard-pm)
    (or (desig:desig-prop ?motion-designator (:type :moving-tcp))
        (desig:desig-prop ?motion-designator (:type :moving-gripper-joint))
        (desig:desig-prop ?motion-designator (:type :moving-arm-joints))
        (desig:desig-prop ?motion-designator (:type :pulling))
        (desig:desig-prop ?motion-designator (:type :pushing))
        (desig:desig-prop ?motion-designator (:type :going))
        (desig:desig-prop ?motion-designator (:type :moving-torso))
        (desig:desig-prop ?motion-designator (:type :moving-custom))
        (desig:desig-prop ?motion-designator (:type :looking))
        (desig:desig-prop ?motion-designator (:type :closing-gripper))
        (desig:desig-prop ?motion-designator (:type :opening-gripper))))

  (prolog:<- (cpm:available-process-module giskard:giskard-pm)
    (prolog:not (cpm:projection-running ?_))))



