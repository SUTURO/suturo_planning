(in-package :su-real)

;;All of the following is Designator/PM stuff.
(defmacro with-hsr-process-modules (&body body)
  "Receives a body of lisp code `body'. Runs the code contained in `body' with all the necessary process modules"
  `(cram-process-modules:with-process-modules-running
       (hsr-navigation
        suturo-pm
        giskard::giskard-pm
        common-desig:wait-pm
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
       (su-demos::call-nav-action-ps argument)))));;change package in the future

(cpm:def-process-module suturo-pm (action-designator)
  (destructuring-bind (command argument-1 &rest rest-args)
      (desig:reference action-designator)
    (ecase command
      (su-real:pick-up
        :goal-pose-left argument-1
        :goal-pose-right (first rest-args)
        :collision-mode (second rest-args)
        :collision-object-b (third rest-args)
        :collision-object-b-link (fourth rest-args)
        :collision-object-a (fifth rest-args)
        :move-base (sixth rest-args)
        :prefer-base (seventh rest-args)
        :straight-line (tenth rest-args)
        :align-planes-left (eighth rest-args)
        :align-planes-right (ninth rest-args)
        :precise-tracking (nth 10 rest-args); that's eleventh element
        :object-pose (nth 11 rest-args)
        :object-size (nth 12 rest-args)
        ))))

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
        (desig:desig-prop ?motion-designator (:type :reaching))
        (desig:desig-prop ?motion-designator (:type :lifting))
        (desig:desig-prop ?motion-designator (:type :retracting))
        (desig:desig-prop ?motion-designator (:type :prepare-placing))
        (desig:desig-prop ?motion-designator (:type :placing))
        (desig:desig-prop ?motion-designator (:type :moving-gripper))
        (desig:desig-prop ?motion-designator (:type :moving-gripper-joint))
        (desig:desig-prop ?motion-designator (:type :moving-arm-joints))
        (desig:desig-prop ?motion-designator (:type :pulling))
        (desig:desig-prop ?motion-designator (:type :pushing))
        (desig:desig-prop ?motion-designator (:type :picking-up))
       ;; (desig:desig-prop ?motion-designator (:type :going))
        (desig:desig-prop ?motion-designator (:type :moving-torso))
        (desig:desig-prop ?motion-designator (:type :moving-custom))
        (desig:desig-prop ?motion-designator (:type :looking))
        (desig:desig-prop ?motion-designator (:type :closing-gripper))
        (desig:desig-prop ?motion-designator (:type :opening-gripper))))

  (prolog:<- (cpm:available-process-module giskard:giskard-pm)
    (prolog:not (cpm:projection-running ?_))))




