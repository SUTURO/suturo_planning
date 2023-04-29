(in-package :su-real)

;; @author Luca Krohm
(def-fact-group suturo-action (desig:action-grounding)
  ;; @author Luca Krohm
  (<- (action-grounding ?designator (su-real:pick-up ?resolved-action-designator))
    (spec:property ?designator (:type :picking-up))
    (once (or (desig:desig-prop ?designator (:collision-mode ?collision-mode))
              (equal ?collision-mode nil)))
    (once (or (desig:desig-prop ?designator (:collision-object-b ?collision-object-b))
              (equal ?collision-object-b nil)))
    (once (or (desig:desig-prop ?designator (:collision-object-b-link
                                             ?collision-object-b-link))
              (equal ?collision-object-b-link nil)))
    (once (or (desig:desig-prop ?designator (:collision-object-a ?collision-object-a))
              (equal ?collision-object-a nil)))
    (once (or (desig:desig-prop ?designator (:move-base ?move-base))
              (equal ?move-base nil)))
    (once (or (desig:desig-prop ?designator (:prefer-base ?prefer-base))
              (equal ?prefer-base nil)))
    (once (or (desig:desig-prop ?designator (:straight-line ?straight-line))
              (equal ?straight-line nil)))
    (once (or (desig:desig-prop ?designator (:align-planes-left ?align-planes-left))
              (equal ?align-planes-left nil)))
    (once (or (desig:desig-prop ?designator (:align-planes-right ?align-planes-right))
              (equal ?align-planes-right nil)))
    (once (or (desig:desig-prop ?designator (:precise-tracking ?precise-tracking))
              (equal ?precise-tracking nil)))
    (once (or (desig:desig-prop ?designator (:object-type ?object-type))
              (equal ?object-type nil)))
    (once (or (desig:desig-prop ?designator (:object-pose ?object-pose))
              (equal ?object-pose nil)))
    (once (or (desig:desig-prop ?designator (:object-size ?object-size))
              (equal ?object-size nil)))

    (desig:designator :action ((:type :picking-up)
                               (:collision-mode ?collision-mode)
                               (:collision-object-b ?collision-object-b)
                               (:collision-object-b-link ?collision-object-b-link)
                               (:collision-object-a ?collision-object-a)
                               (:move-base ?move-base)
                               (:prefer-base ?prefer-base)
                               (:align-planes-left ?align-planes-left)
                               (:align-planes-right ?align-planes-right)
                               (:straight-line ?straight-line)
                               (:precise-tracking ?precise-tracking)
                               (:object-type ?object-type)
                               (:object-pose ?object-pose)
                               (:object-size ?object-size))
                      ?resolved-action-designator))

  ;; @author Luca Krohm
  (<- (action-grounding ?designator (su-real:place ?resolved-action-designator))
    (spec:property ?designator (:type :placing))
    (once (or (desig:desig-prop ?designator (:collision-mode ?collision-mode))
              (equal ?collision-mode nil)))
    (once (or (desig:desig-prop ?designator (:collision-object-b ?collision-object-b))
              (equal ?collision-object-b nil)))
    (once (or (desig:desig-prop ?designator (:collision-object-b-link
                                             ?collision-object-b-link))
              (equal ?collision-object-b-link nil)))
    (once (or (desig:desig-prop ?designator (:collision-object-a ?collision-object-a))
              (equal ?collision-object-a nil)))
    (once (or (desig:desig-prop ?designator (:move-base ?move-base))
              (equal ?move-base nil)))
    (once (or (desig:desig-prop ?designator (:prefer-base ?prefer-base))
              (equal ?prefer-base nil)))
    (once (or (desig:desig-prop ?designator (:straight-line ?straight-line))
              (equal ?straight-line nil)))
    (once (or (desig:desig-prop ?designator (:align-planes-left ?align-planes-left))
              (equal ?align-planes-left nil)))
    (once (or (desig:desig-prop ?designator (:align-planes-right ?align-planes-right))
              (equal ?align-planes-right nil)))
    (once (or (desig:desig-prop ?designator (:precise-tracking ?precise-tracking))
              (equal ?precise-tracking nil)))
    (once (or (desig:desig-prop ?designator (:target-pose ?target-pose))
              (equal ?target-pose nil)))
    (once (or (desig:desig-prop ?designator (:object-height ?object-height))
              (equal ?object-height nil)))

    (desig:designator :action ((:type :placing)
                               (:collision-mode ?collision-mode)
                               (:collision-object-b ?collision-object-b)
                               (:collision-object-b-link ?collision-object-b-link)
                               (:collision-object-a ?collision-object-a)
                               (:move-base ?move-base)
                               (:prefer-base ?prefer-base)
                               (:align-planes-left ?align-planes-left)
                               (:align-planes-right ?align-planes-right)
                               (:straight-line ?straight-line)
                               (:precise-tracking ?precise-tracking)
                               (:target-pose ?target-pose)
                               (:object-height ?object-height))
                      ?resolved-action-designator))

   ;; @author Luca Krohm
  (<- (action-grounding ?designator (su-real:open-door ?resolved-action-designator))
    (spec:property ?designator (:type :opening-door))
    (once (or (desig:desig-prop ?designator (:collision-mode ?collision-mode))
              (equal ?collision-mode nil)))
    (once (or (desig:desig-prop ?designator (:collision-object-b ?collision-object-b))
              (equal ?collision-object-b nil)))
    (once (or (desig:desig-prop ?designator (:collision-object-b-link
                                             ?collision-object-b-link))
              (equal ?collision-object-b-link nil)))
    (once (or (desig:desig-prop ?designator (:collision-object-a ?collision-object-a))
              (equal ?collision-object-a nil)))
    (once (or (desig:desig-prop ?designator (:move-base ?move-base))
              (equal ?move-base nil)))
    (once (or (desig:desig-prop ?designator (:prefer-base ?prefer-base))
              (equal ?prefer-base nil)))
    (once (or (desig:desig-prop ?designator (:straight-line ?straight-line))
              (equal ?straight-line nil)))
    (once (or (desig:desig-prop ?designator (:align-planes-left ?align-planes-left))
              (equal ?align-planes-left nil)))
    (once (or (desig:desig-prop ?designator (:align-planes-right ?align-planes-right))
              (equal ?align-planes-right nil)))
    (once (or (desig:desig-prop ?designator (:precise-tracking ?precise-tracking))
              (equal ?precise-tracking nil)))
    (once (or (desig:desig-prop ?designator (:handle-link ?handle-link))
              (equal ?handle-link nil)))
    (once (or (desig:desig-prop ?designator (:joint-angle ?joint-angle))
              (equal ?joint-angle nil)))

    (desig:designator :action ((:type :opening-door)
                               (:collision-mode ?collision-mode)
                               (:collision-object-b ?collision-object-b)
                               (:collision-object-b-link ?collision-object-b-link)
                               (:collision-object-a ?collision-object-a)
                               (:move-base ?move-base)
                               (:prefer-base ?prefer-base)
                               (:align-planes-left ?align-planes-left)
                               (:align-planes-right ?align-planes-right)
                               (:straight-line ?straight-line)
                               (:precise-tracking ?precise-tracking)
                               (:handle-link ?handle-link)
                               (:joint-angle ?joint-angle))
                      ?resolved-action-designator))
    
  (<- (desig:action-grounding ?action-designator (su-real::pour
                                                  ?resolved-action-designator))
    (spec:property ?action-designator (:type :pouring-without-retries))
    ;; ;; source
    ;; (-> (spec:property ?action-designator (:arm ?arm))
    ;;     (-> (spec:property ?action-designator (:object
    ;;                                            ?source-designator))
    ;;         (once (or (cpoe:object-in-hand ?source-designator ?arm ?grasp)
    ;;                   (format "WARNING: Wanted to pour from object ~a ~
    ;;                            with arm ~a, but it's not in the arm.~%"
    ;;                           ?source-designator ?arm)))
    ;;         (cpoe:object-in-hand ?source-designator ?arm ?grasp))
    ;;     (-> (spec:property ?action-designator (:object
    ;;                                            ?source-designator))
    ;;         (once (or (cpoe:object-in-hand ?source-designator ?arm ?grasp)
    ;;                   (format "WARNING: Wanted to pour from object ~a ~
    ;;                            but it's not in any of the hands.~%"
    ;;                           ?source-designator)))
    ;;         (cpoe:object-in-hand ?source-designator ?arm ?grasp)))
    ;; (desig:current-designator ?source-designator ?current-source-designator)
    ;;destination / target
    (spec:property ?action-designator (:on-object ?target-designator))
    (desig:current-designator ?target-designator ?current-target-designator)
    ;; angle
    (-> (spec:property ?action-designator (:tilt-angle ?tilt-angle))
        (true)
        (lisp-fun man-int:get-tilt-angle-for-pouring ?source-type ?target-type
                  ?tilt-angle))
    ;; cartesian pouring trajectory
    (equal ?objects-acted-on (;; ?current-source-designator
                              ?current-target-designator))
    (-> (equal ?arm :left)
        (and (lisp-fun man-int:get-action-trajectory :pouring
                       ?arm ?side nil ?objects-acted-on
                       :tilt-angle ?tilt-angle :side ?side
                       ?left-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :reaching
                       ?left-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :tilting-down
                       ?left-tilt-down-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :tilting
                       ?left-tilt-up-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :tilting-second
                       ?left-tilt-second-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :tilting-third
                       ?left-tilt-third-poses)
             ;; (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :retracting
             ;;           ?left-retract-poses)
             )
        
        (and (equal ?left-reach-poses NIL)
             (equal ?left-tilt-down-poses NIL)
             (equal ?left-tilt-up-poses NIL)
             (equal ?left-tilt-second-poses NIL)
             (equal ?left-tilt-third-poses NIL)
             (equal ?left-retract-poses NIL)))

    
    ;;put together resulting designator
    (desig:designator :action ((:type :pouring)
                               ;;(:object ?current-source-designator)
                               (:arm ?arm)
                               (:configuration ?side)
                               ;;(:grasp ?grasp)
                               (:on-object ?current-target-designator)
                               ;; ;; (:other-object-is-a-robot ?other-object-is-a-robot)
                               ;;(:look-pose ?look-pose)
                               ;(:robot-arm-is-also-a-neck ?robot-arm-is-also-a-neck)
                               ;;(:wait-duration ?wait-duration)
                               
                               (:left-reach-poses ?left-reach-poses)
                               (:left-tilt-down-poses ?left-tilt-down-poses)
                               (:left-tilt-up-poses ?left-tilt-up-poses)
                               (:left-tilt-second-poses ?left-tilt-second-poses)
                               (:left-tilt-third-poses ?left-tilt-third-poses)
                               )
                      
                      ?resolved-action-designator)))
  
                                                    
  
(def-fact-group suturo-motion (desig:motion-grounding)
  (<- (motion-grounding ?designator (?open-or-close ?effort))
    (spec:property ?designator (:type :gripper-motion))
    (or (and (spec:property ?designator (:open-close :open))
             (equal ?open-or-close su-real:open-gripper))
        (and (spec:property ?designator (:open-close :close))
             (equal ?open-or-close su-real:close-gripper)))
    (once (or (desig:desig-prop ?designator (:effort ?effort))
              (equal ?effort 0)))))
