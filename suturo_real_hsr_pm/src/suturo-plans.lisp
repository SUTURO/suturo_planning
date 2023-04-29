(in-package :su-real)

;; @author Luca Krohm
;; @TODO failurehandling
(defun pick-up (&key
                  ((:collision-mode ?collision-mode))
                  ((:collision-object-b ?collision-object-b))
                  ((:collision-object-b-link ?collision-object-b-link))
                  ((:collision-object-a ?collision-object-a))
                  ((:move-base ?move-base))
                  ((:prefer-base ?prefer-base))
                  ((:straight-line ?straight-line))
                  ((:align-planes-left ?align-planes-left))
                  ((:align-planes-right ?align-planes-right))
                  ((:precise-tracking ?precise-tracking))
                  ((:object-type ?object-type))
                  ((:object-pose ?object-pose))
                  ((:object-size ?object-size))
                &allow-other-keys)
  "Receives parameters from action-designator, and then executes the corresponding motions"
  (declare (type boolean ?move-base ?prefer-base ?straight-line ?precise-tracking
                 ?align-planes-left ?align-planes-right))

  (cpl:with-retry-counters ((manip-retries 1))
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (suturo-pickup grasp-object)
                             "Some manipulation failure happened: ~a"
                             e)
           (cpl:do-retry manip-retries
             (roslisp:ros-warn (suturo-pickup grasp-object) "Retrying...")
             (su-demos:park-robot)
             (let* ((?object-type :muesli)
                    (?source-object-desig
                      (desig:an object
                                (type ?object-type)))
                    ;; detect object and safe the return value
                    (?object-desig
                      (exe:perform (desig:an action
                                             (type detecting)
                                             (object ?source-object-desig)))))
               (roslisp:with-fields 
                   ((?pose
                     (cram-designators::pose cram-designators:data))) 
                   ?object-desig
                 (setf ?object-pose ?pose)))
             (cpl:retry))))
      
      (roslisp:with-fields 
          ((?object-height (z)))
          ?object-size
        
        (exe:perform (desig:a motion
                        (type aligning-height)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking)
                        (goal-pose ?object-pose)
                        (object-height ?object-height))))
      
      (exe:perform (desig:a motion
                            (type :opening-gripper)))
      
      (exe:perform (desig:a motion
                            (type reaching)
                            (collision-mode ?collision-mode)
                            (object-pose ?object-pose)
                            (object-size ?object-size)))
      
      (exe:perform (desig:a motion
                            (type :closing-gripper)))))
      
  (exe:perform (desig:a motion
                        (type :lifting)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking)
                        (object-pose ?object-pose)))

  (exe:perform (desig:a motion
                        (type :retracting)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking))))

;; @author Luca Krohm
;; @TODO failurehandling
(defun place (&key
                  ((:collision-mode ?collision-mode))
                  ((:collision-object-b ?collision-object-b))
                  ((:collision-object-b-link ?collision-object-b-link))
                  ((:collision-object-a ?collision-object-a))
                  ((:move-base ?move-base))
                  ((:prefer-base ?prefer-base))
                  ((:straight-line ?straight-line))
                  ((:align-planes-left ?align-planes-left))
                  ((:align-planes-right ?align-planes-right))
                  ((:precise-tracking ?precise-tracking))
                  ((:target-pose ?target-pose))
                  ((:object-height ?object-height))
              &allow-other-keys)
  "Receives parameters from action-designator, and then executes the corresponding motions"
  (declare (type boolean ?move-base ?prefer-base ?straight-line ?precise-tracking
                 ?align-planes-left ?align-planes-right))

  (exe:perform (desig:a motion
                        (type aligning-height)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking)
                        (goal-pose ?target-pose)
                        (object-height ?object-height)))

  (exe:perform (desig:a motion
                        (type placing)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking)
                        (target-pose ?target-pose)
                        (object-height ?object-height)))

  (exe:perform (desig:a motion
                        (type :opening-gripper)))
  
  (exe:perform (desig:a motion
                        (type :retracting)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking))))


;; @author Luca Krohm
;; @TODO failurehandling
(defun open-door (&key
                    ((:collision-mode ?collision-mode))
                    ((:collision-object-b ?collision-object-b))
                    ((:collision-object-b-link ?collision-object-b-link))
                    ((:collision-object-a ?collision-object-a))
                    ((:move-base ?move-base))
                    ((:prefer-base ?prefer-base))
                    ((:straight-line ?straight-line))
                    ((:align-planes-left ?align-planes-left))
                    ((:align-planes-right ?align-planes-right))
                    ((:precise-tracking ?precise-tracking))
                    ((:handle-link ?handle-link))
                    ((:joint-angle ?joint-angle))
              &allow-other-keys)
  "Receives parameters from action-designator, and then executes the corresponding motions"
  (declare (type boolean ?move-base ?prefer-base ?straight-line ?precise-tracking
                 ?align-planes-left ?align-planes-right))

  (exe:perform (desig:a motion
                        (type gripper-motion)
                        (:open-close :open)
                        (effort 0.1)))
    
  (exe:perform (desig:a motion
                        (type reaching)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking)
                        (object-name ?handle-link)))
    
  (exe:perform (desig:a motion
                        (type gripper-motion)
                        (:open-close :close)
                        (effort 0.1)))

  (exe:perform (desig:a motion
                        (type pulling)
                        (arm :left)
                        (collision-object-b-link ?handle-link)
                        (joint-angle ?joint-angle)))

  (exe:perform (desig:a motion
                        (type gripper-motion)
                        (:open-close :open)
                        (effort 0.1)))
    
  (exe:perform (desig:a motion
                        (type :retracting)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking)
                        (tip-link t))))

;; @author Luca Krohm
(defun open-gripper (&key
                     ((:effort ?effort))
                     &allow-other-keys)
  (call-gripper-action (abs ?effort)))

;; @author Luca Krohm
(defun close-gripper (&key
                     ((:effort ?effort))
                     &allow-other-keys)
  (call-gripper-action (* -1 (abs ?effort))))

;; (defun pour (&key
;;                ((:arm ?arm))
;;                ((:side ?side))
;;                ;;grasp
;;                ((:left-reach-poses ?left-reach-poses))
;;                ((:left-tilt-down-poses ?left-tilt-down-poses))
;;                ((:left-tilt-up-poses ?left-tilt-up-poses))
;;                ((:left-tilt-second-poses ?left-tilt-second-poses))
;;                ((:left-tilt-third-poses ?left-tilt-third-poses))
               
;;                ((:on-object ?on-object))
;;                ;;object
;;                ((:wait-duration ?wait-duration))
;;                ((:look-location ?look-location))
;;                robot-arm-is-also-a-neck
;;              &allow-other-keys)
  
;;   ;; (declare (type (or null list)
;;   ;;                ?left-reach-poses ?right-reach-poses
;;   ;;                ?left-tilt-down-poses ?right-tilt-down-poses
;;   ;;                ?left-tilt-up-poses ?right-tilt-up-poses
;;   ;;                ?left-retract-poses ?right-retract-poses)
;;   ;;          (type desig:object-designator ?on-object object)
;;   ;;          (type desig:location-designator ?look-location)
;;   ;;          ;;(type keyword ?arm side grasp)
;;   ;;          (type number ?wait-duration)
;;   ;;          (ignore side grasp object))
;;   (let* ((sleepy nil)
;;          (?movy nil)
;;          (?align-planes-left nil)
;;          (?align-planes-right nil)
;;          (?move-base-when-reaching t))

;;     (cpl:with-failure-handling
;;         (((or common-fail:manipulation-low-level-failure
;;               common-fail:manipulation-goal-not-reached) (e)
;;            (roslisp:ros-warn (pp-plans pour-reach)
;;                              "Manipulation messed up: ~a~%Ignoring."
;;                              e)
;;            (return)))
;;       (let ((?goal `(cpoe:tool-frames-at ,?left-reach-poses ,?right-reach-poses)))
;;         (exe:perform
;;          (desig:an action
;;                    (type reaching)
;;                    (object ?on-object)
;;                    (left-poses ?left-reach-poses)
;;                    (right-poses ?right-reach-poses)
;;                    (move-base ?move-base-when-reaching)
;;                    (goal ?goal)))))

;;     (setf ?move-base-when-reaching nil)
 
;;     (cpl:with-retry-counters ((giskardside-retries 3))
;;       (cpl:with-failure-handling
;;           (((or common-fail:manipulation-low-level-failure
;;                 common-fail:manipulation-goal-not-reached) (e)
;;              (roslisp:ros-warn (pp-plans pour-reach)
;;                                "Manipulation messed up: ~a~%Failing."
;;                                e)
             
;;              (cpl:do-retry giskardside-retries
;;                (break)
;;                (cpl:retry))
;;              (return)))
;;         (let ((?goal `(cpoe:tool-frames-at ,?left-reach-poses ,?right-reach-poses)))
;;           (exe:perform
;;            (desig:an action
;;                      (type reaching)
;;                      (object ?on-object)
;;                      (left-poses ?left-reach-poses)
;;                      (right-poses ?right-reach-poses)
;;                      (move-base ?move-base-when-reaching)
;;                      (goal ?goal))))))

    
   
    
;;     (when sleepy
;;       (sleep 2))

;;     (cpl:with-retry-counters ((giskardside-retries 3))
;;       (cpl:with-failure-handling
;;           (((or common-fail:manipulation-low-level-failure
;;                 common-fail:manipulation-goal-not-reached) (e)
;;              (roslisp:ros-warn (pp-plans pour-tilt-down-more)
;;                                "Manipulation messed up: ~a~%Failing."
;;                                e)
             
;;              (cpl:do-retry giskardside-retries
;;                (cpl:retry))
;;              (return)))

        
;;         (let ((?goal `(cpoe:tool-frames-at ,?left-tilt-up-poses ,?right-tilt-up-poses)))
;;           (exe:perform
;;            (desig:an action
;;                      (type tilting)
;;                      (object ?on-object)
;;                      (left-poses ?left-tilt-up-poses)
;;                      (right-poses ?right-tilt-up-poses)
;;                      (align-planes-left ?align-planes-left)
;;                      (align-planes-right ?align-planes-right)
;;                      (move-base ?movy)
;;                      ;;(collision-mode :allow-attached)
;;                      (goal ?goal))))
;;         ))
    
    
;;     (when sleepy
;;       (sleep 5))


    
    
    
;;     (cpl:with-retry-counters ((giskardside-retries 3))
;;       (cpl:with-failure-handling
;;           (((or common-fail:manipulation-low-level-failure
;;                 common-fail:manipulation-goal-not-reached) (e)
;;              (roslisp:ros-warn (pp-plans pour-retract)
;;                                "Manipulation messed up: ~a~%Failing."
;;                                e)
             
;;              (cpl:do-retry giskardside-retries
;;                (cpl:retry))
;;              (return)))

        
;;         (let ((?goal `(cpoe:tool-frames-at ,?left-reach-poses ,?right-reach-poses)))
;;           (exe:perform
;;            (desig:an action
;;                      (type retracting)
;;                      (object ?on-object)
;;                      (left-poses ?left-reach-poses)
;;                      (right-poses ?right-reach-poses)
;;                      (application-context pouring)
;;                      (goal ?goal))))))))
