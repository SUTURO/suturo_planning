(in-package :plc)

;; TODO still in progress
(cram-process-modules:def-process-module hsr-navigation (motion-designator)
  ;;;;;;;;;;;;;;;;;;;; BASE ;;;;;;;;;;;;;;;;;;;;;;;;
  ;; (roslisp:ros-info (hsr-navigation-process-modules)
  ;;                   "hsr-navigation called with motion designator `~a'."
  ;;                   motion-designator)
  (destructuring-bind (command target) (desig:reference motion-designator)
    (ecase command
      ;;TODO differentiate types
      (going
       ;;(format t "COMMAND: ~a POSE: ~a" command (desig:reference target))
       (lli::call-nav-action-ps (desig:reference target))
       )))) ;;??? maybe add (desig:reference ..)

  ;;;;;;;;;;;;;;;;;;;; BODY ;;;;;;;;;;;;;;;;;;;;;;;;
(cram-process-modules:def-process-module hsr-motion (motion-designator)
  ;; (roslisp:ros-info (hsr-motion-process-modules)
  ;;                   "hsr-motion called with motion designator `~a'."
  ;;                   motion-designator)
  (destructuring-bind (command pos) (desig:reference motion-designator)
    (if (typep pos 'sequence)
        (lli:call-move-head-action pos)
        (ecase pos
          (:front
           (lli:call-move-head-action (vector 0.0 0.0)))
          (:perceive
           (lli::call-move-head-action (vector 0.0 -0.2)))
          (:perceive-down
           (lli:call-move-head-action (vector 0.0 -0.4)))
          (:safe
           (lli:call-move-head-action (vector 0.0 0.1)))
          (:left
           (lli:call-move-head-action (vector 1.5 0.1)))
          (:left-down
           (lli:call-move-head-action (vector 1.5 -0.3)))
          (:left-down-2
           (lli:call-move-head-action (vector 1.5 -0.4)))
          (:left-down-3
           (lli:call-move-head-action (vector 1.5 -0.7)))
          (:right
           (lli:call-move-head-action (vector -1.5 0.1)))
          (:right-down
           (lli:call-move-head-action (vector -1.5 -0.3)))
          (:right-down-2
           (lli:call-move-head-action (vector -1.5 -0.4)))
          (:right-down-3
           (lli:call-move-head-action (vector -1.5 -0.7)))))))

  ;;;;;;;;;;;;;;;;;;;; TORSO ;;;;;;;;;;;;;;;;;;;;;;;;
(cram-process-modules:def-process-module hsr-torso (motion-designator)
  ;; (roslisp:ros-info (hsr-torso-process-modules)
  ;;                   "hsr-torso called with motion designator `~a'."
  ;;                   motion-designator)
  (destructuring-bind (command ?height) (desig:reference motion-designator)
    (lli:call-giskard-joints-move-action (vector ?height) (vector 0.0))))

  ;;;;;;;;;;;;;;;;;;;; SAY ;;;;;;;;;;;;;;;;;;;;;;;;
(cram-process-modules:def-process-module hsr-say (motion-designator)
  (roslisp:ros-info (hsr-say-process-modules)
                    "hsr-say-action called with motion designator `~a'."
                    motion-designator)
  (destructuring-bind (command text) (desig:reference motion-designator)
    ;(format t "command: ~a  text: ~a"command text)
    (ecase command
      (say
       (lli:call-text-to-speech-action text)))))

  ;;;;;;;;;;;;;;;;;;;; PERCEIVE ;;;;;;;;;;;;;;;;;;;;;;;;
(cram-process-modules:def-process-module hsr-perception (motion-designator)
  ;; (roslisp:ros-info (hsr-perception-process-modules)
  ;;                   "hsr-say-action called with motion designator `~a'."
  ;;                   motion-designator)
  (destructuring-bind (command list) (desig:reference motion-designator)
    ;(format t "command: ~a  text: ~a"command text)
    (ecase command
      (perceive
       (lli:call-robosherlock-pipeline list)))))

  ;;;;;;;;;;;;;;;;;;;; ARM ;;;;;;;;;;;;;;;;;;;;;;;;
(cram-process-modules:def-process-module hsr-arm-motion (motion-designator)
  ;; (roslisp:ros-info (hsr-arm-motion-process-modules)
  ;;                   "hsr-arm-motion called with motion designator `~a'."
  ;;                   motion-designator)
  (destructuring-bind (command
                       ?pose
                       ?weight
                       ?width
                       ?height
                       ?depth
                       ?modus)
      (desig:reference motion-designator)
    (ecase command
      (grasping
       (lli:call-giskard-joints-grasping-action
        ?pose
        (plc::map-T-odom ?pose) ;;?pose-odom
        ?weight
        ?width
        ?height
        "grip" ;;obj pose /text
        ?depth
        ?modus))
      
      (placing
       (lli:call-giskard-joints-grasping-action
        ?pose
        (plc::map-T-odom ?pose) ;;?pose-odom
        ?weight
        ?width
        ?height
        "place" ;;obj pose /text
        ?depth
        ?modus))
       
       (perceiving
        (lli:call-giskard-joints-grasping-action
         ?pose
         ?pose ;;?pose-odom
         ?weight
         ?width
         ?height
         "perceive" ;;obj pose /text
         ?depth
         ?modus))

      (perceiving-high
       (lli:call-giskard-joints-grasping-action
        ?pose
        ?pose ;;?pose-odom
        ?weight
        ?width
        ?height
        "perceive_up" ;;obj pose /text
        ?depth
        ?modus))

      (perceiving-side
       (lli:call-giskard-joints-grasping-action
        ?pose
        ?pose ;;?pose-odom
        ?weight
        ?width
        ?height
        "perceive_side" ;;obj pose /text
        ?depth
        ?modus)))))


  ;;;;;;;;;;;;;;;;;;;; TESTS FOR DEBUGGING ;;;;;;;;;;;;;;;;;;;;;;;;
(defun test-head-motion ()
  (cram-language:top-level
    (cram-process-modules:with-process-modules-running (hsr-motion)
      (let ((look (desig:a motion
                              (:type :looking)
                              (:positions :front))))
        (cram-process-modules:pm-execute 'hsr-motion look)))))

(defun test-head-motion2 (?direction)
  (cram-language:top-level
    (cram-process-modules:with-process-modules-running (hsr-motion)
      (let ((look-at (desig:a motion
                              (:type :looking)
                              (:direction ?direction))))
        (cram-process-modules:pm-execute 'hsr-motion look-at)))))
                          
(defun test-move-base-motion (?pose)
  (cram-language:top-level
    (cram-process-modules:with-process-modules-running (hsr-navigation)
      (let ((going (desig:a motion
                              (:type :going)
                              (:target ?pose))))
        (cram-process-modules:pm-execute 'hsr-navigation going)))))


(defun test-say (?text)
  (cpl:top-level
    (cram-process-modules:with-process-modules-running (hsr-say)
      (let ((say          
              (desig:a motion
                       (:type :say)
                       (:text ?text))))
        (cram-process-modules:pm-execute 'hsr-say say)))))

(defun test-navigation-desig (?pose)
  (cpl:top-level
    (cram-process-modules:with-process-modules-running (hsr-navigation)
      (let ((going 
              (desig:a motion
                       (:type :going)
                       (:target (desig:a location
                                         (:pose ?pose))))))
        (cpl:seq
          (cram-process-modules:pm-execute 'hsr-navigation going))))))

(defmacro with-hsr-process-modules (&body body)
  `(cram-process-modules:with-process-modules-running
       (plc::hsr-navigation
        plc::hsr-motion
        plc::hsr-say
        plc::hsr-arm-motion
        plc::hsr-torso
        plc::hsr-perception)
     (cpl-impl::named-top-level (:name :top-level)
       ,@body)))

;; (defmacro with-hsr-proj-process-modules (&body body)
;;   `(cram-process-modules:with-process-modules-running
;;        (hsrb-proj::hsrb-proj-navigation
;;         hsrb-proj::hsrb-proj-torso
;;         hsrb-proj::hsrb-proj-ptu
;;         hsrb-proj::hsrb-proj-perception
;;         hsrb-proj::hsrb-proj-grippers)
;;      (cpl-impl::named-top-level (:name :top-level)
;;      ,@body)))


