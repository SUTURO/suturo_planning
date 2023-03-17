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
                  ((:object-pose ?object-pose))
                  ((:object-size ?object-size))
                &allow-other-keys)
  "Receives parameters from action-designator, and then executes the corresponding motions"
  (declare (type boolean ?move-base ?prefer-base ?straight-line ?precise-tracking
                 ?align-planes-left ?align-planes-right))

  (exe:perform (desig:a motion
                        (type :opening-gripper)))
      
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
                        (object-pose ?object-pose)
                        (object-size ?object-size)))
    
  (exe:perform (desig:a motion
                        (type :closing-gripper)))
    
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
                        (precise-tracking ?precise-tracking)
                        (object-pose ?object-pose))))

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
                        (type prepare-placing)
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
                          (type :opening-gripper))))
