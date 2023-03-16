(in-package :su-real)

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
  (declare (type boolean ?move-base ?prefer-base ?straight-line ?precise-tracking
                 ?align-planes-left ?align-planes-right))

  (exe:perform (desig:a motion
                        (type :opening-gripper)))
      
  ;;Moves the gripper to the cereal box, implicitly opens the gripper beforehand.
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
  (declare (type boolean ?move-base ?prefer-base ?straight-line ?precise-tracking
                 ?align-planes-left ?align-planes-right))

  ;; replace place MOTION with "torso on level with targetpose" "placing motion"
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

  ;; replace place MOTION with "torso on level with targetpose" "placing motion"
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
