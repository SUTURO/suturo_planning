(in-package :su-demos)

;;====================================================================================================
;;navigation

;;used in go-get-it
;;@author Torge Olliges    
(defun move-hsr (nav-goal-pose-stamped)
  "Receives pose `nav-goal-pose-stamped'. Lets the robot move to the given pose with a motion designator"
  (let* ((?successfull-pose (try-movement-stamped-list
                             (list nav-goal-pose-stamped))))
    (exe:perform 
     (desig:a motion
              (type going)
              (pose ?successfull-pose)))))

;;====================================================================================================
;;knowledge

;; TODO add list of possible querys
;; @author Luca Krohm
(defun call-knowledge (query &key param-list result)
  "Receives query `query', list of strings `param-list' and symbol `result' . Returns the query result for the chosen parameters. Used for querys that receive the 'result' as last parameter in prolog."
  (roslisp:ros-info (knowledge-object-client)
                    "Calling query ~a."
                    query)
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog
                          (alexandria:flatten (list query param-list result))))))
    ;; (roslisp:ros-info (knowledge-object-client)
    ;;                   "Goal pose for object ~a is ~a."
    ;;                   object-name
    ;;                   raw-response)
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-object-client)
                                             "Query ~a didn't reach any solution."
                                             query))
      (t (fix-prolog-string (cdr (assoc (match-prolog-symbol result) (cut:lazy-car raw-response))))))))

;; @author Luca Krohm
(defun call-knowledge2 (query &key param-list result)
  "Receives query `query', list of strings `param-list' and symbol `result' . Returns the query result for the chosen parameters. Used for querys that receive the 'result' as first parameter in prolog."
  (roslisp:ros-info (knowledge-object-client)
                    "Calling query ~a."
                    query)
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog
                          (alexandria:flatten (list query result param-list))))))
    (roslisp:ros-info (knowledge-object-client)
                      "Raw Response for object is ~a."
                      ;;object-name
                      raw-response)
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-object-client)
                                             "Query ~a didn't reach any solution."
                                             query))
      (t (fix-prolog-string (cdr (assoc (match-prolog-symbol result) (cut:lazy-car raw-response))))))))


(defun perceive-handle-closed ()
  "Test function that returns working pose for closed drawer handle for testing purposes. Pose was previously perceived by Perception."
   (cl-tf2::make-pose-stamped
    "map" 0
    (cl-tf2::make-3d-vector 0.18888034742219937d0 -0.30079838556398194d0 0.2854991327720375d0)
    (cl-tf2::make-quaternion 0.0 0.0 0.0 1.0)))

(defun perceive-handle-opened ()
   "Test function that returns working pose for opened drawer handle for testing purposes. Pose was calculated using a pose previously perceived by Perception."
   (cl-tf2::make-pose-stamped
    "map" 0
    (cl-tf2::make-3d-vector 0.18888034742219937d0 0.10079838556398194d0 0.2854991327720375d0)
    (cl-tf2::make-quaternion 0.0 0.0 0.0 1.0)))


;;;;;;;;;;;;;;;;;;
;; Fake-Designator

(defun check-arm-for-object ()
  (let ((link
        (cut:var-value
          '?link
          (car
           (prolog:prolog
            `(and (rob-int:robot ?rob)
                  (rob-int:end-effector-link ?rob :left ?link)))))))
    (car
     (btr:link-attached-object-names
     (btr:get-robot-object)
     link))))

(defgeneric exe-perform-type (action-type &key &allow-other-keys))

(defmethod exe-perform-type ((action-type (eql :picking-up)) &key (arm :left) object)
  (let*((gripper-pose
            (cl-tf::lookup-transform cram-tf:*transformer* "map" "/hand_gripper_tool_frame"))
          (obj-name (desig:desig-prop-value object :name)))
      (roslisp:with-fields (translation rotation)
          gripper-pose
        (btr-utils:move-object
         obj-name
         (cl-tf:make-pose-stamped
          "map" 0.0
          translation
          (cl-tf:make-quaternion 0 0 0 1))))
      (cram-occasions-events:on-event
       (make-instance 'cpoe:object-attached-robot
                      :arm arm
                      :object-name obj-name
                      :object-designator object
                      :grasp :front))))

(defmethod exe-perform-type ((action-type (eql :placing)) &key (arm :left) pose)
  (let*((obj-name (check-arm-for-object)))
      (btr-utils:move-object
         obj-name
         pose)
      (cram-occasions-events:on-event
       (make-instance 'cpoe:object-detached-robot
                      :arm (list arm)
                      :object-name obj-name))))
