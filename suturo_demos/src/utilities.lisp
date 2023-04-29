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

;; (with-knowledge-result (a b) '(= (list 1 2) (list a b))
;;             (print a)
;;             (print b))
;;@author Tede von Knorre, Felix Krause
(defmacro with-knowledge-result (vars query &body body)
  `(let ((raw-response (simple-knowledge ,query (find-package :common-lisp-user))))
     (if (eq raw-response 1)
         (error "Knowledge query failed at the macro with-knowledge-result")
         (unwind-protect
              (let ,(loop for x in vars
                          collect `(,x (fix-prolog-string (cdr (assoc ',(match-prolog-symbol x (find-package :common-lisp-user)) (car raw-response))))))
                ,@body
                )
           (json-prolog:finish-query raw-response)))))



;;@author Tede von Knorre, Felix Krause
(defun simple-knowledge (query &optional (package *package*))
  (print package)
  (with-safe-prolog
    (json-prolog:prolog query :package package)))

;;@author Felix Krause
(defun make-pose-stamped-from-knowledge-result (result)
  (cl-tf:make-pose-stamped
   (first result) 0.0
   (cl-tf:make-3d-vector
    (first (second result)) (second (second result)) (third (second result)))
   (cl-tf:make-quaternion (first (third result)) (second (third result)) (third (third result)) (fourth (third result))))
  )


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

(defmethod exe-perform-type ((action-type (eql :opening)) &key object (distance 1.7))
    (cond
      ((search "right" object)  (let ((?object-pose (knowledge-get-right-handle-pose)))
                                  (exe:perform
                                   (desig:an action
                                             (type looking)
                                             (target (desig:a location (pose ?object-pose)))))
                                  (sleep 1)
                                  (let ((dist (* distance 1)))
                                    (btr:set-robot-state-from-joints
                                     `((,object ,dist))
                                     (btr:get-environment-object)))))
       
      ((search "left" object)  (let ((?object-pose (knowledge-get-left-handle-pose)))
                                 (exe:perform
                                  (desig:an action
                                            (type looking)
                                            (target (desig:a location (pose ?object-pose)))))
                                 (sleep 1)
                                 (let ((dist (* distance -1)))
                                   (btr:set-robot-state-from-joints
                                    `((,object ,dist))
                                    (btr:get-environment-object)))))
      
      ((search "origin" object)  (let ((?object-pose (knowledge-get-entrance-handle-pose)))
                                  (exe:perform
                                   (desig:an action
                                             (type looking)
                                             (target (desig:a location (pose ?object-pose)))))
                                   (sleep 1)
                                   (let ((dist (if (> distance 0)
                                                    0
                                                    distance)))
                                     (btr:set-robot-state-from-joints
                                      `((,object ,dist))
                                      (btr:get-environment-object)))))
      (t (error "Please enter a valid openable object"))))

(defmethod exe-perform-type ((action-type (eql :closing)) &key object)
  (cond
      ((search "right" object)  (let ((?object-pose (knowledge-get-right-handle-pose)))
                                  (exe:perform
                                   (desig:an action
                                             (type looking)
                                             (target (desig:a location (pose ?object-pose)))))
                                  (sleep 1)
                                  (btr:set-robot-state-from-joints
                                   `((,object 0))
                                   (btr:get-environment-object))))
       
      ((search "left" object)  (let ((?object-pose (knowledge-get-left-handle-pose)))
                                 (exe:perform
                                  (desig:an action
                                            (type looking)
                                            (target (desig:a location (pose ?object-pose)))))
                                 (sleep 1)
                                 (btr:set-robot-state-from-joints
                                  `((,object 0))
                                  (btr:get-environment-object))))
      
      ((search "origin" object)  (let ((?object-pose (knowledge-get-entrance-handle-pose)))
                                   (exe:perform
                                    (desig:an action
                                              (type looking)
                                              (target (desig:a location (pose ?object-pose)))))
                                   (sleep 1)
                                     (btr:set-robot-state-from-joints
                                      `((,object -1.4))
                                      (btr:get-environment-object))))
      (t (error "Please enter a valid openable object"))))

(defmethod exe-perform-type ((action-type (eql :reaching)) &key (arm :left) ?object)
        (exe:perform
         (desig:an action
                   (type reaching)
                   (object ?object)
                   )))

;;;;;;;;;;;;;;
;; BTR-Utility

;; use to quickly test navigation poses 
(defun move-hsr-debug (?pose)
  (urdf-proj:with-simulated-robot
    (exe:perform
     (desig:an action
               (type going)
               (target (desig:a location (pose ?pose)))))))

;; example for how to spawn objects
(defun spawn-pringles()
  (btr:add-object btr:*current-bullet-world* :cylinder-item 'cylinder-1
                    '((1.5 -1.5 0.78) (0 0 0 1))
                    :mass 0.2 :size (cl-transforms:make-3d-vector 0.03 0.03 0.08)
                    :item-type :pringles))

(defun move-obj (obj-name pose)
  (btr-utils:move-object
   obj-name
   pose))

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

;;;;;;;;;;;;
;; Knowledge

;; if you are using a different map, you might need to adjust the strings to reference the proper link

(defun knowledge-get-left-handle-pose()
  (cl-tf:pose->pose-stamped
   cram-tf:*fixed-frame*
   0 
   (btr:link-pose (btr:get-environment-object) "shelf:shelf:shelf_door_left:shelf_link_handle")))

(defun knowledge-get-right-handle-pose()
  (cl-tf:pose->pose-stamped
   cram-tf:*fixed-frame*
   0 
   (btr:link-pose (btr:get-environment-object) "shelf:shelf:shelf_door_right:shelf_link_handle")))

(defun knowledge-get-entrance-handle-pose()
  (cl-tf:pose->pose-stamped
   cram-tf:*fixed-frame*
   0 
   (btr:link-pose (btr:get-environment-object) "iai_kitchen:living_room:door_handle_inside")))


;; those are the joints that can be assigned different values to open/close objects, for example doors
(defun knowledge-get-shelf-left()
  "shelf:shelf:shelf_door_left:shelf_joint")

(defun knowledge-get-shelf-right()
  "shelf:shelf:shelf_door_right:shelf_joint")

(defun knowledge-get-entrance()
  "iai_kitchen:living_room:door_origin_revolute_joint")

(defun knowledge-get-looking-pose(obj-name)
  (cl-tf:pose->pose-stamped
                   cram-tf:*fixed-frame*
                   0 
                   (btr:object-pose obj-name)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun round-for-knowrob (v &optional (n 10))
  (declare (type float v)
           (type (integer 0) n))
  (setf v (coerce v 'single-float))
  (let ((10^-n (expt 10 (- n))))
     (* (fround v 10^-n)
               10^-n)))

