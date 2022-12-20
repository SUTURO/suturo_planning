(in-package :su-demos)

;;====================================================================================================
;;navigation

;;used in go-get-it
;;@author Torge Olliges    
(defun move-hsr (nav-goal-pose-stamped)
  "Receives pose `nav-goal-pose-stamped'. Lets the robot move to the given pose with a motion designator"
  (let* ((?successfull-pose (try-movement-stamped-list
                             (list nav-goal-pose-stamped))))
    (print "xx")
    ;;(break)
    (exe:perform 
     (desig:a motion
              (type going)
              (pose ?successfull-pose)))))

;;====================================================================================================
;;knowledge


(defun knowledge-dummy()
  (cl-tf2::make-pose-stamped
               "map" 0
               (cl-tf2::make-3d-vector 0.16 0.322 0)
               (cl-tf2::make-quaternion 0 0 0 1)))

(defun knowledge-dummy2()
  (cl-tf2::make-pose-stamped
               "map" 0
               (cl-tf2::make-3d-vector 0.241 0.32 0)
               (cl-tf2::make-quaternion 0 0 0 1)))


;; TODO remove ' from prolog response. Example: |'Table_LTKIUPNG'| -> |Table_LTKIUPNG|
;; Note: Pipes (this -> |) need to stay
;;
;; TODO support für mehrere variablen  hinzufügen. Example object-type AND object-surface -> object-name
;; @author Luca Krohm
(defun call-knowledge (query &key param-list result)
  "Receives query `query', list of strings `param-list' and symbol `result' . Returns the query result for the chosen parameters.
- Possible querys:
--'has_tf_frame'"
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
  "Receives query `query', list of strings `param-list' and symbol `result' . Returns the query result for the chosen parameters.
- Possible querys:
--'has_tf_frame'"
  (roslisp:ros-info (knowledge-object-client)
                    "Calling query ~a."
                    query)
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog
                          (alexandria:flatten (list query result param-list))))))
    ;; (roslisp:ros-info (knowledge-object-client)
    ;;                   "Goal pose for object ~a is ~a."
    ;;                   object-name
    ;;                   raw-response)
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-object-client)
                                             "Query ~a didn't reach any solution."
                                             query))
      (t (fix-prolog-string (cdr (assoc (match-prolog-symbol result) (cut:lazy-car raw-response))))))))


;; (defun prolog-temporary-storage-pose (object-name)
;;   "Receives object name `object-name'. Returns the goal pose for an object name"
;;   (roslisp:ros-info (knowledge-object-client)
;;                     "Getting goal pose for object ~a."
;;                     object-name) 
;;   (let* ((raw-response (with-safe-prolog
;;                          (json-prolog:prolog
;;                           (list "has_tf_name" object-name 'frame)))))
;;     (roslisp:ros-info (knowledge-object-client)
;;                       "Goal pose for object ~a is ~a."
;;                       object-name
;;                       raw-response)
;;     (cond
;;       ((eq raw-response 1) (roslisp:ros-warn (knowledge-object-client)
;;                                              "Query object_goal_pose_offset didn't reach any solution."))
;;       (t (cdr (assoc '?frame (cut:lazy-car raw-response)))))))



;;=================================================================================================

;; (defun ensure-perception-pose (pose effort)
;;   (let ((position
;;           (etypecase action-type-or-position
;;             (number
;;              (cond
;;                ((< action-type-or-position *gripper-minimal-position*)
;;                 (roslisp:ros-warn (gripper-action)
;;                                   "POSITION (~a) cannot be < ~a. Clipping."
;;                                   action-type-or-position *gripper-minimal-position*)
;;                 *gripper-minimal-position*)
;;                ((> action-type-or-position *gripper-maximal-position*)
;;                 (roslisp:ros-warn (gripper-action)
;;                                   "POSITION (~a) shouldn't be > ~a. Clipping."
;;                                   action-type-or-position *gripper-maximal-position*)
;;                 *gripper-maximal-position*)
;;                (t
;;                 action-type-or-position)))
;;             (keyword
;;              (ecase action-type-or-position
;;                (:open *gripper-maximal-position*)
;;                (:close *gripper-minimal-position*)
;;                (:grip *gripper-minimal-position*)))))
;;         (effort
;;           (cl-tf2::make-pose-stamped
;;               "map" 0
;;               (cl-tf2::make-3d-vector 0.18888034742219937d0 -0.30079838556398194d0 0.2854991327720375d0)
;;               (cl-tf2::make-quaternion 0.0 0.0 0.0 1.0))))
;;     (values position effort)))

(defun perceive-handle-closed ()
   (cl-tf2::make-pose-stamped
    "map" 0
    (cl-tf2::make-3d-vector 0.18888034742219937d0 -0.30079838556398194d0 0.2854991327720375d0)
    (cl-tf2::make-quaternion 0.0 0.0 0.0 1.0)))

(defun perceive-handle-opened ()
   (cl-tf2::make-pose-stamped
    "map" 0
    (cl-tf2::make-3d-vector 0.18888034742219937d0 0.10079838556398194d0 0.2854991327720375d0)
    (cl-tf2::make-quaternion 0.0 0.0 0.0 1.0)))
