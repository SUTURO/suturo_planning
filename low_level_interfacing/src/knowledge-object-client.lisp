(in-package :llif)


;; @author Tom-Eric Lehmkuhl, based on the code from suturo18/19
(defun prolog-object-goal (object-name)
  "returns the goal shelf (tf-frame) for an object name"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting goal surface for object ~a." object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog  
                         (json-prolog:prolog-simple
                          (concatenate 'string 
                                       "object_goal_surface('"
                                       knowrob-name
                                       "', SURFACE)")
                          :package :llif)))
         (surface (if (eq raw-response 1)
                      NIL
                      (remove-string +HSR-SURFACE-PREFIX+ (string-trim "'" (cdr (assoc '?surface (cut:lazy-car raw-response))))))))
    ;;(print "raw-respose")
    ;;(print raw-response)
    (or surface
        (roslisp:ros-warn (json-prolog-client)
                          "Query didn't object_goal_surface reach any solution."))))


;; used in tableclean
;; @author Tom-Eric Lehmkuhl, based on the code from suturo18/19
(defun prolog-temporary-storage-surface ()
  "retruns the temporary storage location - currently the long_table"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting temporary_storage_surface.")
  (let* ((raw-response (with-safe-prolog  
                         (json-prolog:prolog-simple "temporary_storage_surface(SURFACE)" :package :llif)))
         (surface (if (eq raw-response 1)
                      NIL
                      (remove-string +HSR-SURFACE-PREFIX+ (string-trim "'" (cdr (assoc '?surface (cut:lazy-car raw-response))))))))
    ;;(print "raw-respose")
    ;;(print raw-response)
    (or surface
        (roslisp:ros-warn (json-prolog-client)
                          "Query temporary_storage_surface didn't reach any solution."))))

;; used in cleanup
;; @author Torge Olliges
(defun prolog-object-source (object-name)
  "returns the supporting surface of an object"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting supporting surface for object ~a." object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "object_supported_by_surface('"
                                       knowrob-name
                                       "', SURFACE)")
                          :package :llif)))
         (supporting-surface (if (eq raw-response 1)
                                 NIL
                                 (remove-string +HSR-SURFACE-PREFIX+ (string-trim "'" (cdr 
                                    (assoc '?surface 
                                      (cut:lazy-car raw-response))))))))
    (or supporting-surface
        (roslisp:ros-warn (knowledge-object-client)
                          "Query didn't find_supporting_surface reach any solution."))))

;; used in cleanup
;; @author Tom-Eric Lehmkuhl, based on the code from suturo18/19
(defun prolog-object-goal-pose (object-name)
  "returns the goal pose for an object name"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting goal pose for object ~a." object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "object_goal_pose_offset('"
                                       knowrob-name"', POSE, CONTEXT).")
                          :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-object-client)
                          "Query object_goal_pose_offset didn't reach any solution.")
        (values-list `(,(cdr (assoc '?pose (cut:lazy-car raw-response)))
                       ,(string-trim "'" (cdr (assoc '?context
                                                     (cut:lazy-car raw-response)))))))))


;; used in cleanup
;; @author Tom-Eric Lehmkuhl, based on the code from suturo18/19
(defun prolog-temporary-storage-pose (object-name)
  "returns the goal pose for an object name"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting goal pose for object ~a." object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "temporary_storage_pose('"
                                       knowrob-name"', POSE).")
                          :package :llif))))
    (roslisp:ros-info (knowledge-object-client)
                      "Goal pose for object ~a is ~a." object-name raw-response)
     (print "1.1")
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-object-client)
                          "Query object_goal_pose_offset didn't reach any solution.")
        (cdr (assoc '?pose (cut:lazy-car raw-response))))))

;; used in cleanup
;; @author Torge Olliges
(defun prolog-next-object ()
 "returns the next object to grasp"
  (roslisp:ros-info (knowledge-object-client) "Getting next object to grasp.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          "next_object(OBJECT,0)"
                          :package :llif)))
         (object (if (or (eq raw-response nil)
                         (eq raw-response 1))
                     nil
                     (remove-string +HSR-OBJECTS-PREFIX+
                                    (string-trim "'"
                                                 (cdr
                                                  (assoc '?object (cut:lazy-car raw-response))))))))
    (or object
        (roslisp:ros-warn (knowledge-object-client)
                          "Query next_object didn't reach any solution."))))
                          
;; @author Torge Olliges
(defun prolog-next-graspable-objects ()
 "returns the next graspable objects which are on a source surface"
  (roslisp:ros-info (knowledge-object-client) "Getting next graspable object on a source surface.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          "next_graspable_objects_on_source_surface(OBJECTS)"
                          :package :llif)))
         (object (if (eq raw-response 1) NIL 
                     (cdr (assoc '?object (cut:lazy-car raw-response))))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-object-client)
                          "Query didn't next_object reach any solution.")
        (knowrob-symbol->string object))))

;; @author Torge Olliges
(defun prolog-non-graspable-objects-on-surface (surface)
 "returns a list of non graspable objects on a given surface"
  (roslisp:ros-info (knowledge-object-client) "Getting non graspable objects on surface: ~a" surface)
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          (concatenate 'string
                                       "all_not_graspable_objects_on_surface('"
                                       surface
                                       "', OBJECTS)")
                          :package :llif)))         
          (objects (if (eq raw-response 1) NIL 
                     (cdr (assoc '?objects (cut:lazy-car raw-response))))))
    
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-object-client)
                          "Query didn't next_object reach any solution.")
        (mapcar 'knowrob-symbol->string objects))))

;; @author Torge Olliges
(defun prolog-set-object-not-graspable (object-name reason)
 "returns a list of non graspable objects on a given surface"
  (roslisp:ros-info (knowledge-object-client) "setting objects not graspable")
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                          (json-prolog:prolog-simple
                           (concatenate 'string
                                        "set_not_graspable('"
                                        knowrob-name "', "
                                        (write-to-string reason) ").")
                          :package :llif)))
         (answer (if (eq raw-response 1) 
                      NIL 
                      t)))
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-object-client)
                          "Query didn't set_not_graspable reach any solution.")
        answer)))

;; Reasons:
;;   0 Object on Table to deep, cant reach (Knowledge)
;;   1 Object to close to a wall, cant grasp dew to collision avoidance (Giskard)
;;   2 Object is to small / to big for gripper (Knowledge)
;;   3 cant move near to the object to grab (Navigation)
;;   4 to high in a shelf (Knowledge)

;; @author Torge Olliges
(defun prolog-get-reason-for-object-goal-pose (object-name)
 ""
  (roslisp:ros-info (knowledge-object-client) "")
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                          (json-prolog:prolog-simple
                            (concatenate 'string "object_reason_goal_pose('"
                              knowrob-name"', Reason, Obj2ID).")
                          :package :llif)))
          (answer (if (eq raw-response 1) NIL 
                     (cdr (assoc '?reason (cut:lazy-car raw-response))))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-object-client)
                          "Query object_reason_goal_pose didn't next_object reach any solution.")
        answer)))

;; used in cleanup
;; @author Tom-Eric Lehmkuhl, based on the code from suturo18/19
(defun prolog-object-dimensions (object-name)
  "returns the dimensions of an object as list with '(depth width height)"
  (roslisp:ros-info (json-prolog-client)
                    "Getting dimensions for object ~a." object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "object_dimensions('"
                                       knowrob-name "', D, W, H)")
                          :package :llif)))
         (raw-dimensions (if (eq raw-response 1)
                             NIL
                             (cut:lazy-car raw-response)))
         (dimensions (when raw-dimensions
                       (list (cdr (assoc '?D raw-dimensions))
                             (cdr (assoc '?W raw-dimensions))
                             (cdr (assoc '?H raw-dimensions))))))
    (or dimensions
        (roslisp:ros-warn (knowledge-object-client)
                          "Query object_dimensions didn't reach any solution."))))

;; used in cleanup
;; @author Tom-Eric Lehmkuhl, based on the code from suturo18/19
(defun prolog-object-pose (object-name)
  "returns the pose of an object"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting pose for object ~a." object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          (concatenate 'string "object_pose('"
                                       knowrob-name "', POSE)")
                          :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-object-client)
                          "Query object_pose didn't reach any solution.")
        (values-list `(,(cdr (assoc '?pose (cut:lazy-car raw-response)))
                       ,(string-trim "'"
                                     (cdr 
                                      (assoc '?context
                                             (cut:lazy-car raw-response)))))))))
                                      
;; @author Torge Olliges
(defun prolog-object-room (object-id)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                           (json-prolog:prolog-simple
                           (concatenate 'string
                                        "object_in_room("
                                        knowrob-name
                                        ",ROOM)")
                           :package :llif))))
     (if (eq raw-response 1)
         (roslisp:ros-warn (knowledge-surface-client)
                           "Query didn't object_in_room reach any solution.")
         (values-list (list (mapcar
                       (lambda (x) (string-trim "'" x))
                       (cdr (assoc '?Object (cut:lazy-car raw-response)))))))))

;; @author Torge Olliges
(defun prolog-room-objects (room-id)
  (let* ((knowrob-name (format nil "~a~a" +hsr-rooms-prefix+ room-id))
         (raw-response (with-safe-prolog
                           (json-prolog:prolog-simple
                           (concatenate 'string
                                        "objects_in_room("
                                        knowrob-name
                                        ",OBJECTS)")
                           :package :llif))))
     (if (eq raw-response 1)
         (roslisp:ros-warn (knowledge-surface-client)
                           "Query didn't objects_in_room reach any solution.")
         (values-list (list (mapcar
                       (lambda (x) (string-trim "'" x))
                       (cdr (assoc '?Objects (cut:lazy-car raw-response)))))))))

(defun prolog-furniture-objects (furniture-id)
  (let* ((knowrob-name (format nil "~a~a" +hsr-rooms-prefix+ room-id))
         (raw-response (with-safe-prolog
                           (json-prolog:prolog-simple
                           (concatenate 'string
                                        "furniture_all_objects("
                                        knowrob-name
                                        ",OBJECTS)")
                           :package :llif))))
     (if (eq raw-response 1)
         (roslisp:ros-warn (knowledge-surface-client)
                           "Query didn't objects_in_room reach any solution.")
         (values-list (list (mapcar
                       (lambda (x) (string-trim "'" x))
                       (cdr (assoc '?Objects (cut:lazy-car raw-response)))))))))

;; used in cleanup
(defun prolog-set-object-handled (object-id)
  (let* ((knowrob-name (format nil "~a~a" +HSR-OBJECTS-PREFIX+ object-id))
        (raw-response (with-safe-prolog
                        (json-prolog:prolog-simple
                         (concatenate 'string
                                      "set_object_handeled('"
                                        knowrob-name
                                        "')")
                           :package :llif))))
    (roslisp:ros-warn (knowledge-surface-client) "Query set_object_handled executed.")
    raw-response))
