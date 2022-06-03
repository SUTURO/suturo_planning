(in-package :llif)

;; @author Tom-Eric Lehmkuhl, based on the code from suturo18/19
(defun prolog-object-goal (object-name)
  "Receives object name `object-name'. Returns the goal shelf (tf-frame) for an object name"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting goal surface for object ~a."
                    object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog  
                         (json-prolog:prolog-simple
                          (concatenate 'string 
                                       "object_goal_surface('"
                                       knowrob-name
                                       "', SURFACE)")
                          :package :llif)))
         (surface (unless (eq raw-response 1)
                    (remove-string +HSR-SURFACE-PREFIX+ (string-trim "'" (cdr (assoc '?surface (cut:lazy-car raw-response))))))))
    ;;(print "raw-respose")
    ;;(print raw-response)
    (or surface
        (roslisp:ros-warn (json-prolog-client)
                          "Query didn't object_goal_surface reach any solution."))))


;; used in tableclean
;; @author Tom-Eric Lehmkuhl, based on the code from suturo18/19
(defun prolog-temporary-storage-surface ()
  "Returns the temporary storage location - currently the long_table"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting temporary_storage_surface.")
  (let* ((raw-response (with-safe-prolog  
                         (json-prolog:prolog-simple "temporary_storage_surface(SURFACE)" :package :llif)))
         (surface (unless (eq raw-response 1)
                    (remove-string +HSR-SURFACE-PREFIX+ (string-trim "'" (cdr (assoc '?surface (cut:lazy-car raw-response))))))))
    ;;(print "raw-respose")
    ;;(print raw-response)
    (or surface
        (roslisp:ros-warn (json-prolog-client)
                          "Query temporary_storage_surface didn't reach any solution."))))

;;@author Felix Krause
(defun prolog-stored-objects ()
  "Returns all objects where prolog-temporary-storage-surface has been called with that object"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting stored_objects.")
  (let* ((raw-response (with-safe-prolog  
                         (json-prolog:prolog-simple "stored_objects(INSTANCES)" :package :llif)))
         (instances (unless (eq raw-response 1)
                      (cdr (assoc '?instances (cut:lazy-car raw-response))))))
    ;;(print "raw-respose")
    ;;(print raw-response)
    (mapcar (lambda (id) (remove-string +hsr-objects-prefix+ (string-trim "'" id))) instances)))

;; used in cleanup
;; @author Torge Olliges
(defun prolog-object-source (object-name)
  "Receives object name `object-name'. Returns the supporting surface of an object"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting supporting surface for object ~a."
                    object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "object_supported_by_surface('"
                                       knowrob-name
                                       "', SURFACE)")
                          :package :llif)))
         (supporting-surface
           (unless (eq raw-response 1)
             (remove-string +HSR-SURFACE-PREFIX+
                            (string-trim "'" (cdr
                                              (assoc
                                               '?surface
                                               (cut:lazy-car raw-response))))))))
    (or supporting-surface
        (roslisp:ros-warn (knowledge-object-client)
                          "Query didn't find_supporting_surface reach any solution."))))

;; used in cleanup
;; @author Tom-Eric Lehmkuhl, based on the code from suturo18/19
(defun prolog-object-goal-pose (object-name)
  "Receives object name `object-name'. Returns the goal pose for an object name"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting goal pose for object ~a."
                    object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "object_goal_pose_offset('"
                                       knowrob-name"', POSE, CONTEXT).")
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-object-client)
                                             "Query object_goal_pose_offset didn't reach any solution."))
      (t (values-list `(,(cdr (assoc '?pose (cut:lazy-car raw-response)))
                        ,(string-trim "'" (cdr (assoc '?context
                                                      (cut:lazy-car raw-response))))))))))


;; used in cleanup
;; @author Felix Krause
(defun prolog-temporary-storage-pose (object-name)
  "Receives object name `object-name'. Returns the goal pose for an object name"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting goal pose for object ~a."
                    object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "temporary_storage_pose('"
                                       knowrob-name"', POSE).")
                          :package :llif))))
    (roslisp:ros-info (knowledge-object-client)
                      "Goal pose for object ~a is ~a."
                      object-name
                      raw-response)
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-object-client)
                                             "Query object_goal_pose_offset didn't reach any solution."))
      (t (cdr (assoc '?pose (cut:lazy-car raw-response)))))))

;; used in cleanup
;; @author Felix Krause
(defun prolog-surface-rel-pose (surface-name x y zdistance)
  "Receives surface name `surface-name'. Returns the goal pose for wipe."
  (roslisp:ros-info (knowledge-object-client)
                    "Getting goal pose for surface ~a."
                    surface-name)
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (format nil "surface_rel_pose('~a~a', ~a, ~a, ~a, POSE)."
                                  +hsr-surface-prefix+ surface-name x y zdistance)
                          :package :llif))))
    (roslisp:ros-info (knowledge-object-client)
                      "Goal pose for surface ~a is ~a."
                      surface-name
                      raw-response)
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-object-client)
                                             "Query surface_rel_pose didn't reach any solution."))
      (t (cdr (assoc '?pose (cut:lazy-car raw-response)))))))

;;@author Felix Krause
(defun prolog-source-pose (object-name)
  "Receives object name `object-name'. Returns the source pose for an object name"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting goal pose for object ~a."
                    object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "source_pose('"
                                       knowrob-name"', FRAME, POSE).")
                          :package :llif))))
    (roslisp:ros-info (knowledge-object-client)
                      "Source pose for object ~a is ~a."
                      object-name
                      raw-response)
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-object-client)
                                             "Query object_goal_pose_offset didn't reach any solution."))
      (t (cdr (assoc '?pose (cut:lazy-car raw-response)))))))

;; used in cleanup
;; @author Torge Olliges
(defun prolog-next-object ()
  "Returns the next object to grasp"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting next object to grasp.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          "next_object(OBJECT,0)"
                          :package :llif)))
         (object (unless (or (eq raw-response nil)
                             (eq raw-response 1))
                   (remove-string +HSR-OBJECTS-PREFIX+
                                  (string-trim "'"
                                               (cdr
                                                (assoc '?object (cut:lazy-car raw-response))))))))
    (or object
        (roslisp:ros-warn (knowledge-object-client)
                          "Query next_object didn't reach any solution."))))

;; @author Torge Olliges
(defun prolog-next-graspable-objects ()
  "Returns the next graspable objects which are on a source surface"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting next graspable object on a source surface.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          "next_graspable_objects_on_source_surface(OBJECTS)"
                          :package :llif)))
         (object (unless (eq raw-response 1) 
                   (cdr (assoc '?object (cut:lazy-car raw-response))))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-object-client)
                                             "Query didn't next_object reach any solution."))
      (t (knowrob-symbol->string object)))))

;; @author Torge Olliges
(defun prolog-non-graspable-objects-on-surface (surface)
  "Receives surface `surface'. Returns a list of non graspable objects on a given surface"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting non graspable objects on surface: ~a" surface)
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          (concatenate 'string
                                       "all_not_graspable_objects_on_surface('"
                                       surface
                                       "', OBJECTS)")
                          :package :llif)))         
         (objects (unless (eq raw-response 1)
                    (cdr (assoc '?objects (cut:lazy-car raw-response))))))
    
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-object-client)
                                             "Query didn't next_object reach any solution."))
      (t (mapcar 'knowrob-symbol->string objects)))))

;; @author Torge Olliges
(defun prolog-set-object-not-graspable (object-name reason)
  "Receives object name `object-name' and a reason `reason'. Returns a list of non graspable objects on a given surface"
  (roslisp:ros-info (knowledge-object-client)
                    "setting objects not graspable")
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "set_not_graspable('"
                                       knowrob-name "', "
                                       (write-to-string reason) ").")
                          :package :llif)))
         (answer (unless (eq raw-response 1) 
                   t)))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-object-client)
                                             "Query didn't set_not_graspable reach any solution."))
      (t answer))))

;; Reasons:
;;   0 Object on Table to deep, cant reach (Knowledge)
;;   1 Object to close to a wall, cant grasp dew to collision avoidance (Giskard)
;;   2 Object is to small / to big for gripper (Knowledge)
;;   3 cant move near to the object to grab (Navigation)
;;   4 to high in a shelf (Knowledge)

;; @author Torge Olliges
(defun prolog-get-reason-for-object-goal-pose (object-name)
  "Receives object name `object-name'. Requests reason for the object pose from Knowledge"
  (roslisp:ros-info (knowledge-object-client) "")
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "object_reason_goal_pose('"
                                       knowrob-name"', Reason, Obj2ID).")
                          :package :llif)))
         (answer (unless (eq raw-response 1) 
                   (cdr (assoc '?reason (cut:lazy-car raw-response))))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-object-client)
                                             "Query object_reason_goal_pose didn't reach any solution."))
      (t answer))))

;; used in cleanup
;; @author Tom-Eric Lehmkuhl, based on the code from suturo18/19
(defun prolog-object-dimensions (object-name)
  "Receives object name `object-name'. Returns the dimensions of an object as list with '(depth width height)"
  (roslisp:ros-info (json-prolog-client)
                    "Getting dimensions for object ~a."
                    object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "object_dimensions('"
                                       knowrob-name "', D, W, H)")
                          :package :llif)))
         (raw-dimensions (unless (eq raw-response 1)
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
  "Receives object name `object-name'. Returns the pose of an object"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting pose for object ~a."
                    object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          (concatenate 'string "object_pose('"
                                       knowrob-name "', POSE)")
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-object-client)
                                             "Query object_pose didn't reach any solution."))
      (t (values-list `(,(cdr (assoc '?pose (cut:lazy-car raw-response)))
                        ,(string-trim "'" (cdr (assoc '?context
                                                      (cut:lazy-car raw-response))))))))))

;; @author Torge Olliges
(defun prolog-object-room (object-id)
  "Receives object ID `object-id'. Requests room that object is in from Knowledge"
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-id))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "object_in_room("
                                       knowrob-name
                                       ",ROOM)")
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query didn't object_in_room reach any solution."))
      (t (values-list (list (mapcar
                             (lambda (x) (string-trim "'" x))
                             (cdr (assoc '?Object (cut:lazy-car raw-response))))))))))

;; @author Torge Olliges
(defun prolog-room-objects (room-id)
  "Receives room ID `room-id'. Requests objects inside a room from Knowledge"
  (let* ((knowrob-name (format nil "~a~a" +hsr-rooms-prefix+ room-id))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "objects_in_room("
                                       knowrob-name
                                       ",OBJECTS)")
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query didn't objects_in_room reach any solution."))
      (t (values-list (list (mapcar
                             (lambda (x) (string-trim "'" x))
                             (cdr (assoc '?Objects (cut:lazy-car raw-response))))))))))

(defun prolog-furniture-objects (furniture-id)
  "Receives furniture ID `furniture-id'. Requests objects inside of furniture from Knowledge"
  (let* ((knowrob-name (format nil "~a~a" +hsr-rooms-prefix+ furniture-id))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "furniture_all_objects("
                                       knowrob-name
                                       ",OBJECTS)")
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query didn't objects_in_room reach any solution."))
      (t (values-list (list (mapcar
                             (lambda (x) (string-trim "'" x))
                             (cdr (assoc '?Objects (cut:lazy-car raw-response))))))))))

;; used in cleanup
(defun prolog-set-object-handled (object-id)
  "Receives object ID `object-id'. Tells Knowledge that an object was handled"
  (let* ((knowrob-name (format nil "~a~a" +HSR-OBJECTS-PREFIX+ object-id))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "set_object_handeled('"
                                       knowrob-name
                                       "')")
                          :package :llif))))
    (roslisp:ros-warn (knowledge-surface-client)
                      "Query set_object_handled executed.")
    raw-response))

;; @author Luca Krohm
(defun prolog-get-object (object-type)
  "Receives object type `object-type'. Asks Knowledge for the name of an object of type `object-type'."
  (roslisp:ros-info (knowledge-object-client)
                    "Getting object ~a."
                    object-type)
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                          "has_type(OBJECT,hsr_objects:'"
                          object-type
                          "')")
                          :package :llif)))
         (object (unless (or (eq raw-response nil)
                             (eq raw-response 1))
                   (remove-string
                    +HSR-OBJECTS-PREFIX+
                    (string-trim "'"
                                 (cdr
                                  (assoc '?object (cut:lazy-car raw-response))))))))
    (or object
        (roslisp:ros-warn (knowledge-object-client)
                          "Query has_type didn't reach any solution."))))


;; \author Luca Krohm
(defun prolog-get-object-surfaces (object-type)
  "Receives object type `object-type'. Asks knowledge for the default surface of `object-type'"
  (roslisp:ros-info (knowledge-object-client)
                    "Getting default surface for object-type ~a." object-type)
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                          "get_current_or_default_surfaces(hsr_objects:'"
                          object-type
                          "',SURFACES)")
                          ;;"get_sponge_surfaces(SURFACES)"
                          :package :llif)))
         (surfaces (unless (or (eq raw-response nil)
                             (eq raw-response 1))
                     (mapcar (lambda (id)
                               (remove-string +hsr-surface-prefix+ (string-trim "'" id)))
                             (cdr
                              (assoc '?surfaces (cut:lazy-car raw-response)))))))
    (or surfaces
        (roslisp:ros-warn (knowledge-object-client)
                          "Query get_current_or_default_surfaces didn't reach any solution."))))
