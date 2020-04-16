;;; Json-Prolog client for communication with KnowRob (Knowledge)
(in-package :llif)

(alexandria:define-constant +knowrob-prefix+
  "http://knowrob.org/kb/knowrob.owl#" :test 'string=)
(alexandria:define-constant +hsr-objects-prefix+
  "http://www.semanticweb.org/suturo/ontologies/2018/10/objects#" :test 'string=)
(alexandria:define-constant +robocup-prefix+
  "http://knowrob.org/kb/robocup.owl#" :test 'string=)
(alexandria:define-constant +srld-prefix+
  "http://knowrob.org/kb/srdl2-comp.owl#" :test 'string=)


(defmacro with-safe-prolog (&body body)
  `(handler-case
      ,@body
     (simple-error ()
       (roslisp:ros-error (json-prolog-client)
                            "Json prolog client error. Check your query again."))
     (SB-KERNEL:CASE-FAILURE ()
       (roslisp:ros-error (json-prolog-client) "Startup your rosnode first"))
     (ROSLISP::ROS-RPC-ERROR ()
       (roslisp:ros-error (json-prolog-client)
                            "Is the json_prolog server running?"))))

(defun knowrob-symbol->string (knowrob-symbol)
  "Converts a given knowrob symbol into a string."
  (string-trim "'" (subseq (write-to-string knowrob-symbol)
                           (1+ (position #\# (write-to-string knowrob-symbol)))
                           (- (length (write-to-string knowrob-symbol)) 2))))


(defun object-name->class (object-name)
  "returns the class of the given object name"
  (subseq object-name 0 (position #\_ object-name)))

(defun knowledge-set-tables-source ()
  "set the tables as source for storing objects"
  (roslisp:ros-info (json-prolog-client) "Set tables as source")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple "make_all_tables_source"
                                                    :package :llif))))))

(defun knowledge-set-ground-source ()
  "set the ground as source cleanup"
  (roslisp:ros-info (json-prolog-client) "Set ground as source")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple "make_ground_source."
                                                    :package :llif))))))

(defun knowledge-set-target-surfaces ()
  "set the target surfaces"
  (roslisp:ros-info (json-prolog-client) "Set target surfaces.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple "make_all_shelves_target."
                                                    :package :llif))))))

(defun prolog-add-test-objects ()
  "add some test objects to the knowledge base"
  (roslisp:ros-info (json-prolog-client) "Add objects to knowledge base")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple (concatenate 'string
                                                      "table_surfaces(Tables),"
                                                      "member(Table, Tables)," 
                                         "create_object_on_surface(Table)")
                                             :package :llif))))))

(defun prolog-table-objects ()
  "returns the list of all objects on the table"
  (roslisp:ros-info (json-prolog-client) "Getting objects on the table.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple (concatenate 'string
                                       "all_objects_on_tables(INSTANCES),"
                                       "member(INSTANCE, INSTANCES)")
                                             :package :llif)))
         (instances (if (eq raw-response 1)
                        NIL
                        (cdr (assoc '?instances (cut:lazy-car raw-response))))))
    (if instances
        (mapcar #'knowrob-symbol->string instances)
        (roslisp:ros-warn (json-prolog-client)
                           "Query didn't reach any solution."))))


(defun prolog-object-goal (object-name)
  "returns the goal shelf (tf-frame) for an object name"
  (roslisp:ros-info (json-prolog-client)
                     "Getting goal floor for object ~a." object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog  
                         (json-prolog:prolog-simple 
                              (concatenate 'string 
                                           "object_goal_surface('"
                                           knowrob-name "', SURFACE)," 
                                           "surface_frame(SURFACE, FRAME)")
                                             :package :llif)))
         (surface (if (eq raw-response 1)
                      NIL
                      (cdr (assoc '?frame (cut:lazy-car raw-response))))))
    (if surface
        (format nil "environment/~a" (string-trim "'" surface))
        (roslisp:ros-warn (json-prolog-client)
                           "Query didn't reach any solution."))))

(defun prolog-object-goal-pose (object-name)
  "returns the goal pose for an object name"
  (roslisp:ros-info (json-prolog-client)
                     "Getting goal pose for object ~a." object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "object_goal_pose_offset('"
                                                knowrob-name"', POSE, CONTEXT).")
                          :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (json-prolog-client)
                           "Query didn't reach any solution.")
        (values-list `(,(cdr (assoc '?pose (cut:lazy-car raw-response)))
                       ,(string-trim "'" (cdr (assoc '?context
                                              (cut:lazy-car raw-response)))))))))

(defun prolog-all-objects-in-shelf ()
 "returns all objects in the shelf"
  (roslisp:ros-info (json-prolog-client) "Getting all objects in shelf.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          (concatenate 'string
                                       "all_objects_in_whole_shelf(INSTANCES),"
                                       "member(INSTANCE, INSTANCES)")
                          :package :llif)))
            (instances (if (eq raw-response 1)  NIL 
                        (cdr (assoc '?instances (cut:lazy-car raw-response))))))
    (if instances
        (mapcar #'knowrob-symbol->string instances)
        (roslisp:ros-warn (json-prolog-client)
                           "Query didn't reach any solution."))))

(defun prolog-next-object ()
 "returns the next object to grasp"
  (roslisp:ros-info (json-prolog-client) "Getting next object to grasp.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          "next_object(OBJECT)"
                          :package :llif)))
    (object (if (eq raw-response 1) NIL 
              (cdr (assoc '?object (cut:lazy-car raw-response))))))
    (if object
        (knowrob-symbol->string object)
        (roslisp:ros-warn (json-prolog-client)
                           "Query didn't reach any solution."))))

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
        (roslisp:ros-warn (json-prolog-client)
                           "Query didn't reach any solution."))))

(defun prolog-object-pose (object-name)
  "returns the pose of an object"
  (roslisp:ros-info (json-prolog-client)
                     "Getting pose for object ~a." object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          (concatenate 'string "object_pose('"
                                                        knowrob-name "', POSE)")
                          :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (json-prolog-client)
                           "Query didn't reach any solution.")
        (values-list `(,(cdr (assoc '?pose (cut:lazy-car raw-response)))
                       ,(string-trim "'" (cdr 
                            (assoc '?context (cut:lazy-car raw-response)))))))))    

(defun prolog-table-pose ()
  "returns the pose of the table"
  (roslisp:ros-info (json-prolog-client) "Getting pose from table")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          (concatenate 'string "get_surface_id_by_name"
                                                "(table_center, TABLE),"
                                               "surface_pose_in_map"
                                               "(TABLE, [TRANSLATION, ROTATION])")
                          :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (json-prolog-client)
                           "Query didn't reach any solution.")
        (values-list `(,(cdr (assoc '?translation (cut:lazy-car raw-response)))
                       ,(string-trim "'" (cdr (assoc '?context
                                              (cut:lazy-car raw-response)))))))))

(defun prolog-shelf-pose ()
  "returns the pose of the shelf"
  (roslisp:ros-info (json-prolog-client) "Getting pose from shelf")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          (concatenate 'string "get_surface_id_by_name"
                                                "(shelf_floor_0_piece, SHELF),"
                                       "surface_pose_in_map"
                                       "(SHELF, [TRANSLATION, ROTATION])")
                          :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (json-prolog-client)
                           "Query didn't reach any solution.")
        (values-list `(,(cdr (assoc '?translation (cut:lazy-car raw-response)))
                       ,(string-trim "'" (cdr (assoc '?context
                                              (cut:lazy-car raw-response)))))))))                                                  

(defun prolog-object-in-gripper ()
  "returns the object in the gripper"
  (roslisp:ros-info (json-prolog-client) "Getting object in gripper.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                                       "gripper(Gripper)"
                          :package :llif)))
         (instance (if (eq raw-response 1) NIL 
                         (cdr (assoc '?instance (cut:lazy-car raw-response))))))
    (if instance
        (knowrob-symbol->string instance)
        (roslisp:ros-warn (json-prolog-client)
                           "Query didn't reach any solution."))))

(defun prolog-forget-table-objects ()
    "Forgets all objects on table"
  (roslisp:ros-info (json-prolog-client) "Forget all objects on table")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          (concatenate 'string "get_surface_id_by_name
                                                (table_2_center, TABLE),"
                                               "forget_objects_on_surface
                                                (TABLE).")
                          :package :llif))))))  

;;; former planning_communication/json-prolog.lisp
#+deprecated
((defun prolog-objects-around-pose (pose &optional (threshold 3.0))
  "checks if there are objects around a certain pose or not."
  (handler-case
      (with-slots (cl-tf:x cl-tf:y cl-tf:z) (cl-tf:origin pose)
        (json-prolog:prolog-simple
         (apply 'format nil
                "object_at(_, [map, _, [~a, ~a, ~a], [0, 0, 0, 1]], ~a, INST)"
                (mapcar (alexandria:rcurry 'coerce 'short-float) 
                        (list cl-tf:x cl-tf:y cl-tf:z threshold)))))
    (simple-error () (roslisp:ros-warn (prolog-table-objects) "No objects found"))))


(defun dummy-test ()
  "roslaunch object_state object_state.launch.
   basically checks the conenction to the knowledge node."
  (let ((mypose (cl-tf:make-pose (cl-tf:make-3d-vector 1 0 0.8)
                                 (cl-tf:make-identity-rotation))))
    (handler-case
        (and (json-prolog:prolog-simple "spawn_on_table")
             (prolog-objects-around-pose mypose))
    (simple-error () 
      (roslisp:ros-warn (dummy-test) "Something went wrong"))))))
