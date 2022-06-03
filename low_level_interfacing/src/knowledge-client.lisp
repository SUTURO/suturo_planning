
;;; Json-Prolog client for communication with KnowRob (Knowledge)
(in-package :llif)

(alexandria:define-constant +knowrob-prefix+
  "http://knowrob.org/kb/knowrob.owl#" :test 'string=)
(alexandria:define-constant +hsr-objects-prefix+
  "http://www.semanticweb.org/suturo/ontologies/2020/3/objects#" :test 'string=)
(alexandria:define-constant +hsr-rooms-prefix+
  "http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#" :test 'string=)
(alexandria:define-constant +hsr-surface-prefix+
  "http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#" :test 'string=)
(alexandria:define-constant +robocup-prefix+
  "http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#" :test 'string=)
(alexandria:define-constant +srld-prefix+
  "http://knowrob.org/kb/srdl2-comp.owl#" :test 'string=)


(defmacro with-safe-prolog (&body body)
  "Receives body of LISP code `body'. Runs body with failurehandling."
  `(handler-case
       ,@body
     (simple-error ()
       (roslisp:ros-error (json-prolog-client)
                          "Json prolog client error. Check your query again."))
     (SB-KERNEL:CASE-FAILURE ()
       (roslisp:ros-error (json-prolog-client)
                          "Startup your rosnode first"))
     (ROSLISP::ROS-RPC-ERROR ()
       (roslisp:ros-error (json-prolog-client)
                          "Is the json_prolog server running?"))))

(defun knowrob-symbol->string (knowrob-symbol)
  "Receives knowrob symbol `knowrob-symbol'. Converts a given knowrob symbol into a string."
  (string-trim "'" (subseq (write-to-string knowrob-symbol)
                           (1+ (position #\# (write-to-string knowrob-symbol)))
                           (- (length (write-to-string knowrob-symbol)) 2))))

(defun object-name->class (object-name)
  "Receives object name `object-name'. Returns the class of the given object name"
  (subseq object-name 0 (position #\_ object-name)))

;; @author Tom-Eric Lehmkuhl
(defun knowledge-set-tables-source ()
  "Set the tables as source for storing objects"
  (roslisp:ros-info (json-prolog-client)
                    "Set tables as source")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple "make_all_tables_source"
                                                    :package :llif))))))

;; @author Tom-Eric Lehmkuhl
(defun knowledge-set-ground-source ()
  "Set the ground as source cleanup"
  (roslisp:ros-info (json-prolog-client)
                    "Set ground as source")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple "make_ground_source."
                                                    :package :llif))))))

;; @author Tom-Eric Lehmkuhl
(defun knowledge-set-buckets-target ()
  "Set buckets as target surfaces"
  (roslisp:ros-info (json-prolog-client)
                    "Set buckets as target surfaces.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple "make_all_buckets_target."
                                                    :package :llif))))))

;; @author Tom-Eric Lehmkuhl
(defun knowledge-set-target-surfaces ()
  "Set the target surfaces"
  (roslisp:ros-info (json-prolog-client)
                    "Set target surfaces.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple "make_all_shelves_target."
                                                    :package :llif))))))

;; @author Tom-Eric Lehmkuhl
(defun prolog-add-test-objects ()
  "Add some test objects to the knowledge base"
  (roslisp:ros-info (json-prolog-client)
                    "Add objects to knowledge base")
  (let* ((raw-response
           (with-safe-prolog
             (json-prolog:prolog-simple
              (concatenate 'string
                           "table_surfaces(Tables),"
                           "member(Table, Tables)," 
                           "create_object_on_surface(Table)")
              :package :llif))))))

;; @author Tom-Eric Lehmkuhl, based on the code from suturo18/19
(defun prolog-all-objects-in-shelf ()
  "Returns all objects in the shelf"
  (roslisp:ros-info (json-prolog-client)
                    "Getting all objects in shelf.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          (concatenate 'string
                                       "all_objects_in_whole_shelf(INSTANCES),"
                                       "member(INSTANCE, INSTANCES)")
                          :package :llif)))
         (instances (unless (eq raw-response 1) 
                      (cdr (assoc '?instances (cut:lazy-car raw-response))))))
    (cond
      (instances (mapcar #'knowrob-symbol->string instances))
      (t (roslisp:ros-warn (json-prolog-client)
                           "Query didn't reach any solution.")))))

;; used in cleanup
;; @author Tom-Eric Lehmkuhl, based on the code from suturo18/19
(defun prolog-object-in-gripper ()
  "Returns the object in the gripper"
  (roslisp:ros-info (json-prolog-client)
                    "Getting object in gripper.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          "all_objects_in_gripper(OBJECTS)"
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query didn't objects_in_room reach any solution."))
      (t (values-list (list (mapcar
                             (lambda (x) (string-trim "'" x))
                             (cdr (assoc '?Objects (cut:lazy-car raw-response))))))))))

;; @author Tom-Eric Lehmkuhl
(defun prolog-forget-table-objects ()
  "Forgets all objects on table"
  (roslisp:ros-info (json-prolog-client)
                    "Forget all objects on table")
  (let* ((raw-response
           (with-safe-prolog
             (json-prolog:prolog-simple 
              (concatenate
               'string "get_surface_id_by_name (table_2_center, TABLE),"
               "forget_objects_on_surface(TABLE).")
              :package :llif))))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;; Luca technische Präsentation ;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; @author Tom-Eric Lehmkuhl
(defun prolog-get-objects ()
  "Gets all objects on table"
  (roslisp:ros-info (json-prolog-client)
                    "Gets all objects on table")
  (let* ((raw-response
           (with-safe-prolog
             (json-prolog:prolog-simple 
              "findall(Object, object_manipulation:is_suturo_object(Object), OBJECTS)"
              :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query findall is_suturo_object didn't reach any solution."))
      (t (mapcar
          (lambda (x) (remove-string +HSR-OBJECTS-PREFIX+ (string-trim "'" x)))
          (cdr (assoc '?Objects (cut:lazy-car raw-response))))))))
