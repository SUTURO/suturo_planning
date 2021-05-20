
;;; Json-Prolog client for communication with KnowRob (Knowledge)
(in-package :llif)

(alexandria:define-constant +knowrob-prefix+
  "http://knowrob.org/kb/knowrob.owl#" :test 'string=)
(alexandria:define-constant +hsr-objects-prefix+
  "http://www.semanticweb.org/suturo/ontologies/2020/3/objects#" :test 'string=)
(alexandria:define-constant +hsr-rooms-prefix+
  "http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#" :test 'string=)
(alexandria:define-constant +robocup-prefix+
  "http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#" :test 'string=)
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

;; @author Tom-Eric Lehmkuhl
(defun knowledge-set-tables-source ()
  "set the tables as source for storing objects"
  (roslisp:ros-info (json-prolog-client) "Set tables as source")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple "make_all_tables_source"
                                                    :package :llif))))))

;; @author Tom-Eric Lehmkuhl
(defun knowledge-set-ground-source ()
  "set the ground as source cleanup"
  (roslisp:ros-info (json-prolog-client) "Set ground as source")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple "make_ground_source."
                                                    :package :llif))))))

;; @author Tom-Eric Lehmkuhl
(defun knowledge-set-buckets-target ()
  "set buckets as target surfaces"
  (roslisp:ros-info (json-prolog-client) "Set buckets as target surfaces.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple "make_all_buckets_target."
                                                    :package :llif))))))

;; @author Tom-Eric Lehmkuhl
(defun knowledge-set-target-surfaces ()
  "set the target surfaces"
  (roslisp:ros-info (json-prolog-client) "Set target surfaces.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple "make_all_shelves_target."
                                                    :package :llif))))))

;; @author Tom-Eric Lehmkuhl
(defun prolog-add-test-objects ()
  "add some test objects to the knowledge base"
  (roslisp:ros-info (json-prolog-client) "Add objects to knowledge base")
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

;; @author Tom-Eric Lehmkuhl, based on the code from suturo18/19
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

;; @author Tom-Eric Lehmkuhl
(defun prolog-forget-table-objects ()
  "Forgets all objects on table"
  (roslisp:ros-info (json-prolog-client) "Forget all objects on table")
  (let* ((raw-response
           (with-safe-prolog
             (json-prolog:prolog-simple 
              (concatenate
               'string "get_surface_id_by_name (table_2_center, TABLE),"
               "forget_objects_on_surface(TABLE).")
              :package :llif))))))

;; @author Jan Schimpf
;; ask knowledge for how the object should be grasped 
;; 1 for grasping from front and 2 for grasping from the top
;;(defun prolog-object-grasp-mode (object-id))
;;    "Asking knowledge what graspmode to use"
;;    (roslisp:ros-info (json-prolog-client) "Asking knowledge what graspmode to use")
;;    (let* ((raw-response
;;           (with-safe-prolog
;;             (json-prolog:prolog-simple 
;;               
;;              :package :llif))))))  


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
