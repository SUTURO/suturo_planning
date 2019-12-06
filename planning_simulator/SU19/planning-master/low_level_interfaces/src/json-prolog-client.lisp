;;; Json-Prolog client for communication with KnowRob (Knowledge)


(in-package :lli)

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
       (roslisp:ros-error (json-prolog-client) "Json prolog client error. Check your query again."))
     (SB-KERNEL:CASE-FAILURE ()
       (roslisp:ros-error (json-prolog-client) "Startup your rosnode first"))
     (ROSLISP::ROS-RPC-ERROR ()
       (roslisp:ros-error (json-prolog-client) "Is the json_prolog server running?"))))


(defun knowrob-symbol->string (knowrob-symbol)
  "Converts a given knowrob symbol into a string."
  (string-trim "'" (subseq (write-to-string knowrob-symbol)
                           (1+ (position #\# (write-to-string knowrob-symbol)))
                           (- (length (write-to-string knowrob-symbol)) 2))))


(defun object-name->class (object-name)
  "returns the class of the given object name"
  (subseq object-name 0 (position #\_ object-name)))


(defun prolog-table-objects ()
  "returns the list of all objects on the table"
  (roslisp:ros-info (json-prolog-client) "Getting objects on the table.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog `(and ("table_surface" ?table)
                                                   ("objects_on_surface" ?instances ?table))
                                             :package :lli)))
         (instances (if (eq raw-response 1)
                        NIL
                        (cdr (assoc '?instances (cut:lazy-car raw-response))))))
    (if instances
        (mapcar #'knowrob-symbol->string instances)
        (roslisp:ros-warn (json-prolog-client) "Query didn't reach any solution."))))


(defun prolog-object-goal (object-name)
  "returns the goal shelf (tf-frame) for an object name"
  (roslisp:ros-info (json-prolog-client) "Getting goal floor for object ~a." object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (rdf-urdf (format nil "~aurdfName" +srld-prefix+))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog `(and ("object_goal_surface" ,knowrob-name ?surface)
                                                   ("rdf_has_prolog" ?surface ,rdf-urdf ?urdfname))
                                             :package :lli)))
         (surface (if (eq raw-response 1)
                      NIL
                      (cdr (assoc '?urdfname (cut:lazy-car raw-response))))))
    (if surface
        (format nil "environment/~a" (string-trim "'" surface))
        (roslisp:ros-warn (json-prolog-client) "Query didn't reach any solution."))))

(defun prolog-object-goal-pose (object-name)
  "returns the goal shelf (tf-frame) for an object name"
  (roslisp:ros-info (json-prolog-client) "Getting goal floor for object ~a." object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-1 `("object_goal_pose" ,knowrob-name ?pose ?context)
                                             :package :lli))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (json-prolog-client) "Query didn't reach any solution.")
        (values-list `(,(cdr (assoc '?pose (cut:lazy-car raw-response)))
                       ,(string-trim "'" (cdr (assoc '?context (cut:lazy-car raw-response)))))))))

(defun prolog-all-objects-in-shelf ()
 "returns the goal shelf (tf-frame) for an object name"
  (roslisp:ros-info (json-prolog-client) "Getting all objects in shelf.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog `(and ("all_objects_in_whole_shelf" ?instances)
                                                   ("member" ?instance ?instances))
                                             :package :lli)))
         (instances (if (eq raw-response 1)  NIL (cdr (assoc '?instances (cut:lazy-car raw-response))))))
    (if instances
        (mapcar #'knowrob-symbol->string instances)
        (roslisp:ros-warn (json-prolog-client) "Query didn't reach any solution."))))

(defun prolog-object-dimensions (object-name)
  "returns the dimensions of an object as list with '(depth width height)"
  (roslisp:ros-info (json-prolog-client) "Getting dimensions for object ~a." object-name)
  (let* ((knowrob-name (format nil "~a~a" +hsr-objects-prefix+ object-name))
         (dimension-term (format nil "~aboundingBoxSize" +knowrob-prefix+))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog `("rdf_has_prolog" ,knowrob-name ,dimension-term ?dim)
                                             :package :lli)))
         (dimensions (if (eq raw-response 1)
                         NIL
                         (cdr (assoc '?dim (cut:lazy-car raw-response))))))
    (or dimensions
        (roslisp:ros-warn (json-prolog-client) "Query didn't reach any solution."))))

(defun prolog-object-in-gripper ()
  "returns the dimensions of an object as list with '(depth width height)"
  (roslisp:ros-info (json-prolog-client) "Getting object in gripper.")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-1 `(and ("gripper" ?gripper)
                                                     ("objects_on_surface" ?instances ?gripper)
                                                     ("member" ?instance ?instances))
                                               :package :lli)))
         (instance (if (eq raw-response 1) NIL (cdr (assoc '?instance (cut:lazy-car raw-response))))))
    (if instance
        (knowrob-symbol->string instance)
        (roslisp:ros-warn (json-prolog-client) "Query didn't reach any solution."))))

;;; former planning_communication/json-prolog.lisp
;;; +depricated?

(defun prolog-objects-around-pose (pose &optional (threshold 0.3))
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
      (roslisp:ros-warn (dummy-test) "Something went wrong")))))
