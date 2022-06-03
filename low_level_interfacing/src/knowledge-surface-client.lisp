(in-package :llif)

;; used in cleanup
;; @author Torge Olliges
(defun prolog-surface-pose (surface-name)
  "Receives surface name `surface-name'. Returns the pose of the table"
  (roslisp:ros-info (knowledge-surface-client)
                    "Getting pose of surface ~a"
                    surface-name)
  (let* ((knowrob-name (format nil "~a~a" +HSR-SURFACE-PREFIX+ surface-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          (concatenate 'string 
                                       "surface_center_pose('"
                                       knowrob-name
                                       "',[TRANSLATION, ROTATION])")
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query didn't surface_pose_in_map reach any solution."))
      (t (values-list `(,(list (cdr (assoc '?translation (cut:lazy-car raw-response)))
                               (cdr (assoc '?rotation (cut:lazy-car raw-response))))
                        ,(string-trim "'"
                                      (cdr
                                       (assoc '?context
                                              (cut:lazy-car raw-response))))))))))



;; used in cleanup
;; @author Torge Olliges
(defun prolog-set-surface-not-visited (surface-id)
  "Receives surface ID `surface-id'. Sets surface as not visited in Knowledge."
  (let* ((knowrob-name (format nil "~a~a" +HSR-SURFACE-PREFIX+ surface-id))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "set_surface_not_visited('"
                                       knowrob-name
                                       "')")
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query didn't set_surface_visited reach any solution."))
      (t (roslisp:ros-info (Knowledge-surface-client)
                           "Visit state for  ~a set to not visited."
                           surface-id)))))

;; used in cleanup
;; @author Torge Olliges
(defun prolog-set-surface-visited (surface-id)
  "Receives surface ID `surface-id'. Sets surface as visited in Knowledge."
  (let* ((knowrob-name (format nil "~a~a" +HSR-SURFACE-PREFIX+ surface-id))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "set_surface_visited('"
                                       knowrob-name
                                       "')")
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query didn't set_surface_visited reach any solution."))
      (t (roslisp:ros-info (Knowledge-surface-client) "Visit state for ~a set to visited." surface-id)))))



;; used in cleanup
;; @author Torge Ollige
(defun prolog-surfaces-not-visited-in-room (room-id)
  "Receives room ID `room-id'. Asks Knowledge for all unvisited surfaces in room."
  (let* ((knowrob-name (format nil "~a~a" +HSR-ROOMS-PREFIX+ room-id))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "surfaces_not_visited_in_room('"
                                       knowrob-name
                                       "',SURFACES)")
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query didn't surfaces_not_visited reach any solution."))
      (t (values-list (list (mapcar
                             (lambda (x) (remove-string +HSR-SURFACE-PREFIX+ (string-trim "'" x)))
                             (cdr (assoc '?Surfaces (cut:lazy-car raw-response))))))))))

;; used in cleanup
;;author Torge Olliges
(defun prolog-all-rooms ()
  "Asks Knowledge for a list of all available rooms."
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          "all_rooms(ROOMS)"
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query all_rooms didn't reacha any solution"))
      (t (values-list (list (mapcar
                             (lambda (x) (remove-string +HSR-ROOMS-PREFIX+ (string-trim "'" x)))
                             (cdr (assoc '?Rooms (cut:lazy-car raw-response))))))))))

;; used in cleanup
;; @author Torge Olliges
(defun prolog-current-room ()
  "Asks Knowledge for the current room"
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "robot_in_room(ROOM)")
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query robot_in_room didn't reach any solution."))
      (t (roslisp::ros-info (prolog-current-room)
                            "I am in room ~a"
                            (string-trim "'" (cdr (assoc '?Room (cut:lazy-car raw-response)))))
         (remove-string +HSR-ROOMS-PREFIX+
                        (string-trim "'" (cdr (assoc '?Room (cut:lazy-car raw-response)))))))))

;; used in cleanup
;; @author Torge Olliges
(defun prolog-room-surfaces (room)
  "Receives room `room'. Asks Knowledge for all surfaces available in a room."
  (let* ((knowrob-name (format nil "~a~a" +HSR-ROOMS-PREFIX+ room))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "surfaces_in_room('"
                                       knowrob-name
                                       "',SURFACES)")
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query didn't surfaces_in_room reach any solution."))
      (t (values-list (list (mapcar
                             (lambda (x) (remove-string +HSR-SURFACE-PREFIX+ (string-trim "'" x)))
                             (cdr (assoc '?Surfaces (cut:lazy-car raw-response))))))))))

;; used in cleanup
;;@author Torge Olliges
(defun prolog-surface-region (surface-id)
  "Receives surface ID `surface-id'. Asks Knowledge for surface region of the surface."
  (let* ((knowrob-name (format nil "~a~a" +HSR-SURFACE-PREFIX+ surface-id))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "get_perception_surface_region('"
                                       knowrob-name
                                       "',REGION)")
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query surface_in_room didn't reach any solution."))
      (t (string-trim "\""(string-trim "'" (cdr (assoc '?Region (cut:lazy-car raw-response)))))))))

;; used in cleanup
;;@author Torge Olliges
(defun prolog-pose-to-perceive-surface (surface-id)
  "Receives surface ID `surface-id'. Asks Knowledge for a viable pose to perceive the surface."
  (let* ((knowrob-name (format nil "~a~a" +HSR-SURFACE-PREFIX+ surface-id))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "surface_pose_to_perceive_from('"
                                       knowrob-name
                                       "', [TRANSLATION, ROTATION])")
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query surface_pose_to_perceive_from didn't reach any solution."))
      (t (values-list `(,(list (cdr (assoc '?translation (cut:lazy-car raw-response)))
                               (cdr (assoc '?rotation (cut:lazy-car raw-response))))
                        ,(string-trim "'" (cdr (assoc '?context (cut:lazy-car raw-response))))))))))

;;source https://stackoverflow.com/questions/669407/common-lisp-the-remove-function-how-is-it-used
(defun remove-string (rem-string full-string &key from-end (test #'eql)
                                               test-not (start1 0) end1 (start2 0) end2 key)
  "Returns full-string with rem-string removed"
  (let ((subst-point (search rem-string full-string 
                             :from-end from-end
                             :test test :test-not test-not
                             :start1 start1 :end1 end1
                             :start2 start2 :end2 end2 :key key)))
    (cond
      (subst-point (concatenate 'string
                                (subseq full-string 0 subst-point)
                                (subseq full-string (+ subst-point (length rem-string)))))
      (t full-string))))

;;used in go-get-it
;;this is also just implemented for the robocup 2021
(defun prolog-deliver-object-pose (side)
  "Receives side `side'. Asks Knowledge for a pose to deliver an object to."
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "deliver_object_pose('"
                                       side
                                       "',[TRANSLATION, ROTATION])")
                          :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query didn't deliver_object_pose reach any solution."))
      (t (values-list `(,(list (cdr (assoc '?translation (cut:lazy-car raw-response)))
                               (cdr (assoc '?rotation (cut:lazy-car raw-response))))
                        ,(string-trim "'" (cdr (assoc '?context (cut:lazy-car raw-response))))))))))

;; used in tableclean
(defun prolog-surface-from-urdf (urdf-name)
  "Receives urdf name `urdf-name'. Asks Knowledge for the surface name by probiving `urdf-name'"
  (let ((raw-response (with-safe-prolog
                        (json-prolog:prolog-simple
                         (concatenate 'string
                                      "has_urdf_name(OBJECT,'"
                                      urdf-name
                                      "')")
                         :package :llif))))
    (cond
      ((eq raw-response 1) (roslisp:ros-warn (knowledge-surface-client)
                                             "Query didn't deliver_object_pose reach any solution."))
      (t (remove-string +HSR-SURFACE-PREFIX+
                        (string-trim "'" (cdr (assoc '?object (cut:lazy-car raw-response)))))))))

(defun prolog-add-default-surface (object-class surface)
  "eceives object class `object-class' and surface `surface'. Adds `surface' as the default surface as `object-class' in Knowledge."
  (let ((raw-response (with-safe-prolog
                        (json-prolog:prolog-simple
                         (concatenate 'string
                                      "assertz(default_surface('"
                                      +hsr-objects-prefix+
                                      object-class
                                      "', '"
                                      +hsr-rooms-prefix+
                                      surface
                                      "'))")
                         :package :llif))))
    (when (eq raw-response 1)
       (roslisp:ros-error (knowledge-surface-client)
                          "Query didn't default_surface ran into an error."))))

;; TODO
(defun prolog-get-surface-type (surface)
  "Receives surface `surface'. Asks Knowledge for the surface-type of `surface'."
  (let ((raw-response (with-safe-prolog
                        (json-prolog:prolog-simple
                         (concatenate 'string
                                      "has_type(OBJECT, hsr_rooms:'"
                                      surface
                                      "')")
                         :package :llif))))
    (when (eq raw-response 1)
       (roslisp:ros-error (knowledge-surface-client)
                          "Query didn't has_type ran into an error."))
    "drawer"))

;;@author Felix Krause
(defun prolog-get-dimensions-width (surface)
  "Receives surface `surface'. Asks Knowledge for the width of `surface'."
  (let ((raw-response (with-safe-prolog
                        (json-prolog:prolog-simple
                         (concatenate 'string
                                      "surface_dimensions('"
                                      +hsr-surface-prefix+
                                      surface
                                      "', _, WIDTH, _)")
                         :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-error (knowledge-surface-client)
                           "Query didn't reach a solution. Surface_dimensions ran into an error.")
        (cdr (assoc '?width (cut:lazy-car raw-response))))))
