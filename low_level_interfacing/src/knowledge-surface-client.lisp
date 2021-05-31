(in-package :llif)

;; @author Torge Olliges
(defun prolog-surface-pose (surface-name)
  "returns the pose of the table"
  (roslisp:ros-info (knowledge-surface-client) "Getting pose of surface ~a" surface-name)
  (let* ((knowrob-name (format nil "~a~a" +HSR-SURFACE-PREFIX+ surface-name))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          (concatenate 'string 
                                       "surface_center_pose('"
                                       knowrob-name
                                       "',[TRANSLATION, ROTATION])")
                          :package :llif))))
     (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-surface-client)
                          "Query didn't surface_pose_in_map reach any solution.")
        (values-list `(,(list (cdr (assoc '?translation (cut:lazy-car raw-response)))
                              (cdr (assoc '?rotation (cut:lazy-car raw-response))))
                       ,(string-trim "'"
                                     (cdr
                                      (assoc '?context
                                             (cut:lazy-car raw-response)))))))))

(defun prolog-surface-front-edge-pose (surface-name)
  "returns the pose of the front edge of a surface"
  (roslisp::ros-info (knowledge-surface-client) "Getting front edge pose for surface ~a" surface-name)
  (let*  ((knowrob-name (format nil "~a~a" +HSR-SURFACE-PREFIX+ surface-name))
          (raw-response (with-safe-prolog
                           (json-prolog:prolog-simple
                            (concatenate 'string
                                         "surface_front_edge_center_pose('"
                                         knowrob-name
                                         "', [TRANSLATION, ROTATION])")
                            :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-surface-client)
                          "Query didn't surface_pose_in_map reach any solution.")
        (values-list `(,(list (cdr (assoc '?translation (cut:lazy-car raw-response)))
                              (cdr (assoc '?rotation (cut:lazy-car raw-response))))
                       ,(string-trim "'"
                                     (cdr
                                      (assoc '?context
                                             (cut:lazy-car raw-response)))))))))
                                              

;; @author Torge Olliges
;; (defun prolog-shelfs ()
;;   "returns the names of all available shelfs"
;;   (roslisp:ros-info (knowledge-surface-client) "Getting shelfs")
;;   (let* ((raw-response (with-safe-prolog
;;                          (json-prolog:prolog-simple
;;                           (concatenate 'string
;;                                        "shelf_surfaces(POSITIONS)")
;;                                                     :package :llif))))
;;     (if (eq raw-response 1)
;;         (roslisp:ros-warn (json-prolog-client)
;;                           "Query didn't shelf_surfaces reach any solution.")
;;         (values-list (list (mapcar
;;                       (lambda (x) (string-trim "'" x))
;;                       (cdr (assoc '?Positions (cut:lazy-car raw-response)))))))))

;; @author Torge Olliges
;; (defun prolog-tables ()
;;   "returns the names of all available tables"
;;   (roslisp:ros-info (knowledge-surface-client) "Getting pose for tables")
;;   (let* ((raw-response (with-safe-prolog
;;                          (json-prolog:prolog-simple
;;                           (concatenate 'string
;;                                        "table_surfaces(POSITIONS)")
;;                                                     :package :llif))))
;;     (if (eq raw-response 1)
;;         (roslisp:ros-warn (knowledge-surface-client)
;;                           "Query didn't shelf_surfaces reach any solution.")
;;         (values-list (list (mapcar
;;                       (lambda (x) (string-trim "'" x))
;;                       (cdr (assoc '?Positions (cut:lazy-car raw-response)))))))))

;; @author Torge Olliges
;; (defun prolog-buckets ()
;;   "returns the bucket names"
;;   (roslisp:ros-info (knowledge-surface-client) "Getting bucket names")
;;   (let* ((raw-response (with-safe-prolog
;;                          (json-prolog:prolog-simple
;;                           (concatenate 'string
;;                                        "bucket_surfaces(POSITIONS)")
;;                           :package :llif))))
;;     (if (eq raw-response 1)
;;         (roslisp:ros-warn (knowledge-surface-client)
;;                           "Query didn't shelf_surfaces reach any solution.")
;;         (values-list (list (mapcar
;;                       (lambda (x) (string-trim "'" x))
;;                       (cdr (assoc '?Positions (cut:lazy-car raw-response)))))))))

;; @author Torge Olliges
(defun sort-surfaces-by-distance (surface-names)
  (sort
   (mapcar
    (lambda (surface-name) (list
                            surface-name
                            (current-pose-prolog-pose->distance
                             (first (prolog-surface-pose surface-name)))))
    surface-names)
   #'tuple-compare))

;; @author Torge Olliges
(defun tuple-compare (l r)
  (< (nth 1 l) (nth 1 r)))

;; @author Torge Olliges                             
(defun current-pose-prolog-pose->distance (prolog-pose-values)
  (let ((current-pose-translation
          (cl-transforms::translation
           (cl-tf::lookup-transform cram-tf::*transformer* "map" "base_footprint")))
        (prolog-pose-vector (cl-tf2::make-3d-vector
                      (first prolog-pose-values)
                      (second prolog-pose-values)
                      (third prolog-pose-values))))
    (cl-transforms:v-dist current-pose-translation prolog-pose-vector)))

;; @author Torge Ollige
(defun prolog-set-surfaces-visit-state (surface-id)
  (let* ((raw-response (with-safe-prolog
                          (json-prolog:prolog-simple
                            (concatenate 'string
                                        "update_visit_state('"
                                        surface-id
                                        "')")
                            :package :llif))))
      (if (eq raw-response 1)
          (roslisp:ros-warn (knowledge-surface-client)
                            "Query didn't assign_visit_state reach any solution.")
          (roslisp:ros-info (Knowledge-surface-client) "Visit state for object ~a was updated" surface-id))))

;; @author Torge Ollige
(defun prolog-surfaces-not-visited ()
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "surfaces_not_visited(SURFACES)")
                          :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-surface-client)
                          "Query didn't surfaces_not_visited reach any solution.")
        (values-list (list (mapcar
                            (lambda (x) (remove-string +HSR-SURFACE-PREFIX+ (string-trim "'" x)))
                      (cdr (assoc '?Surfaces (cut:lazy-car raw-response)))))))))

;; @author Torge Ollige
(defun prolog-surfaces-not-visited-in-room (room-id)
  (let* ((knowrob-name (format nil "~a~a" +HSR-ROOMS-PREFIX+ room-id))
         (raw-response (with-safe-prolog
                          (json-prolog:prolog-simple
                            (concatenate 'string
                                        "surfaces_not_visited('"
                                        knowrob-name
                                        "',SURFACES)")
                            :package :llif))))
      (if (eq raw-response 1)
          (roslisp:ros-warn (knowledge-surface-client)
                            "Query didn't shelf_surfaces reach any solution.")
          (values-list (list (mapcar
                        (lambda (x) (remove-string +HSR-SURFACE-PREFIX+ (string-trim "'" x)))
                        (cdr (assoc '?Surfaces (cut:lazy-car raw-response)))))))))

;; @author Torge Olliges
(defun prolog-current-room ()
  (let* ((raw-response (with-safe-prolog
                           (json-prolog:prolog-simple
                           (concatenate 'string
                                        "robot_in_room(ROOM)")
                           :package :llif))))
     (if (eq raw-response 1)
         (roslisp:ros-warn (knowledge-surface-client)
                           "Query didn't robot_in_room reach any solution.")
         (remove-string +HSR-ROOMS-PREFIX+
                        (string-trim "'" (cdr (assoc '?Room (cut:lazy-car raw-response))))))))

;; @author Torge Olliges
(defun prolog-room-surfaces (room)
  (let* ((knowrob-name (format nil "~a~a" +HSR-ROOMS-PREFIX+ room))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "surfaces_in_room('"
                                       knowrob-name
                                       "',SURFACES)")
                          :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-surface-client)
                          "Query didn't surfaces_in_room reach any solution.")
        (values-list (list (mapcar
                            (lambda (x) (remove-string +HSR-SURFACE-PREFIX+ (string-trim "'" x)))
                            (cdr (assoc '?Surfaces (cut:lazy-car raw-response)))))))))

(defun prolog-surface-room (surface-id)
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "surface_in_room('"
                                       surface-id
                                       "',ROOM)")
                          :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-surface-client)
                          "Query didn't surface_in_room reach any solution.")
        (values-list (list (mapcar
                            (lambda (x) (string-trim "'" x))
                            (cdr (assoc '?Positions (cut:lazy-car raw-response)))))))))


(defun prolog-surface-region (surface-id)
  (let* ((knowrob-name (format nil "~a~a" +HSR-SURFACE-PREFIX+ surface-id))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "get_perception_surface_region('"
                                       knowrob-name
                                       "',REGION)")
                          :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-surface-client)
                          "Query didn't surface_in_room reach any solution.")
        (string-trim "\""(string-trim "'" (cdr (assoc '?Region (cut:lazy-car raw-response))))))))


(defun remove-string (rem-string full-string &key from-end (test #'eql)
                      test-not (start1 0) end1 (start2 0) end2 key)
  "returns full-string with rem-string removed"
  (let ((subst-point (search rem-string full-string 
                             :from-end from-end
                             :test test :test-not test-not
                             :start1 start1 :end1 end1
                             :start2 start2 :end2 end2 :key key)))
    (if subst-point
        (concatenate 'string
                     (subseq full-string 0 subst-point)
                     (subseq full-string (+ subst-point (length rem-string))))
        full-string)))
