(in-package :llif)

;; @author Torge Olliges
(defun prolog-surface-pose (surface-name)
  "returns the pose of the table"
  (roslisp:ros-info (json-prolog-client) "Getting pose from table(")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple 
                          (concatenate 'string 
                                       "surface_pose_in_map("
                                       surface-name
                                       ", [TRANSLATION, ROTATION])")
                          :package :llif))))
     (print raw-response)
     (if (eq raw-response 1)
        (roslisp:ros-warn (json-prolog-client)
                          "Query didn't surface_pose_in_map reach any solution.")
        (values-list `(,(cdr (assoc '?translation (cut:lazy-car raw-response)))
                       ,(string-trim "'"
                                     (cdr
                                      (assoc '?context
                                             (cut:lazy-car raw-response)))))))))


;; @author Torge Olliges
(defun prolog-shelfs ()
  "returns the pose of the shelf"
  (roslisp:ros-info (json-prolog-client) "Getting pose for shelf")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "shelf_surfaces(POSITIONS)")
                                                    :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (json-prolog-client)
                          "Query didn't shelf_surfaces reach any solution.")
        (values-list (list (mapcar
                      (lambda (x) (string-trim "'" x))
                      (cdr (assoc '?Positions (cut:lazy-car raw-response)))))))))

;; @author Torge Olliges
(defun prolog-tables ()
  "returns the pose of the shelf"
  (roslisp:ros-info (json-prolog-client) "Getting pose for shelf")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "table_surfaces(POSITIONS)")
                                                    :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (json-prolog-client)
                          "Query didn't shelf_surfaces reach any solution.")
        (values-list (list (mapcar
                      (lambda (x) (string-trim "'" x))
                      (cdr (assoc '?Positions (cut:lazy-car raw-response)))))))))

;; @author Torge Olliges
(defun prolog-buckets ()
  "returns the bucket names"
  (roslisp:ros-info (json-prolog-client) "Getting bucket names")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "bucket_surfaces(POSITIONS)")
                          :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (json-prolog-client)
                          "Query didn't shelf_surfaces reach any solution.")
        (values-list (list (mapcar
                      (lambda (x) (string-trim "'" x))
                      (cdr (assoc '?Positions (cut:lazy-car raw-response)))))))))

;; @author Torge Olliges
(defun sort-by-distance (surface-names)
  (sort
   (mapcar
    (lambda (surface-name) (list
                            surface-name
                            (current-pose-prolog-pose->distance
                             (prolog-surface-pose surface-name))))
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

;; @author Torge Olliges
(defun prolog-get-surface-regions (surface-id)
)

;; @author Torge Olliges
(defun prolog-get-surface-room (surface-id)
)

;; @author Torge Olliges
(defun prolog-current-room ()
)

;; @author Torge Olliges
(defun prolog-get-room-to-room-obstacles (room-start room-goal)
  ;;  liste obstacles: (obstacle (tür, position))
)
