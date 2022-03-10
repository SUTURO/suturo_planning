(in-package :go-get-it)

(defparameter *deliver-pose* nil)
(defparameter *fetch-fluent* (make-fluent :value nil))
(defparameter *deliver-fluent* (make-fluent :value nil))

;;used in go-get-it
;;@author Torge Olliges
(defun execute-go-get-it()
  (init-interfaces)
  
  (comf::with-hsr-process-modules
    (loop for room in (llif::prolog-all-rooms)
          do
             (loop for surface in (llif::prolog-room-surfaces room)
                   do
                        (when
                            (not
                             (eq
                              (search "Shelf" surface) nil))
                          (llif::prolog-set-surface-not-visited surface))))

    (print "Test")
    (llif::call-take-pose-action 7)
    (print "Changed pose.")
    (wait-for-orders)
    (print "Waiting for orders.")
    (whenever (*fetch-fluent*)
      (print "Fetch request recieved.")
      (handle-fetch-request *fetch-fluent*)
      (print "Fetch request handled.")
      (whenever (*deliver-fluent*)
        (print "Deliver request recieved.")
        (handle-deliver-request *deliver-fluent*)
	(print "Deliver request handled.")
        (comf::get-motion-des-going-for-doors (list (list 1.916 3.557 0) (list 0 0 0.714223615142 0.699917586272)) nil)))
    ;; move to predefined location
    ;; (move-to-start-position)
))

;;used in go-get-it
;;@author Torge Olliges
(defun wait-for-orders()
  (llif::call-take-pose-action 1)
  (subscribe "/fetch_request" "nlp_msgs/GoAndGetIt" #'set-fetch-fluent)
  (subscribe "/deliver_request" "nlp_msgs/GoAndGetIt" #'set-deliver-fluent))

;;used in go-get-it
(defun set-fetch-fluent(fetch-request)
  (roslisp::ros-info (set-fetch-fluent) "Setting fetch fluent to ~a" fetch-request)
  (setf (value *fetch-fluent*) fetch-request))

;;used in go-get-it
(defun set-deliver-fluent(deliver-request)
  (roslisp::ros-info (set-deliver-fluent) "Setting deliver fluent to ~a" deliver-request)
  (setf (value *deliver-fluent*) deliver-request))

;;used in go-get-it
;;@author Torge Olliges
(defun handle-fetch-request (fetch-request)
  (setf fetch-request (value fetch-request))
  (roslisp::ros-info (handle-fetch-request) "Handling fetch request: ~a" fetch-request)
  ;;(comf::with-hsr-process-modules
  (roslisp::with-fields (perceived_object_name)
      fetch-request
    (let ((room-id (llif::prolog-current-room))) 
      (let ((object-id
              (llif::prolog-perceived-object-in-room->object-id perceived_object_name room-id)))
        (roslisp::ros-info (handle-fetch-request)
                           "Object ~a not yet known" perceived_object_name)
        (if (eq object-id nil)
            (retrieve-object-from-room
             (find-object-in-room perceived_object_name room-id)
             room-id)
            (retrieve-object-from-room object-id room-id)))))
  (setf (values *fetch-fluent*) nil))

;;used in go-get-it
(defun handle-deliver-request (deliver-request)
  (setf deliver-request (value deliver-request))
  (roslisp::ros-info (handle-fetch-request) "Handling deliver request: ~a" deliver-request)
  ;;(comf::with-hsr-process-modules
  (roslisp::with-fields (person_left person_right)
      deliver-request
    (if person_left
        (if person_left
            (setf *deliver-pose*
                  (comf::prolog-object-goal-pose->pose-stamped 
                   (llif::prolog-deliver-object-pose "left")))
            (roslisp::ros-warn (handle-deliver-request) "Deliver pose not set to person left"))
        (if person_right
            (setf *deliver-pose*
                  (comf::prolog-object-goal-pose->pose-stamped 
                   (llif::prolog-deliver-object-pose "right")))
            (roslisp::ros-warn (handle-deliver-request) "Deliver pose not set to person right"))))
  (comf::move-hsr *deliver-pose*)
  (llif::call-take-pose-action 6)
  (setf (value *deliver-fluent*) nil))

;;used in go-get-it
;;@author Torge Olliges
(defun find-object-in-room (perceived_object_name room-id)
  (print "find object in room")
  ;;(comf::move-to-room room-id)
  ;;(loop for surface-info in (llif::sort-surfaces-by-distance
  ;;                           (llif::prolog-room-surfaces (llif::prolog-current-room)))
  ;;                            (llif::prolog-surfaces-not-visited-in-room room-id))
  (loop for surface-info in (comf::sort-surfaces-by-distance
                                        (llif::prolog-surfaces-not-visited-in-room
                                         room-id))
        do
           ;;(when (and
           ;;       (eq (search "Table" (car surface-info)) nil)
           ;;       (eq (search "Kitchen" (car surface-info)) nil)
           ;;       (eq (search "bucket"
           ;;                   (llif::prolog-surface-region (car surface-info))) nil)
           ;;       (eq (search "chair"
           ;;                   (llif::prolog-surface-region (car surface-info))) nil))
           (comf::move-to-surface (car surface-info) t)
           (comf::perceive-surface (car surface-info))
           (let (;;(furniture-id (llif::prolog-surface-furniture (car surface-info)))
                 (object-id
                   (llif::prolog-perceived-object-in-room->object-id
                    perceived_object_name
                    (llif::prolog-current-room))))
           (roslisp::ros-info (find-object-in-room) "Trying to find ~a in room ~a" object-id room-id)
           (when object-id
             (roslisp::ros-info (find-object-in-room) "Object ~a found as ~a" perceived_object_name object-id)
             (return-from find-object-in-room object-id))
           
           (roslisp:ros-info (find-object-in-room) 
                             "Object ~a wasn't on surface ~a" perceived_object_name (car surface-info))
           (roslisp::ros-warn (find-object-in-room)
                              "Object ~a wasn't found on any surface in room ~a" perceived_object_name room-id)
           (return-from find-object-in-room (llif::prolog-next-object)))))

;;used in go-get-it
;;@author Torge Olliges
(defun retrieve-object-from-room (object-id room-id)
    ;;(or (eq (llif::prolog-current-room) room-id)
        ;;(comf::move-to-room room-id))
    (comf::move-to-surface (llif::prolog-object-source object-id) nil)
    (comf::grasp-handling object-id))
