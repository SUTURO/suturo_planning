(in-package :go-get-it)

(defparameter *starting-position* nil)
(defparameter *starting-room* nil)

;;@author Torge Olliges
(defun execute-go-get-it()
    ;;(init-interfaces)
    ;;(comf::with-hsr-process-modules

    ;; move to predefined location
    ;;(move-to-start-position)
    (wait-for-orders))

;;@author Torge Olliges
(defun move-to-start-position()
  (setf *starting-position* (cl-tf2::transform-stamped->pose-stamped
                             (cl-tf2::lookup-transform cram-tf:*transformer* "map" "base_footprint")))
  (setf *starting-room* (llif::prolog-current-room)))

;;@author Torge Olliges
(defun wait-for-orders()
  (llif::call-take-pose-action 1)
    (subscribe "/fetch_request" "nlp_msgs/FetchRequest" #'handle-fetch-request))

;;@author Torge Olliges
(defun handle-fetch-request (fetch-request)
  (comf::with-hsr-process-modules
  (print "####################################################################")
  (roslisp::with-fields (perceived_object_name perceived_room_name)
      fetch-request
    (let ((room-id (llif::prolog-perceived-room->room-id perceived_room_name))) 
      (when (eq room-id 1) 
        (roslisp:ros-info (handle-fetch-request)
                          "No room found for perceived room name ~a" perceived_room_name)
        (return-from handle-fetch-request nil))
      (let ((object-id (llif::prolog-perceived-object->object-id perceived_object_name room-id)))

        (when (eq object-id 1)
          (roslisp::ros-info (handle-fetch-request)
                             "Object ~a not yet known" perceived_object_name)
          (find-object-in-room perceived_object_name room-id))
        
        (retrieve-object-from-room (llif::prolog-perceived-object->object-id
                                    perceived_object_name room-id)
                                   room-id))))))

;;@author Torge Olliges
(defun find-object-in-room (perceived_object_name room-id)
    (comf::move-to-room room-id)
  (loop for surface-info in (llif::sort-surfaces-by-distance
                             (llif::prolog-surfaces-not-visited))
    ;;                            (llif::prolog-surfaces-not-visited-in-room room-id))
        do 
            (comf::move-to-surface (car surface-info) t)
            (comf::perceive-surface (car surface-info))
            (let ((object-id (llif::prolog-perceived-object->object-id perceived_object_name room-id)))
              (when object-id
                (when (/= object-id 1)
                  (roslisp::ros-info (find-object-in-room) "Object ~a found as ~a" perceived_object_name object-id)
                  (return-from find-object-in-room nil)))
              (roslisp:ros-info (find-object-in-room) 
                                "Object ~a wasn't on surface ~a" perceived_object_name (car surface-info))))
  (roslisp::ros-warn (find-object-in-room) "Object ~a wasn't found on any surface in room ~a" perceived_object_name room-id))

;;@author Torge Olliges
(defun retrieve-object-from-room (object-id room-id)
    (or (eq (llif::prolog-current-room) room-id)
        (comf::move-to-room room-id))
    (comf::move-to-surface (llif::prolog-object-source object-id) nil)
    (setf grasp-mode 1)  ;;sets the graspmode should be replaces with the function from knowledge when that is finished
    (comf::grasp-handling object-id)
    (comf::move-to-room *starting-room*)
    (comf::move-hsr *starting-position*)
    (llif::call-take-pose-action 6))
