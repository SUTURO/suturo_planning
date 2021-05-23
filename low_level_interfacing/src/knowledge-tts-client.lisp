(in-package :llif)

;; @author Torge Olliges
(defun prolog-perceived-object->object-id (perceived-object room-id)
(let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "matching_object_in_room_id('"
                                       perceived-object
                                       "','"
                                       room-id
                                       "',OBJECT)")
                          :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-surface-client)
                          "Query didn't surface_in_room reach any solution.")
        (remove-string +HSR-OBJECT-PREFIX+ 
            (string-trim "'" (cdr (assoc '?Object (cut:lazy-car raw-response))))))))

;; @author Torge Olliges
(defun prolog-perceived-room->room-id (perceived-room)
(let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "matching_room('"
                                       knowrob-name
                                       "',ROOM)")
                          :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-surface-client)
                          "Query didn't surface_in_room reach any solution.")
        (cdr (assoc '?Room (cut:lazy-car raw-response))))))
