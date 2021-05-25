(in-package :llif)

;; @author Torge Olliges
(defun prolog-perceived-object->object-id (perceived-object room-id)
  (roslisp:ros-info (knowledge-tts-client)
                    "Getting object id for perceived object ~a in room ~a" perceived-object room-id)
  (let* ((room-knowrob-name (format nil "~a~a" +HSR-ROOMS-PREFIX+ room-id))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "matching_object_in_room_id(\""
                                       perceived-object
                                       "\",'"
                                       room-knowrob-name
                                       "',OBJECT)")
                          :package :llif))))
    
    (if raw-response
        (remove-string +HSR-OBJECTS-PREFIX+ 
            (string-trim "'" (cdr (assoc '?Object (cut:lazy-car raw-response)))))
        (roslisp:ros-warn (knowledge-surface-client)
                          "Query didn't surface_in_room reach any solution."))))

;; @author Torge Olliges
(defun prolog-perceived-room->room-id (perceived-room)
  (roslisp:ros-info (knowledge-tts-client) "Getting room id for perceived room ~a" perceived-room)
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string
                                       "matching_room(\""
                                       perceived-room
                                       "\",ROOM)")
                          :package :llif))))
    (if (eq raw-response 1)
        (roslisp:ros-warn (knowledge-surface-client)
                          "Query didn't surface_in_room reach any solution.")
        (remove-string +HSR-ROOMS-PREFIX+ (string-trim "'" (cdr (assoc '?Room (cut:lazy-car raw-response))))))))
