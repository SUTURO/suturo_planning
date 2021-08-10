(in-package :llif)

;; @author Jan Schimpg
;; Gets the id of the Door and turns that into a knowrob id.
;; Uses that knowrob id to query for the state of the door.
;; 0 for closed and 1 for open
(defun prolog-get-door-state(door-id)
  "Gives back the state of that door"
  (roslisp:ros-info (json-prolog-client) "Getting the state for the door")
  (let* ((knowrob-name (format nil "~a~a" +hsr-rooms-prefix+ door-id))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "get_door_state('"
                                       knowrob-name
                                       "',DOORSTATE).") :package :llif))))
    (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query didn't reach any solution.")
            (cdr (assoc '?Doorstate (cut:lazy-car raw-response) )))))

;; @author Jan Schimpf
;; Query to get a list of tuples, that contain the knowrob door id
;; and the state of the doors.
;; The knowrob id then has the hsr-room-prefix trimed away. 
(defun prolog-get-all-door-states()
  "Gives back an list of the door names and the state"
  (roslisp:ros-info (json-prolog-client) "Getting all doors and their states")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple "get_all_door_states(POSES)."  :package :llif))))
      (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query didn't reach any solution.")
           (mapcar
             (lambda (x)
               (list (string-trim +hsr-rooms-prefix+ (string-trim "'"(car x))) (car (cdr x))))
                (cdr (assoc '?Poses (cut:lazy-car raw-response)))))))

;; @author Jan Schimpf
;;Gets an door id and an angle.
;;The door id is turn into a knowrob id and then used with the angle to updats the
;;Knowledge base 
(defun prolog-update-door-state(door-id angle)
  "updates the state of the door and the angle that the door was closed/opened at"
  (roslisp:ros-info (json-prolog-client) "Updating the door state")
  (let* ((knowrob-name (format nil "~a~a" +hsr-rooms-prefix+ door-id))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "update_door_state_dynamic('"
                                       knowrob-name
                                       "',"
                                       angle").")))))
        (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query update_door_state_dynamic didn't reach any solution.")
            raw-response)))

;; @author Jan Schimpf
;; Gets an door id which is turned into an knowrob id
;; Then knowledge base is queried for the angle that is needed to open the door
(defun prolog-get-angle-to-open-door (door-id)
  "returns the angle that is used to open the door"
  (let* ((knowrob-name (format nil "~a~a" +hsr-rooms-prefix+ door-id))
         (raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "get_angle_to_open_door('"
                                       knowrob-name
                                       "', ANGLE)")  :package :llif))))
        (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query get_door_state didn't reach any solution.")
           (cdr (car (cut:lazy-car raw-response))))))

;; @author Jan Schimpf
;; Gets and door id which is for the query turned into an knowrob id
;; Returns a position in "front" of the door, this can either be used to perceive the door 
(defun prolog-perceiving-pose-of-door (door-id)
 "returns the pose needed to perceive the door"
  (let* ((knowrob-name (format nil "~a~a" +hsr-rooms-prefix+ door-id))
         (raw-response (with-safe-prolog
                        (json-prolog:prolog-simple
                         (concatenate 'string "perceiving_pose_of_door('"
                                      knowrob-name
                                      "', POSE)")  :package :llif))))
        (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query get_door_state didn't reach any solution.")
            (list (cdr (assoc '?Pose (cut:lazy-car raw-response)))
                  ))))

;;@author Jan Schimpf
(defun prolog-get-doorhandle (door-id)
  "returns the doorhandle knowrob name that is needed to open the door"
  (let* ((knowrob-name (format nil "~a~a" +hsr-rooms-prefix+ door-id))
         (raw-response (with-safe-prolog
                        (json-prolog:prolog-simple
                         (concatenate 'string "door_handle_to_open_door('"
                                      knowrob-name
                                      "', DOORHANDLE)")  :package :llif))))
    (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query get_door_state didn't reach any solution.")
             raw-response)))
                              ;; (string-trim +hsr-rooms-prefix+
                              ;; (string-trim "'" (cdr (assoc '?Doorhandle (cut:lazy-car raw-response)))))
;; @author Jan Schimpf
;; Gets and door id which is for the query turned into an knowrob id
;; Returns a position in front of the door handle that we want to use to open the door as well as the knowrob doorhandle id
(defun prolog-manipulating-pose-of-door(door-id)
  "returns the pose from which we can manipulate the door"
  (let* ((knowrob-name (format nil "~a~a" +HSR-ROOMS-PREFIX+ door-id))
         (raw-response (with-safe-prolog
                        (json-prolog:prolog-simple
                         (concatenate 'string "manipulating_pose_of_door('"
                                      knowrob-name
                                      "', POSE)")  :package :llif))))
        (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query get_door_state didn't reach any solution.")
            (cdr (assoc '?Pose (cut:lazy-car raw-response))))))

;; @author Torge Olliges
(defun prolog-shortest-path-between-rooms (start-room-id target-room-id)
  "returns door ids which are obstacles on the path frorm start room to target room"
 (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple  
                          (concatenate 'string 
                            "shortest_path_between_rooms('"
                            start-room-id "', '" 
                            target-room-id "', DOORS)")  :package :llif))))
        (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query shortest_path_between_rooms didn't reach any solution.")
           (cdr (car (cut:lazy-car raw-response))))))

;; @author Jan Schimpf
;; turns the door id into a knowrob id and then into the urdf link name which is needed
;; by manipulation
(defun prolog-knowrob-name-to-urdf-link(door-id)
  "returns the urdf link that has to do is linked to the knowrob name"
  (let* ((knowrob-name (format nil "~a~a" +HSR-ROOMS-PREFIX+ door-id))     
         (raw-response (with-safe-prolog
                        (json-prolog:prolog-simple
                         (concatenate 'string "has_urdf_name('"
                                      knowrob-name
                                      "',URDFNAME)")  :package :llif))))
        (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query get_door_state didn't reach any solution.")
          (string-trim "'" (cdr (assoc '?UrdfName (cut:lazy-car raw-response)))))))
