(in-package :llif)

;; @author Jan Schimpg
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
             (cdr (assoc '?Pose (cut:lazy-car raw-response))))))

;; @author Jan Schimpf
(defun prolog-manipulating-pose-of-door(door-id)
  "returns the pose from which we can manipulate the door"
  (let* ((knowrob-name  (concatenate 'string "'" (format nil "~a~a" +HSR-ROOMS-PREFIX+ door-id) "'" ))
        
         (raw-response (with-safe-prolog
                        (json-prolog:prolog-simple
                         (concatenate 'string "manipulating_pose_of_door("
                                      knowrob-name
                                      ", POSE)")  :package :llif))))
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
                            start-room-id ", " 
                            target-room-id "', DOORS)")  :package :llif))))
        (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query shortest_path_between_rooms didn't reach any solution.")
           (cdr (car (cut:lazy-car raw-response))))))
