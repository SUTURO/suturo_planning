(in-package :llif)

;; @author Jan Schimpg
(defun prolog-get-door-state(door-id)
  "Gives back the state of that door"
  (roslisp:ros-info (json-prolog-client) "Getting all doors and their states")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "get_door_state('"
                                       door-id
                                       "',DOORSTATE).") :package :llif))))
    (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query didn't reach any solution.")
            (cdr (assoc '?DoorState   (cut:lazy-car raw-response))))))

;; @author Jan Schimpf
(defun prolog-get-all-door-states()
  "Gives back an list of the door names and the state"
  (roslisp:ros-info (json-prolog-client) "Getting all doors and their states")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple "get_all_door_states(POSES)."  :package :llif))))
      (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query didn't reach any solution.")
               (car (cdr (assoc '?Poses (cut:lazy-car raw-response)))))))

;; @author Jan Schimpf
(defun prolog-update-door-state(door-id angle)
  "updates the state of the door and the angle that the door was closed/opened at"
  (roslisp:ros-info (json-prolog-client) "Updating the door state")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "update_door_state_dynamic('"door-id"',"angle").")))))
        (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query update_door_state_dynamic didn't reach any solution.")
            raw-response)))

;; @author Jan Schimpf
(defun prolog-get-angle-to-open-door (door-id)
  "returns the angle that is used to open the door"
 (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple  (concatenate 'string "get_door_state('"
                                       door-id "', ANGLE)")  :package :llif))))
        (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query get_door_state didn't reach any solution.")
           (cdr (car (cut:lazy-car raw-response))))))

;; @author Jan Schimpf
(defun prolog-perceiving-pose-of-door (door-id)
 "returns the pose needed to perceive the door"
 (let* ((raw-response (with-safe-prolog
                        (json-prolog:prolog-simple
                         (concatenate 'string "perceiving_pose_of_door('"
                                      door-id
                                      "', POSE)")  :package :llif))))
        (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query get_door_state didn't reach any solution.")
             (cdr (assoc '?Pose (cut:lazy-car raw-response))))))

;; @author Jan Schimpf
(defun prolog-manipulating-pose-of-door(door-id)
  "returns the pose from which we can manipulate the door"
 (let* ((raw-response (with-safe-prolog
                        (json-prolog:prolog-simple
                         (concatenate 'string "manipulating_pose_of_door('"
                                      door-id
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
                            start-room-id ", " 
                            target-room-id "', DOORS)")  :package :llif))))
        (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query shortest_path_between_rooms didn't reach any solution.")
           (cdr (car (cut:lazy-car raw-response))))))
