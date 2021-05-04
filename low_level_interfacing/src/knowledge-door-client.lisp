(in-package :llif)

;; @author Jan Schimpf
(defun prolog-get-all-door-states()
  "Gives back an list of the door names and the state"
  (roslisp:ros-info (json-prolog-client) "Getting all doors and their states")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple "get_all_door_states(POSES)"  :package :llif))))
      (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query didn't reach any solution.")
               (cdr (car (cut:lazy-car raw-response))))))

;; @author Jan Schimpf
(defun prolog-update-door-state(door-id angle)
  "updates the state of the door and the angle that the door was closed/opened at"
  (roslisp:ros-info (json-prolog-client) "Updating the door state")
  (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple
                          (concatenate 'string "update_door_state_dynamic('"door-id"','"angle"')")))))
        (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query didn't reach any solution.")
            (cut:lazy-car raw-response))))

;; @author Jan Schimpg
(defun prolog-get-angle-to-open-door (door-id)
  "returns the angle that is used to open the door"
 (let* ((raw-response (with-safe-prolog
                         (json-prolog:prolog-simple  (concatenate 'string "get_door_state('"
                                       door-id "', ANGLE)")  :package :llif))))
        (if (eq raw-response 1)
            (roslisp:ros-warn (json-prolog-client)
                              "Query didn't reach any solution.")
           (cdr (car (cut:lazy-car raw-response))))))
