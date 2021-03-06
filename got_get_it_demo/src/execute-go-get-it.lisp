(in-package :get-it)

;;@author Torge Olliges
(defun execute-go-get-it()
    (init-interfaces)
    (comf::with-hsr-process-modules

    ;; move to predefined location
    (move-to-start-position)
    (wait-for-orders)
        ))

;;@author Torge Olliges
(defun move-to-start-position()
    )

;;@author Torge Olliges
(defun wait-for-orders()
    (subscribe "fetch_request" "nlp_msgs/FetchRequest" #handle-fetch-request))

;;@author Torge Olliges
(defun handle-fetch-request (fetch-request)
    (roslisp::with-fields (perceived_object_name perceived_room_name)
        (let ((room-id (llif::prolog-perceived-room->room-id perceived_room_name))) 
        (if (eq room-id 1) 
            ((roslisp:ros-info (handle-fetch-request) "No room found for perceived name ~a" perceived_room_name)
             (wait-for-orders)) 
            ((let ((object-id (llif::prolog-perceived-object->object-id perceived_object_name room-id)))
                (if (eq object-id 1)
                    (find-object-in-room perceived_object_name room-id)
                    (retrieve-object-from-room object-id room-id))))))))

;;@author Torge Olliges
(defun find-object-in-room (perceived-object-name room-id)
    (comf::move-to-room room-id)
    (loop for surface-name in (llif::prolog-not-perceived-surfaces-in-room room-id)
        do (
            (comf::move-to-surface surface-name)
            (perceive-surface surface-name)
            ((let ((object-id (llif::prolog-perceived-object->object-id perceived_object_name room-id)))
                (if (eq object-id 1)
                    (roslisp:ros-info (find-object-in-room) "Object ~a wasn't on surface ~a" perceived_object_name surface-name)
                    (retrieve-object-from-room object-id room-id)))))))

;;@author Torge Olliges
(defun retreive-object-from-room (object-id room-id)
    (if (eq (llif::prolog-current-room) room-id)
        ()
        (comf::move-to-room room-id))
    (comf::move-to-surface (llif::prolog-object-supporting-surface object-id))
    (setf *graspmode* 1)  ;;sets the graspmode should be replaces with the function from knowledge when that is finished
    (setf *grasp-object-result* (comf::grasp-object *next-object* *graspmode*))
    ;;If it doesn't work again just stop trying
    (if (roslisp::with-fields (error_code) *grasp-object-result* (= error_code 0)) 
        (llif::call-text-to-speech-action "I have grasped the object") ;;replace with NLG command
        (if (< *grasping-retries* 3)
            (grasp-handling)
            (llif::call-text-to-speech-action  "I am not able to grasp any object could you please put the object into my hand?")))
    (comf::move-to-room *starting-room*))
    ;;TODO:: how to drop/dispose item


;;@author Torge Olliges
(defun perceive-surface (surface-name)
    (llif::call-text-to-speech-action 
        (llif::call-nlg-action-simple "action" "percieve"));;TODO fix when nlg fixed percieve -> perceive

    (if (search "table" surface-name)
        (perceive-table)
        (if (search "shelf" surface-name)
            (perceive-shelf)
            (roslisp:ros-info (perceive-surface) "Neither table nor shelf but: ~a" surface-name))))
    )

;;@author Torge Olliges
(defun perceive-table ()
    (llif::call-take-pose-action 2)
    (setf *perception-objects*  (llif::call-robosherlock-object-pipeline (vector "table") t))
    (llif::insert-knowledge-objects *perception-objects*)
    (comf:reachability-check *perception-objects*)
    (get-it::spawn-btr-objects *perception-objects*)
    (llif::call-text-to-speech-action "I am done perceiving the table now.") ;;TODO: replace with NLG command  [[action, "move"],[start_surface_id,"table"]] ... not sure how to say that we finished this
    (llif::call-take-pose-action 1))

;;@author Torge Olliges
(defun perceive-shelf () ;;TODO: fix this because... not dynamic but hardcoded
    ;; (llif::prolog-get-surface-regions)
    (llif::call-take-pose-action 2)
    (perceive-shelf "bookshelf_0")
    (perceive-shelf "bookshelf_1")
    (llif::call-take-pose-action 3)
    (perceive-shelf "bookshelf_2")
    (llif::call-take-pose-action 1))