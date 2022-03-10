(in-package :clean)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;execute-cleanup.lisp;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;(defun robocup-floor-check ()
;;  (let ((nav-pose (cl-tf2::make-pose-stamped
;;                   "map"
;;                   0
;;                   (cl-tf2::make-3d-vector 0.3567 0.744 0)
;;                   (cl-tf2::q* (cl-tf2::make-quaternion 0 0 0 1)
;;                      (cl-tf2::euler->quaternion
;;                       :ax 0
;;                       :ay 0
;;                       :az (/ pi 1.5))))) dem 
;;        (nav-pose-grasp (cl-tf2::make-pose-stamped
;;                         "map"
;;                         0
;;                         (cl-tf2::make-3d-vector 0.3567 0.744 0)
;;                         (cl-tf2::q* (cl-tf2::make-quaternion 0 0 0 1)
;;                                     (cl-tf2::euler->quaternion
;;                                      :ax 0
;;                                      :ay 0
;;                                      :az 0.75))))
;;        (look-at-pose (llif::prolog-surface-pose (first (llif::prolog-cleanup-surfaces)))))
;;    (comf::move-hsr nav-pose)
;;    (llif::call-take-gaze-pose-action
;;     :px (first (first look-at-pose))
;;     :py (second (first look-at-pose))
;;     :pz 0)
;;    (let ((detected-objects (llif::call-robosherlock-object-pipeline (vector "robocup_default") t)))
;;      (llif::insert-knowledge-objects (comf::get-confident-objects detected-objects))
;;      (llif::call-take-pose-action 1)
;;      (comf::move-hsr nav-pose-grasp)
;;      (handle-detected-objects))))

;;@author Philipp Klein
;; (defun poi-search ()
;;   "approaching the next poi and bring the found object to the basket. When no poi is found, start the poi-search and loop the behavior."
;;   (comf::move-hsr (cl-tf::make-pose-stamped "map" 0 (cl-tf::make-3d-vector 1 0.5 0) (cl-tf::make-quaternion 0 0 0 1)))
;;   (loop do
;;     (block continue
;;       ;;(llif::call-text-to-speech-action "I have found a point of interest to search.") ;;replace with NLG command
;;       (let ((poi-pos (comf::move-to-poi)))
;;       (if (not poi-pos)
;;           (progn
;;             (comf::move-hsr (llif::find-biggest-unsearched-space T))
;;             (return-from continue)))

;;       ;;(comf::announce-perceive-action "future")
        
;;       (let ((confident-objects
;;             (comf::get-confident-objects
;;              (llif::call-robosherlock-object-pipeline (vector "robocup_default") t)
;;              0.8)))

 
;;       ;;percieve -> filter -> insert into knowledge
;;         (llif::call-take-pose-action 1)

;;         (roslisp::with-fields (detectiondata)
;;           confident-objects
;;         (progn
;;           (roslisp::ros-info (poi-search) "Number of objects detected: ~a" (length detectiondata))
;;           (if (> (length detectiondata) 0)
;;               (llif::insert-knowledge-objects confident-objects)
;;               (progn
;;                 (llif::poi-remover (cl-tf::origin poi-pos) 0.4)
;;                 (return-from continue)))))


;;         (let ((next-object (llif::prolog-next-object)))
;;           (when (eq next-object 1)
;;             (progn(llif::poi-remover (cl-tf::origin poi-pos) 0.4) (return-from continue)))
          
;;       ;; turn to face the object
;;       (roslisp::with-fields (translation rotation)
;;         (cl-tf::lookup-transform cram-tf::*transformer* "map" "base_footprint")
;;         (llif::call-nav-action-ps 
;;             (cl-tf::make-pose-stamped "map" 0 translation
;;                 (cl-tf::q* rotation
;;                            (cl-tf::euler->quaternion :ax 0 :ay 0 :az (- (/ pi 2)))))))
      
;;         (let ((object-goal (llif::prolog-object-goal next-object)))
;;           (comf::announce-grasp-action "future" next-object)
;;           (llif::call-take-pose-action 1)
          
;;           (comf::grasp-handling next-object)
;;           (comf::announce-movement  "future")
;;           (comf::move-to-surface object-goal nil)
          
;;           ;;place object at goal surface
;;           (comf::announce-place-action "present" next-object)
          
;;           (comf::place-object next-object 1)
;;           (comf::announce-place-action "past" next-object)

;;           (llif::call-take-pose-action 1))))))))





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;high-level-plans.lisp;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Navigation ;;

;;@author Torge Olliges, Phillip Klein
;;Tries a list of stamped poses in the bulletworld simulation
;;returns a possible pose.
;; (defun try-movement-stamped-list (listStamped)
;;  ;;  (car listStamped))
;;    (let ((?nav-pose listStamped))
;;      (cpl:with-retry-counters ((going-retry 3))
;;        (cpl:with-failure-handling
;;            (((or common-fail:low-level-failure 
;;                  cl::simple-error
;;                  cl::simple-type-error)
;;                 (e)
;;               (setf ?nav-pose (cdr ?nav-pose))
;;               (setf listStamped (cdr ?nav-pose))
;;               (cpl:do-retry going-retry
;;                 (roslisp:ros-warn 
;;                  (going-demo movement-fail) "~%Failed to move to given position~%")
;;                 (cpl:retry))
;;               (roslisp:ros-warn  (going-demo movement-fail) "~%No more retries~%")))
;;          (let ((?actual-nav-pose (car ?nav-pose))) 
;;            (exe:perform
;;             (desig:a motion
;;                       (type going)
;;                       (pose ?actual-nav-pose)))
;;            (setf listStamped (cdr ?nav-pose))
;;            ?actual-nav-pose)))))

;;@author Philipp Klein
;; (defun move-to-poi ()
;;   "moves the robot to the closest poi point"
;;   (roslisp:ros-info (move-to-poi) "Move to point of interest started")
;;   (let ((poi-list (llif::sorted-poi-by-distance
;;                    (roslisp::with-fields (translation)
;;                        (cl-tf::lookup-transform
;;                         cram-tf::*transformer*
;;                         "map"
;;                         "base_footprint")
;;                      translation))))
;;     (when 
;;         (not poi-list) 
;;       (return-from move-to-poi Nil))
;;     ;;(roslisp:ros-info (move-to-poi) "Poi with positions called ~a" poi-list)
;;     (pose-with-distance-to-points *poi-distance-threshold* poi-list 10 t)))

;;@author Philipp Klein
;; (defun move-to-poi-and-scan ()
;;   "moves the robot to a poi and perceives it"
;;   (move-to-poi)
;;   (llif::call-take-pose-action 1)
;;   (roslisp::with-fields
;;       (translation rotation)
;;       (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint")
;;     (llif::call-nav-action-ps 
;;      (cl-tf::make-pose-stamped
;;       "map"
;;       0
;;       translation 
;;       (cl-tf::q* rotation 
;;                  (cl-tf::euler->quaternion :ax 0 :ay 0 :az (/ pi 2)))))))

;;@author Jan Schimpf
;;Gets the door id as input and move into position to open it, currently only for one door as the position query isn't done yet
;; (defun open-room-door (door-id)
;;  (cpl:with-retry-counters ((grasping-retry 2))
;;     (cpl:with-failure-handling
;;         (((or common-fail:low-level-failure 
;;               cl::simple-error
;;               cl::simple-type-error)
;;              (e)
;;            (roslisp:ros-info (open-door) "Retry if opening the door failed")
;;            ;;insert here failure handling new position / retry / perception retry
;;            (cpl:do-retry grasping-retry
;;              (roslisp:ros-warn (grasp-fail) "~%Failed to grasp the object~%")
;;              (cpl:retry))
;;            (comf::get-nav-pose-for-doors (llif::prolog-manipulating-pose-of-door door-id) t)
;;            (roslisp:ros-warn  (going-demo movement-fail) "~%No more retries~%")))
;;       (let ((knowledge-pose-manipulation (llif::prolog-manipulating-pose-of-door door-id))) ;;get position to move the robot to
;;         (comf::get-motion-des-going-for-doors knowledge-pose-manipulation t) ;;execution of the move   
;;         (let ((knowledge-doorhandle-id 
;;                 (concatenate 'string
;;                              "iai_kitchen/"
;;                              (llif::prolog-knowrob-name-to-urdf-link ;;changes the knowrob id to the urdf link
;;                               (car (cdr (llif::prolog-perceiving-pose-of-door door-id)))))) ;; get the knowrob id for the door handle
;;               (knowledge-pose-perceiving   (car (llif::prolog-perceiving-pose-of-door door-id))) 
;;               (knowledge-open-door-angle (llif::prolog-get-angle-to-open-door door-id)))
;;           (llif::call-open-action knowledge-doorhandle-id
;;                                   knowledge-doorhandle-id
;;                                   knowledge-open-door-angle)
;;           (llif::prolog-update-door-state door-id knowledge-open-door-angle) ;; update the door state
;;           (comf::get-motion-des-going-for-doors knowledge-pose-manipulation t)))))) ;; get into a position from which the robot can move through the open door

;;@author Jan Schimpf
;; Function that is work in progress and is intended to navigate the robot between rooms
;; and open door to reach the goal if needed.
;; (defun open-door-on-path (start-room target-room)
;;   (loop for door-id in (llif::prolog-shortest-path-between-rooms start-room target-room)
;;         do
;;            (comf::open-room-door door-id)))

;;@author Torge Olliges
;; (defun move-to-room (room-id)
;;   (roslisp::ros-info (move-to-room) "Starting movement to room ~a" room-id)
;;   (announce-movement "present")
;;     (let ((shortest-path (llif::prolog-shortest-path-between-rooms
;;                            (llif::prolog-current-room) room-id)))
;;         (print shortest-path)
;;     ))






;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;knowledge-functions.lisp;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;@autho Torge Olliges
;; (defun reachability-check (object-ids)
;;   (every
;;    (lambda (x) (eq x T))
;;    (mapcar 
;;     (lambda 
;;      (object-id) 
;;      (if (eq 0 (reachability-check-grasp object-id 1))
;;        T  
;;        (if (eq 0 (reachability-check-grasp object-id 2))
;;          T
;;          (llif::set-object-not-graspable object-id 1))))
;;       object-ids)))

;;@autho Torge Olliges
;; (defun reachability-check-grasp (object-id grasp-mode)
;;     (let 
;;         ((make-plan-result 
;;             (llif::try-make-plan-action
;;                 (cl-tf2::transform-stamped->pose-stamped  
;;                     (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))
;;                 object-id
;;                 (prolog-object-goal-pose->pose-stamped (llif::prolog-object-goal-pose object-id))
;;                 (let ((dimensions (llif::prolog-object-dimensions object-id)))
;;                   (cl-tf2::make-3d-vector
;;                    (nth 0 dimensions)
;;                    (nth 1 dimensions)
;;                    (nth 2 dimensions)))
;;                 grasp-mode
;;                 98)))
;;       (roslisp:ros-info (reachability-check-grasp) "Reachability check result: ~a" make-plan-result)
;;         make-plan-result))

;;@autho Torge Olliges
;; (defun reachability-check-place (object-id grasp-mode)
;;   (let ((make-plan-result 
;;             (llif::try-make-plan-action
;;                 (cl-tf2::stamped-transform->pose-stamped  
;;                     (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))
;;                 object-id
;;                 (prolog-object-goal-pose->pose-stamped (llif::prolog-object-goal-pose object-id))
;;                 (let ((dimensions (llif::prolog-object-dimensions object-id)))
;;                   (cl-tf2::make-3d-vector (nth 0 dimensions) (nth 1 dimensions) (nth 2 dimensions)))
;;                 grasp-mode
;;                 99)))
;;     make-plan-result))

;;@author Torge Olliges, Jan Schimpf
;; (defun get-nav-pose-for-surface (surface-id)
;;   (let ((surface-pose (llif::prolog-surface-pose surface-id))
;;         (surface-edge-pose (llif::prolog-surface-front-edge-pose surface-id)))

;;     (cl-tf2::make-pose-stamped "map" 0
;;      (cl-tf2::make-3d-vector
;;      ;; adds the x value of the Vector to the edge to create an offset
;;       (+ (first (first surface-edge-pose))
;;         ;; creates the x value of the Vector from Center to Edge
;;         (* (- (first (first surface-edge-pose))
;;            (first (first surface-pose))) 1.75))

;;      ;; adds the y value of the Vector to the edge to create an offset
;;      (+ (second (first surface-edge-pose))
;;         ;; creates the y value of the Vector from Center to Edge
;;         (* (- (second (first surface-edge-pose))
;;            (second (first surface-pose))) 1.75))
;;      0)
;;     (cl-tf2::make-quaternion (first (second surface-edge-pose))
;;                              (second (second surface-edge-pose))
;;                              (third (second surface-edge-pose))
;;                              (fourth (second surface-edge-pose)))
;;     )))






;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;navigation-functions.lisp;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;author Philipp Klein
;; (defun points-around-point (distance point amountAlternatePositions turn)
;;   "return a given amount of points with a given distance around a list of points
;;   `distance' The distance, with which the points around the point are generated
;;   `point' The point from which the others are to be generated
;;   `amountAlternatePositions' The number of points to be generated
;;   `turn' whether the robot should be turned 90 degrees to the target at the end"
;;   (setf *currentOrigin*
;;         (cl-tf::origin
;;          (cl-tf::transform-stamped->pose-stamped
;;           (cl-tf::lookup-transform
;;            cram-tf::*transformer*
;;            "map" "base_footprint"))))
;;   (setf *goalOrigin* (cl-tf::origin point))
;;   (setf *newgoalOrigin* (cl-tf::v+ *goalOrigin* 
;;                                    (cl-tf::v* 
;;                                     (cl-tf::v- *currentOrigin* *goalOrigin*) 
;;                                     (/ distance
;;                                        (cl-tf::v-dist
;;                                         *currentOrigin*
;;                                         *goalOrigin*)))))
;;   (setf *radians*
;;         (mapcar (lambda
;;                     (listelem)
;;                   (*
;;                    (/
;;                     6.28319
;;                     amountAlternatePositions)
;;                    listelem)) 
;;                 (loop :for n :from 1 :below amountAlternatePositions :collect n)))
;; 	(setf *alternatePositions*
;;         (mapcar
;;          (lambda (listelem)
;;            (
;;             cl-tf:make-pose-stamped
;;             "map"
;;             (roslisp::ros-time)
;;             (cl-tf::make-3d-vector 
;;              (+
;;               (*
;;                distance
;;                (cos listelem))
;;               (cl-tf::x *goalOrigin*)) 
;;              (+
;;               (*
;;                distance
;;                (sin listelem))
;;               (cl-tf::y *goalOrigin*))
;;              0)
;;             (cl-tf:euler->quaternion
;;              :ax 0.0 :ay 0.0 :az
;;              (let*(    
;;                    (y
;;                      (-
;;                       (cl-tf::y *goalOrigin*)
;;                       (+
;;                        (*
;;                         distance
;;                         (sin listelem))
;;                        (cl-tf::y *goalOrigin*)))) 
;;                    (x
;;                      (-
;;                       (cl-tf::x *goalOrigin*)
;;                       (+
;;                        (*
;;                         distance
;;                         (cos listelem))
;;                        (cl-tf::x *goalOrigin*))))
;;                    (atanValue (atan (/ y x))))
;;                (if turn
;;                    (if (< x 0)
;;                        (-
;;                         atanValue 1.57)
;;                        (+
;;                         atanValue 1.57))
;;                    atanValue))))) 
;;          *radians*))
;;   (setf *alternatePositions*
;;         (llif::sorted-stamped-by-distance
;;          (roslisp::with-fields (translation)
;;              (cl-tf::lookup-transform
;;               cram-tf::*transformer*
;;               "map" "base_footprint")
;;            translation)
;;          *alternatePositions*)))

;;author Philipp Klein
;; (defun pose-with-distance-to-points (distance points number-alternate-positions turn)
;;   "move the roboter to a point with a given distance around the closest point
;;   `distance' The distance, with which the points around the point are generated
;;   `point' The point from which the others are to be generated
;;   `amountAlternatePositions' The number of points to be generated
;;   `turn' whether the robot should be turned 90 degrees to the target at the end"
;;   (let* (
;;         (positions-lists
;;              (mapcar
;;               (lambda (elem) (remove-if-not #'llif::robot-in-obstacle-stamped elem))
;;               (mapcar
;;                (lambda (elem) (remove-if #'llif::prolog-is-pose-outside-stamped elem))
;;                ;;(mapcar
;;                 ;;(lambda (elem) (remove-if-not #'llif::global-planner-reachable-from-current-pose elem))
;;               (mapcar 
;;                (lambda (elem) (points-around-point
;;                                distance elem  number-alternate-positions turn))
;;                points))));;)
;;          (poi-position (car points))
;;          (positions
;;            (flatten 
;;             (mapcar 
;;              (lambda (elem) (remove-if #'null elem)) positions-lists))))
;;     (roslisp:ros-info (poi) "points calculated, number of possible aproaches: ~a" (length positions))
;;     (publish-msg 
;;         (advertise "poi_positions" "geometry_msgs/PoseArray")
;;           :header
;;           (roslisp:make-msg "std_msgs/Header" (frame_id) "map" (stamp) (roslisp:ros-time))
;;           :poses
;;           (make-array (length positions)
;;                       :initial-contents
;;                       (mapcar #'cl-tf::to-msg
;;                               (mapcar #'cl-tf::pose-stamped->pose
;;                                       positions))))
;;     ;;(roslisp::ros-info (navigation-functions) "Started Designator with positions ~a" positions)
;;     (when (not positions) (return-from pose-with-distance-to-points Nil))
;;     ;;(comf::move-hsr (rotate-to-point (cl-tf::origin (try-movement-stamped-list positions))))
;;     (roslisp:ros-info (poi) "point selected: ~a" (try-movement-stamped-list positions))
;;       (if turn (llif::call-take-pose-action 4))
;;       (car points)
;;     ))


;;author Philipp Klein
;; (defun rotate-to-point (3dvec)
;;     (setf *curorigin*
;;        (roslisp::with-fields (translation)
;;                             (cl-tf::lookup-transform
;;                             cram-tf::*transformer*
;;                             "map"
;;                             "base_footprint")
;;                         translation))
;;    (setf *currot*
;;          (roslisp::with-fields (rotation)
;;                             (cl-tf::lookup-transform
;;                             cram-tf::*transformer*
;;                             "map"
;;                             "base_footprint")
;;                         rotation))
;;   (setf *crossp* (cl-tf::cross-product *curorigin* 3dvec))
;;   (print *crossp*)
;;   (cl-tf::make-pose-stamped "map" 0 *curorigin*
;;                             ;;(handler-case
;;                             ;;
;;                             (cl-tf::normalize
;;                              (cl-tf::make-quaternion
;;                               (cl-tf::x *crossp*)
;;                               (cl-tf::y *crossp*)
;;                               (cl-tf::z *crossp*)
;;                           (+
;;                            (sqrt
;;                             (*
;;                              (expt (cl-tf2::v-norm *curorigin*) 2)
;;                              (expt (cl-tf::v-norm 3dvec) 2)))
;;                            (cl-tf::dot-product *curorigin* 3dvec))))
;;                               ;;(error (c)
;;   ;;  (values *currot*))))
;;   ))

;;author Philipp Klein
;; (defun point-in-polygon (edges-list point)
;;   "return if the point is in the polygon
;;   `listOfEdges' the list of all edges of the polygon
;;   `point' the point to check if it is in the polygon"
;;   (setq poly-corner-1  (- (length edges-list) 1))
;;   (setq is-poly nil)
;;   (loop
;;     for i from 0 to (- (length edges-list) 1)
;;     do
;;        (if
;;         (not(and
;;              (> (cl-tf::y (nth i edges-list))
;;                 (cl-tf::y point))
;;              (> (cl-tf::y (nth poly-corner-1 edges-list))
;;                 (cl-tf::y point))))
;;         (if
;;             (< (cl-tf::x (nth i edges-list))
;;                (+ (cl-tf::x (nth i edges-list))
;;                   (/
;;                    (* (- (cl-tf::x (nth poly-corner-1 edges-list))
;;                          (cl-tf::x (nth i edges-list)))
;;                       (-(cl-tf::y point)
;;                         (cl-tf::y (nth i edges-list))))
;;                    (- (cl-tf::y (nth poly-corner-1 edges-list))
;;                       (cl-tf::y (nth i edges-list))))))
;;             (setq is-poly (not is-poly))
;;             (roslisp::ros-info (point-in-polygon) "Hit"))
;;             (setq poly-corner-1 i)))
;;   (roslisp::ros-info (point-in-polygon) "Next edge found: ~a" is-poly)
;;   is-poly)

;;source: https://stackoverflow.com/questions/2680864/how-to-remove-nested-parentheses-in-lisp
;; (defun flatten (l)
;;   "flatten a given list"
;;   (cond ((null l) nil)
;;         ((atom l) (list l))
;;         (t (loop for a in l appending (flatten a)))))

;; (defun create-move-position-list(object-id)
;;     (setq *pose* (llif:prolog-object-pose object-id))
;;   (let ((?nav-pose
;;           (list (cl-tf::make-pose-stamped
;;                  "map" 0 
;;                  (cl-tf:make-3d-vector
;;                   (- (nth 0 (nth 2 *pose*)) 0.5) ;;x-cordinate
;;                   (nth 1 (nth 2 *pose*))  ;;y-cordinate
;;                   0) 
;;                  (cl-tf::make-quaternion 0 0 0 1)) 
;;                 (cl-tf::make-pose-stamped
;;                  "map" 0 
;;                  (cl-tf:make-3d-vector
;;                   (+ (nth 0 (nth 2 *pose*)) 0.5) 
;;                   (nth 1 (nth 2 *pose*))
;;                   3.14) 
;;                  (cl-tf::make-quaternion 0 0 0 1))
;;                 (cl-tf::make-pose-stamped
;;                  "map" 0 
;;                  (cl-tf:make-3d-vector
;;                   (nth 0 (nth 2 *pose*))
;;                   (+ (nth 1 (nth 2 *pose*)) 0.5)
;;                   1.57)
;;                           (cl-tf::make-quaternion 0 0 0 1))
;;                 (cl-tf::make-pose-stamped
;;                  "map" 0 
;;                  (cl-tf:make-3d-vector
;;                   (nth 0 (nth 2 *pose*))
;;                   (- (nth 1 (nth 2 *pose*)) 0.5)
;;                   -1.57)
;;                  (cl-tf::make-quaternion 0 0 0 1)))))
;;     ?nav-pose))






;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;tts-functions.lisp;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; ;;@author Torge Olliges
;; (defun announce-movement-to-surface (time surface)
;;   (llif::call-text-to-speech-action
;;      (llif::get-string-from-nlg-result
;;       (llif::call-nlg-action-with-list
;;        (list
;;         (list "time" time)
;;         (list "action" "move")
;;         (list "goal_surface_id" surface))))))

;; (defun announce-movement-to-room (time room))

;;@author Torge Olliges
;; (defun announce-movement (time)
;;   (llif::call-text-to-speech-action
;;      (llif::get-string-from-nlg-result
;;       (llif::call-nlg-action-with-list
;;        (list
;;         (list "time" time)
;;         (list "action" "move"))))))

;;@author Torge Olliges
;; (defun announce-grasp-action (time object)
;;   (llif::call-text-to-speech-action
;;      (llif::get-string-from-nlg-result
;;       (llif::call-nlg-action-with-list
;;        (list
;;         (list "time" time)
;;         (list "action" "grasp")
;;         (list "object_id" object))))))

;;@author Torge Olliges
;; (defun announce-place-action (time object)
;;   (llif::call-text-to-speech-action
;;      (llif::get-string-from-nlg-result
;;       (llif::call-nlg-action-with-list
;;        (list
;;         (list "time" time)
;;         (list "action" "place")
;;         (list "object_id" object)
;;         (list "goal_surface_id" (llif::prolog-object-goal object)))))))

;;@author Torge Olliges
;; (defun announce-perceive-action-surface (time surface)
;;   (llif::call-text-to-speech-action
;;      (llif::get-string-from-nlg-result
;;       (llif::call-nlg-action-with-list
;;        (list
;;         (list "time" time)
;;         (list "action" "perceive")
;;         (list "goal_surface_id" surface))))))

;;@author Torge Olliges
;; (defun announce-perceive-action (time)
;;   (llif::call-text-to-speech-action
;;      (llif::get-string-from-nlg-result
;;       (llif::call-nlg-action-with-list
;;        (list
;;         (list "time" time)
;;         (list "action" "perceive"))))))






;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;execute-go-get-it.lisp;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;@author Torge Olliges
;; (defun move-to-start-position()
;;   (setf *starting-position* (cl-tf2::transform-stamped->pose-stamped
;;                              (cl-tf2::lookup-transform cram-tf:*transformer* "map" "base_footprint")))
;;   (setf *starting-room* (llif::prolog-current-room)))






;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;knowledge-client.lisp;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; @author Jan Schimpf
;; ask knowledge for how the object should be grasped 
;; 1 for grasping from front and 2 for grasping from the top
;;(defun prolog-object-grasp-mode (object-id))
;;    "Asking knowledge what graspmode to use"
;;    (roslisp:ros-info (json-prolog-client) "Asking knowledge what graspmode to use")
;;    (let* ((raw-response
;;           (with-safe-prolog
;;             (json-prolog:prolog-simple 
;;               
;;              :package :llif))))))  







;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;knowledge-surface-client.lisp;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; (defun prolog-surface-front-edge-pose (surface-name)
;;   "returns the pose of the front edge of a surface"
;;   (roslisp::ros-info (knowledge-surface-client) "Getting front edge pose for surface ~a" surface-name)
;;   (let*  ((knowrob-name (format nil "~a~a" +HSR-SURFACE-PREFIX+ surface-name))
;;           (raw-response (with-safe-prolog
;;                            (json-prolog:prolog-simple
;;                             (concatenate 'string
;;                                          "surface_front_edge_center_pose('"
;;                                          knowrob-name
;;                                          "', [TRANSLATION, ROTATION])")
;;                             :package :llif))))
;;     (if (eq raw-response 1)
;;         (roslisp:ros-warn (knowledge-surface-client)
;;                           "Query didn't surface_pose_in_map reach any solution.")
;;         (values-list `(,(list (cdr (assoc '?translation (cut:lazy-car raw-response)))
;;                               (cdr (assoc '?rotation (cut:lazy-car raw-response))))
;;                        ,(string-trim "'"
;;                                      (cdr
;;                                       (assoc '?context
;;                                              (cut:lazy-car raw-response)))))))))

;; ;; @author Torge Olliges
;; (defun prolog-surfaces-not-visited ()
;;   (let* ((raw-response (with-safe-prolog
;;                          (json-prolog:prolog-simple
;;                           (concatenate 'string
;;                                        "surfaces_not_visited(SURFACES)")
;;                           :package :llif))))
;;     (if (eq raw-response 1)
;;         (roslisp:ros-warn (knowledge-surface-client)
;;                           "Query didn't surfaces_not_visited reach any solution.")
;;         (values-list (list (mapcar
;;                             (lambda (x) (remove-string +HSR-SURFACE-PREFIX+ (string-trim "'" x)))
;;                       (cdr (assoc '?Surfaces (cut:lazy-car raw-response)))))))))

;;@author Philipp Klein
;; (defun prolog-is-pose-outside-stamped (stamped)
;;   (prolog-is-pose-outside
;;    (cl-tf::x (cl-tf::origin stamped))
;;    (cl-tf::y (cl-tf::origin stamped))
;;    (cl-tf::z (cl-tf::origin stamped))))

  
;;@author Torge Olliges
;; (defun prolog-is-pose-outside (x y z)
;;     (setf x (with-input-from-string
;;               (in (format Nil "~5,f" x))(read in)))
;;     (setf y (with-input-from-string
;;                 (in (format Nil "~5,f" y))(read in)))
;;     (setf z (with-input-from-string
;;            (in (format Nil "~5,f" z))(read in)))
;;   (let ((raw-response (with-safe-prolog
;;                         (json-prolog:prolog-simple
;;                          (concatenate 'string
;;                                       "pose_is_outside(["
;;                                       (write-to-string x)
;;                                       ", "
;;                                       (write-to-string y)
;;                                       ", "
;;                                       (write-to-string z)
;;                                       "])")
;;                          :package :llif))))
;;     (if raw-response
;;         T
;;         nil)))

;;@author Torge Olliges
;; (defun prolog-pose-room (x y z)
;;     (let ((raw-response (with-safe-prolog
;;                         (json-prolog:prolog-simple
;;                          (concatenate 'string
;;                                       "pose_in_room(["
;;                                       (write-to-string x)
;;                                       ", "
;;                                       (write-to-string y)
;;                                       ", "
;;                                       (write-to-string z)
;;                                       "], ROOM)")
;;                          :package :llif))))
;;       (remove-string +HSR-ROOMS-PREFIX+
;;                         (string-trim "'" (cdr (assoc '?Room (cut:lazy-car raw-response)))))))

;;@author Torge Olliges 
;; (defun prolog-surface-room (surface-id)
;;   (let* ((raw-response (with-safe-prolog
;;                          (json-prolog:prolog-simple
;;                           (concatenate 'string
;;                                        "surface_in_room('"
;;                                        surface-id
;;                                        "',ROOM)")
;;                           :package :llif))))
;;     (if (eq raw-response 1)
;;         (roslisp:ros-warn (knowledge-surface-client)
;;                           "Query surface_in_room didn't reach any solution.")
;;         (values-list (list (mapcar
;;                             (lambda (x) (string-trim "'" x))
;;                             (cdr (assoc '?Positions (cut:lazy-car raw-response)))))))))

;;@author Torge Olliges
;; (defun prolog-surface-furniture (surface-id)
;;   (let* ((knowrob-name (format nil "~a~a" +HSR-SURFACE-PREFIX+ surface-id))
;;          (raw-response (with-safe-prolog
;;                          (json-prolog:prolog-simple
;;                           (concatenate 'string
;;                                        "has_surface(Furniture,'"
;;                                        knowrob-name
;;                                        "')")
;;                           :package :llif))))
;;     (if (eq raw-response 1)
;;         (roslisp:ros-warn (knowledge-surface-client)
;;                           "Query has_surface didn't reach any solution.")
;;         (string-trim "\""(string-trim "'" (cdr (assoc '?Furniture (cut:lazy-car raw-response))))))))

;; this is ugly and only done due to the robocup 2021
;; (defun prolog-go-get-it-surfaces ()
;;   (let* ((raw-response (with-safe-prolog
;;                          (json-prolog:prolog-simple
;;                           (concatenate 'string
;;                                        "goandgetit_surfaces(SURFACES)")
;;                           :package :llif))))
;;     (if (eq raw-response 1)
;;         (roslisp:ros-warn (knowledge-surface-client)
;;                           "Query didn't  reach any solution.")
;;         (values-list (list (mapcar
;;                             (lambda (x) (remove-string +HSR-SURFACE-PREFIX+ (string-trim "'" x)))
;;                             (cdr (assoc '?Surfaces (cut:lazy-car raw-response)))))))))






;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;nlg-action-client.lisp;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;@autho Torge Olliges
;;Gets a list of tuple and turns, turns them into nlg msgs and then calls nlg with the result. 
;; (defun call-nlg-action-with-list (key-value-pair-lists-list)
;;     (call-nlg-action (mapcar 
;;         (lambda (key-value-list) 
;;             (make-key-value-msg 
;;                 (nth 0 key-value-list) 
;;                 (nth 1 key-value-list))) 
;;         key-value-pair-lists-list)))
