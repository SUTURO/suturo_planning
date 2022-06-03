;;; Perception/RoboSherlock client
(in-package :llif)

(defparameter *robosherlock-object-action-client-drawer* NIL)
(defparameter *robosherlock-action-timeout-drawer* 30.0 "in seconds")
(defparameter *robosherlock-door-client-drawer* NIL)

;; used in cleanup
;;@author Felix Krause
(defun init-robosherlock-object-action-client-drawer ()
  "Initializes the RoboSherlock client"
  (roslisp:ros-info (init-robosherlock-client)
                    "Creating robosherlock action client for server 'perception_actionserver_drawer'.")
  (setf *robosherlock-object-action-client-drawer*
        (actionlib:make-action-client 
         "/perception_actionserver_drawer"
         "suturo_perception_msgs/ExtractDrawerInfoAction"))
  (roslisp:ros-info (init-robosherlock-client)
                    "'Actionlib made action client.")
  (loop until 
        (actionlib:wait-for-server 
         *robosherlock-object-action-client-drawer*
         *robosherlock-action-timeout-drawer*))
  (roslisp:ros-info (init-robosherlock-client)
                    "Robosherlock action client for ~a created."
                    "'extract_object_infos'"))

;; used in cleanup
;;@author Felix Krause
(defun get-robosherlock-client-drawer ()
  "Returns a RoboSherlock client if one already exists. Creates one otherwise."
  (unless *robosherlock-object-action-client-drawer*
    (init-robosherlock-object-action-client-drawer))
  *robosherlock-object-action-client-drawer*)

;;@author Torge Olliges
(defun make-action-goal-drawer (in-regions in-visualize)
  "Receives values `in-regions' and `in-visualize'. Creates action goal using `in-regions' and `in-visualize'."
  (actionlib::make-action-goal 
      (get-robosherlock-client)
    :regions in-regions
    :visualize in-visualize))

;; used in cleanup
;;@author Felix Krause
(defun call-robosherlock-object-pipeline-drawer (regions-value  viz-value)
  "Receives regions value `regions-value' and viz value `viz-value'. Calls the RoboSherlock pipeline, triggering perception to perceive."
  (roslisp:ros-info (robosherlock-object-client) 
                    "Calling pipeline for regions ~a." 
                    regions-value)
  ;;actual call
  (format t "vector: ~a" regions-value)
  (multiple-value-bind 
        (result status)
      (actionlib:call-goal 
       (get-robosherlock-client-drawer)
       (actionlib::make-action-goal 
           (get-robosherlock-client-drawer)
         :regions regions-value
         :visualize viz-value))
    (roslisp:ros-info 
     (robosherlock-object-action) 
     "robosherlock action finished")
    (values result status)))








