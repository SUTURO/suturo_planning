;;; Perception/RoboSherlock client
(in-package :llif)

(defparameter *robosherlock-plane-action-client* NIL)
(defparameter *robosherlock-plane-action-timeout* 30.0 "in seconds")
(defparameter *robosherlock-door-client* NIL)

(defun init-robosherlock-plane-action-client ()
  "Initializes the RoboSherlock plane client"
  (roslisp:ros-info (init-plane-robosherlock-client)
                    "Creating robosherlock plane action client for server 'perception_server'.")
  (setf *robosherlock-plane-action-client*
        (actionlib:make-action-client 
         "/perception_actionserver"
         "suturo_perception_msgs/ExtractPlaneInfoAction"))
  (roslisp:ros-info (init-plane-robosherlock-client)
                    "'Actionlib made plane action client.")
  (loop until 
        (actionlib:wait-for-server 
         *robosherlock-plane-action-client*
         *robosherlock-plane-action-timeout*))
  (roslisp:ros-info (init-plane-robosherlock-client)
                    "Robosherlock plane action client for ~a created."
                    "'extract_object_infos'"))

(defun get-robosherlock-plane-client ()
  "Returns a RoboSherlock client if one already exists. Creates one otherwise."
  (unless *robosherlock-plane-action-client*
    (init-robosherlock-plane-action-client))
  *robosherlock-plane-action-client*)

;;@author Torge Olliges
(defun make-plane-action-goal (in-regions in-visualize)
  "Receives values `in-regions' and `in-visualize'. Creates action goal using `in-regions' and `in-visualize'."
  (actionlib::make-action-goal 
      (get-robosherlock-plane-client)
    :regions in-regions
    :visualize in-visualize))

;;@author Torge Olliges
(defun call-robosherlock-plane-pipeline (regions-value  viz-value)
  "Receives values `regions-value' and `in-visualize'. Calls the RoboSherlock pipeline. Triggers perception to perceive.
    Expects a regions to be given as a vector. E.g. `regions-value' (vector (string 'robocup_table))"
  (roslisp:ros-info (robosherlock-object-client) 
                    "Calling pipeline for regions ~a." 
                    regions-value)
  ;;actual call
  (format t "vector: ~a" regions-value)
  (multiple-value-bind 
        (result status)
      (actionlib:call-goal 
       (get-robosherlock-plane-client)
       (make-plane-action-goal regions-value viz-value))
    (roslisp:ros-info (robosherlock-plane-action) 
                      "robosherlock plane action finished")
    (values result status)))
