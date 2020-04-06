;;; Perception/RoboSherlock client
(in-package :llif)

(defparameter *robosherlock-object-action-client* NIL)
(defparameter *robosherlock-action-timeout* 30.0 "in seconds")
(defparameter *robosherlock-door-client* NIL)

(defun init-robosherlock-object-action-client ()
    "initializes the RoboSherlock client"
    (roslisp:ros-info 
        (init-robosherlock-client)
        "Creating robosherlock action client for server 'perception_server'.")
    (setf *robosherlock-object-action-client*
        (actionlib:make-action-client 
            "/perception_actionserver"
            "suturo_perception_msgs/ExtractObjectInfoAction"))
    (roslisp:ros-info (init-robosherlock-client) "'Actionlib made action client.")
    (loop until 
        (actionlib:wait-for-server 
            *robosherlock-object-action-client*
            *robosherlock-action-timeout*))
    (roslisp:ros-info 
        (init-robosherlock-client)
        "Robosherlock action client for ~a created." "'extract_object_infos'"))

(defun get-robosherlock-client ()
    "returns a RoboSherlock client if one already exists. Creates one otherwise."
    (unless *robosherlock-object-action-client*
        (init-robosherlock-object-action-client))
    *robosherlock-object-action-client*)

;;@author Torge Olliges
(defun make-action-goal (in-regions in-visualize)
    (actionlib::make-action-goal 
        (get-robosherlock-client)
        :regions in-regions
        :visualize in-visualize))

;;@author Torge Olliges
(defun call-robosherlock-object-pipeline (regions-value  viz-value)
    "Calls the RoboSherlock pipeline. Triggers perception to perceive.
    Expects a regions to be given as a vector. E.g.
    `regions-value' (vector (string 'robocup_table))"
    (roslisp:ros-info 
        (robosherlock-object-client) 
        "Calling pipeline for regions ~a." 
        regions-value)
  ;;actual call
  (format t "vector: ~a" regions-value)
  (multiple-value-bind 
      (result status)
      (actionlib:call-goal 
          (get-robosherlock-client)
          (make-action-goal regions-value viz-value))
      (roslisp:ros-info 
          (robosherlock-object-action) 
          "robosherlock action finished")
      (values result status)))
