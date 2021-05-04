;;; Perception/RoboSherlock client
(in-package :llif)

(defparameter *robosherlock-door-action-client* NIL)
(defparameter *robosherlock-door-action-timeout* 30.0 "in seconds")
(defparameter *robosherlock-door-client* NIL)

(defun init-robosherlock-door-client ()
    "initializes the RoboSherlock door client"
    (roslisp:ros-info 
        (init-robosherlock-door-client)
        "Creating robosherlock door action client for server 'perception_server'.")
    (print "was here")
    (setf *robosherlock-door-action-client*
        (actionlib:make-action-client 
            "/perception_actionserver_plane"
            "suturo_perception_msgs/ExtractPlaneInfoAction"))
    (roslisp:ros-info (init-robosherlock-client) "'Actionlib made action client.")
    (loop until 
        (actionlib:wait-for-server 
            *robosherlock-door-action-client*
            *robosherlock-door-action-timeout*))
    (roslisp:ros-info 
        (init-robosherlock-door-client)
        "Robosherlock action client for ~a created." "'extract_door_infos'"))

(defun get-robosherlock-door-client ()
    "returns a RoboSherlock client if one already exists. Creates one otherwise."
    (unless *robosherlock-door-action-client*
        (init-robosherlock-door-client))
    *robosherlock-door-action-client*)

;;@author Jan Schimpf
(defun make-action-goal-door (in-regions in-visualize)
    (actionlib::make-action-goal 
        (get-robosherlock-door-client)
        :regions in-regions
        :visualize in-visualize))

;;@author Jan Schimpf
(defun call-robosherlock-door-pipeline (regions-value viz-value)
    "Calls the RoboSherlock door pipeline. Triggers perception to perceive."
        (roslisp:ros-info 
        (robosherlock-object-client) 
        "Calling pipeline for regions ~a." 
        regions-value)
  ;;actual call
  (multiple-value-bind 
      (result status)
      (actionlib:call-goal 
          (get-robosherlock-door-client)
          (make-action-goal-door regions-value viz-value))
      (roslisp:ros-info 
          (robosherlock-open-action) 
          "robosherlock open finished")
      (values result status)))
