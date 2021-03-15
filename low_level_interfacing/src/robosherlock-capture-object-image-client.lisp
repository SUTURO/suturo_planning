;;; Perception/RoboSherlock client
(in-package :llif)

(defparameter *robosherlock-capture-object-action-client* NIL)
(defparameter *robosherlock-capture-action-timeout* 30.0 "in seconds")

(defun init-robosherlock-capture-object-action-client ()
    "initializes the RoboSherlock client"
    (roslisp:ros-info 
        (init-robosherlock-capture-client)
        "Creating robosherlock action client for server 'perception_server'.")
    (setf *robosherlock-capture-object-action-client*
        (actionlib:make-action-client 
            "/perception_capture_server"
            "suturo_perception_msgs/CaptureImageAction"))
    (roslisp:ros-info (init-robosherlock-capture-client) "'Actionlib made action client.")
    (loop until 
        (actionlib:wait-for-server 
            *robosherlock-capture-object-action-client*
            *robosherlock-capture-action-timeout*))
    (roslisp:ros-info 
        (init-robosherlock-capture-client)
        "Robosherlock action client for ~a created." "'extract_object_infos'"))

(defun get-robosherlock-capture-client ()
    "returns a RoboSherlock client if one already exists. Creates one otherwise."
    (unless *robosherlock-capture-object-action-client*
        (init-robosherlock-capture-object-action-client))
    *robosherlock-capture-object-action-client*)

;;@author Torge Olliges
(defun make-capture-action-goal (in-regions in-visualize)
    (actionlib::make-action-goal 
        (get-robosherlock-capture-client)
        :regions in-regions
        :visualize in-visualize))

;;@author Torge Olliges
(defun call-robosherlock-capture-object-action (regions-value  viz-value)
    "Calls the RoboSherlock pipeline. Triggers perception to perceive.
    Expects a regions to be given as a vector. E.g.
    `regions-value' (vector (string 'robocup_table))"
    (roslisp:ros-info 
        (robosherlock-capture-object-client) 
        "Calling pipeline for regions ~a." 
        regions-value)
  ;;actual call
  (format t "vector: ~a" regions-value)
  (multiple-value-bind 
      (result status)
      (actionlib:call-goal 
          (get-robosherlock-capture-client)
          (make-action-goal regions-value viz-value))
      (roslisp:ros-info 
          (robosherlock-capture-object-action) 
          "robosherlock action finished")
      (values result status)))
