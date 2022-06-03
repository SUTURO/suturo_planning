;;; Perception/RoboSherlock client
(in-package :llif)

(defparameter *robosherlock-process-object-image-action-client* NIL)
(defparameter *robosherlock-process-object-image-action-timeout* 30.0 "in seconds")

(defun init-robosherlock-process-object-image-action-client ()
  "Initializes the RoboSherlock client"
  (roslisp:ros-info (init-robosherlock-process-object-image-client)
                    "Creating robosherlock action client for server 'perception_server'.")
  (setf *robosherlock-process-object-image-action-client*
        (actionlib:make-action-client 
         "/perception_process_server"
         "suturo_perception_msgs/ProcessImageAction"))
  (roslisp:ros-info (init-process-object-image-robosherlock-client)
                    "'Actionlib made action client.")
  (loop until 
        (actionlib:wait-for-server 
         *robosherlock-process-object-image-action-client*
         *robosherlock-process-object-image-action-timeout*))
  (roslisp:ros-info (init-robosherlock-process-object-image-client)
                    "Robosherlock action client for ~a created."
                    "'extract_object_infos'"))

(defun get-robosherlock-process-object-image-client ()
  "Returns a RoboSherlock client if one already exists. Creates one otherwise."
  (unless *robosherlock-process-object-image-action-client*
    (init-robosherlock-process-object-image-action-client))
  *robosherlock-process-object-image-action-client*)

;;@author Torge Olliges
(defun make-process-object-image-action-goal (image-id)
  "Receives image ID `image-id'. Creates action goal using `image-id'."
  (actionlib::make-action-goal 
      (get-robosherlock-process-object-image-client)
    :cas_id image-id))

;;@author Torge Olliges
(defun call-robosherlock-process-object-image-pipeline (image-id)
  "Receives image ID `image-id'. Calls the perception process server with `image-id' which is to be proccessed."
  (roslisp:ros-info (robosherlock-process-object-image-client) 
                    "Calling pipeline process object image")
  ;;actual call
  (multiple-value-bind 
        (result status)
      (actionlib:call-goal 
       (get-robosherlock-process-object-image-client)
       (make-action-goal image-id))
    (roslisp:ros-info (robosherlock-process-object-image-action) 
                      "robosherlock process object impage action finished")
    (values result status)))
