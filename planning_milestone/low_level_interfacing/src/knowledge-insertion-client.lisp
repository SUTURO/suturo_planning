;;; Knowledge Insertion Client
(in-package :llif)

(defvar *knowledge-action-client* NIL)
(defparameter *knowledge-action-timeout* 120.0 "in seconds")

(defun init-knowledge-action-client ()
  "initializes the Knowledge insertion client"
  (roslisp:ros-info (knowledge-client)
                    "Creating robosherlock action client for server 'knowledge_insertion_server'.")
  (setf *knowledge-action-client*
        (actionlib:make-action-client "/store_object_info_server"
                                      "knowledge_msgs/StoreObjectInfoAction"))
  (roslisp:ros-info (knowledge-client) "'Actionlib made action client.")
  (loop until (actionlib:wait-for-server *knowledge-action-client*
                                         *knowledge-action-timeout*))
  (roslisp:ros-info (knowledge-client)
                    "knowledge insertion action client for ~a created." "'store_object_infos'"))

(defun get-knowledge-client ()
  "returns a knowledge insertion client if one already exists. Creates one otherwise."
  (unless *knowledge-action-client*
    (init-knowledge-action-client))
  *knowledge-action-client*)


(defun insert-knowledge-objects (objectArrayMsg)
  "Expects The ObjectDetectionData Msg as Input and insert the Objects to the Knowledge Database"
  (roslisp:ros-info (knowledge-client) "Inserting object msg to knowledge"
  (actionlib:call-goal (llif::get-knowledge-client)
                       objectArrayMsg
                       :timeout *robosherlock-action-timeout*
                       :result-timeout *robosherlock-action-timeout*)))

