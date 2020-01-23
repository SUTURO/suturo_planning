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

(defun knowledge-insertion-test ()
(defparameter *position* (roslisp:make-msg "geometry_msgs/Point" (x) 2 (y) 4 (z) 0))
(defparameter *orientation* (roslisp:make-msg "geometry_msgs/Quaternion" (x) 0 (y) 1 (z) 0 (w) 1))
(defparameter *pose* (roslisp:make-msg "geometry_msgs/Pose" (position) *position* (orientation) *orientation*))
(defparameter *header* (roslisp:make-msg "std_msgs/Header" (frame_id) "/frame"))
  (defparameter *object-message* (roslisp:make-msg "suturo_perception_msgs/ObjectDetectionData"
    (name) "test-object"
    (obj_class) "test-ckass"
    (confidence_class) 1.0
    (shape) 1
    (confidence_shape) 0.8
    (pose) (roslisp:make-msg "geometry_msgs/PoseStamped"
      (header) *header*
      (pose) *pose*)
    (width) 1.3
    (height) 5.7
    (depth) 0.2
    (color) (roslisp:make-msg "std_msgs/ColorRGBA"
      (r) 1
      (g) 1
      (b) 0
      (a) 0)
    (confidence_color) 0.6
    (region) "somewhere"))
    (insert-knowledge-objects *object-message*))