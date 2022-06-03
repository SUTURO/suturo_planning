;;; Knowledge Insertion Client
(in-package :llif)

(defvar *knowledge-action-client* NIL)
(defparameter *knowledge-action-timeout* 30.0 "in seconds")

;; used in cleanup
(defun init-knowledge-action-client ()
  "Initializes the Knowledge insertion client"
  (roslisp:ros-info (knowledge-client)
                    "Creating robosherlock action client for server 'knowledge_insertion_server'.")
  (setf *knowledge-action-client*
        (actionlib:make-action-client "/store_object_info_server"
                                      "knowledge_msgs/StoreObjectInfoAction"))
  (roslisp:ros-info (knowledge-client)
                    "Actionlib made action client.")
  (loop until (actionlib:wait-for-server *knowledge-action-client*
                                         *knowledge-action-timeout*))
  (roslisp:ros-info (knowledge-client)
                    "knowledge insertion action client for ~a created." "'store_object_infos'"))

(defun get-knowledge-client ()
  "Returns a knowledge insertion client if one already exists. Creates one otherwise."
  (unless (and *knowledge-action-client*
              (actionlib::connected-to-server *knowledge-action-client*))
    (init-knowledge-action-client))
  *knowledge-action-client*)

;; used in cleanup
(defun insert-knowledge-objects (detected-objects)
  "Receives detected objects `detected-objects'. Expects The ObjectDetectionData Msg as Input and insert the Objects to the Knowledge Database"
  (roslisp:ros-info (knowledge-client)
                    "Sending object detection msg to knowledge.")
  (print detected-objects)
  (actionlib:call-goal (llif::get-knowledge-client)
                       detected-objects
                       :timeout *robosherlock-action-timeout*
                       :result-timeout *robosherlock-action-timeout*))

(defun insert-knowledge-drawer (drawer-handle)
  "Receives detected objects `detected-objects'. Expects The ObjectDetectionData Msg as Input and insert the Objects to the Knowledge Database"
  (roslisp:ros-info (knowledge-client)
                    "Sending object detection msg to knowledge.")
  (print drawer-handle)
  (actionlib:call-goal (llif::get-knowledge-client)
                       drawer-handle
                       :timeout *robosherlock-action-timeout*
                       :result-timeout *robosherlock-action-timeout*))

;; @author Tom-Eric Lehmkuhl
(defun knowledge-insertion-test ()
  "Insertion test with dummy message. Not working yet. Probably wrong message type"
  (roslisp:ros-info (knowledge-insertion-client)
                    "Insert dummy object to knowledge")
  (let* ((position (roslisp:make-msg "geometry_msgs/Point" (x) 2 (y) 4 (z) 0))
         (orientation (roslisp:make-msg "geometry_msgs/Quaternion"
                                        (x) 0 (y) 1 (z) 0 (w) 1))
         (pose (roslisp:make-msg "geometry_msgs/Pose" (position) position
                                 (orientation) orientation))
         (header (roslisp:make-msg "std_msgs/Header" (frame_id) "/frame"))
         (objectmessage (roslisp:make-msg
                         "suturo_perception_msgs/ObjectDetectionData"
                         (no-applicable-method) "test-object"
                         (obj_class) "test-ckass"
                         (confidence_class) 1.0
                         (shape) 1
                         (confidence_shape) 0.8
                         (pose) (roslisp:make-msg "geometry_msgs/PoseStamped"
                                                  (header) header
                                                  (pose) pose)
                         (width) 1.3
                         (height) 5.7
                         (depth) 0.2
                         (color) (roslisp:make-msg "std_msgs/ColorRGBA"
                                                   (r) 1
                                                   (g) 1
                                                   (b) 0
                                                   (a) 0)
                         (confidence_color) 0.6
                         (region) "robocup_default"))
         (detection-data (roslisp:make-msg "knowledge_msgs/StoreObjectInfoGoal"
                                           (detectionData)
                                           (make-array 1 :initial-contents
                                                       '((objectmessage))))))
    (insert-knowledge-objects detection-data)))
