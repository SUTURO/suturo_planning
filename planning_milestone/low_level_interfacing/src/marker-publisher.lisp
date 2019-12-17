(in-package :lli)

(defparameter *perceived-data* nil)
(defparameter *perception-subscriber* nil)

(defun init-marker-publisher()
  (setf *marker-publisher*
          (roslisp:advertise "~location_marker" "visualization_msgs/Marker")))

(defparameter *marker-publisher* nil)
(defun get-marker-publisher ()
  (unless *marker-publisher*
    (init-marker-publisher))
  *marker-publisher*)


(defun publish-marker-pose (pose &key (parent "map") id (g 0.0))
  "gets a cl-tf:pose and places a visualization marker at the given pose"
  (let ((point (cl-transforms:origin pose))
        (rot (cl-transforms:orientation pose))
        (current-index 0))
    (roslisp:publish (get-marker-publisher)
                     (roslisp:make-message "visualization_msgs/Marker"
                                           (std_msgs-msg:stamp header) 
                                           (roslisp:ros-time)
                                           (std_msgs-msg:frame_id header)
                                           (typecase pose
                                             (cl-tf:pose-stamped (cl-tf:frame-id pose))
                                             (t parent))
                                           ns "goal_locations"
                                           id (or id (incf current-index))
                                           type (roslisp:symbol-code
                                                 'visualization_msgs-msg:<marker> :arrow)
                                           action (roslisp:symbol-code
                                                   'visualization_msgs-msg:<marker> :add)
                                           (x position pose) (cl-transforms:x point)
                                           (y position pose) (cl-transforms:y point)
                                           (z position pose) (cl-transforms:z point)
                                           (x orientation pose) (cl-transforms:x rot)
                                           (y orientation pose) (cl-transforms:y rot)
                                           (z orientation pose) (cl-transforms:z rot)
                                           (w orientation pose) (cl-transforms:w rot)
                                           (x scale) 0.09
                                           (y scale) 0.09
                                           (z scale) 0.09
                                           (r color) 1.0
                                           (g color) g
                                           (b color) 0.0
                                           (a color) 1.0))))

;(roslisp-utilities:register-ros-init-function init-perception-subscriber)
;(roslisp-utilities:register-ros-init-function get-marker-publisher)

                                        ;(roslisp-utilities:register-ros-cleanup-function cleanup-perception-subscriber)
