(in-package :comp)

(defun init-planning()
  "Init all interfaces from planning to other groups"
  (print "init ros node...")

  (roslisp-utilities:startup-ros :name "planning_node" :anonymous nil)
  (cram-tf::init-tf)

  ;;Init action clients
  (roslisp:ros-info (init-clients) "init navigation action client")
  (llif::init-nav-client)

  (roslisp:ros-info (init-clients) "init move-grippper-client action client")
  (llif::init-move-gripper-action-client)

)
