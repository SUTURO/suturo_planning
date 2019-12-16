(in-package :comp)

(defun init-interface()
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

(defun init-planning()
 "Initialize only local nodes for working without the real robot."
  (print "create ros node")
  (roslisp-utilities:startup-ros :name "planning_node" :anonymous nil)
  (print "init tf-listener")
  (plc::get-tf-listener)
  (print "init viz-box")
  (lli:viz-box-init)
  (print "init marker publisher")
  (lli:init-marker-publisher)
)
