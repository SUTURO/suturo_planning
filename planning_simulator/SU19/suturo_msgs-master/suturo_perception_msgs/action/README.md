# Suturo Perception Actions

There are currently two actions defined for the Suturo perception module. Each of them creates a specific [RoboSherlock](https://github.com/Suturo1819/robosherlock) pipeline. Action servers as well as a launch file is provided by the [hsr_perception package](https://github.com/Suturo1819/hsr_perception).

## ExtractObjectInformation
This action starts a [RoboSherlock](https://github.com/Suturo1819/robosherlock) pipeline that is optimized to detect and recognize objects that are orthogonal to a large plane.

* **goal:** 
  * `visualize (bool)` Decide whether the [RoboSherlock](https://github.com/Suturo1819/robosherlock) visualization should  be displayed or not.
   * `regions (string[])` The semantic regions that should be considered.
* **result:**
  * `detection_data (Array made from `[ObjectDetectionData](https://github.com/Suturo1819/suturo_msgs/blob/master/suturo_perception_msgs/msg/ObjectDetectionData.msg)`)` One instance of [ObjectDetectionData](https://github.com/Suturo1819/suturo_msgs/blob/master/suturo_perception_msgs/msg/ObjectDetectionData.msg) for each observed and recognized object in the scene. 
* **feedback:**
  * `feedback (string)` A string containing information about whether the communcation was successfull or not.

## AnalyseShelfStatus
To be done...
