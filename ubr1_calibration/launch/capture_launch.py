<launch>

  <arg name="velocity_factor" default="1.0" />

  <!-- Disable auto white balance and exposure -->
  <node pkg="ubr_calibration" type="camera_reconfigure.py" name="camera_reconfigure" args="--disable" />

  <rosparam command="delete" param="robot_calibration" />
  <node pkg="robot_calibration" type="calibrate" name="robot_calibration"
        args="$(find ubr_calibration)/config/calibration_poses.bag"
        output="screen" required="true">
    <param name="velocity_factor" value="$(arg velocity_factor)" />
    <rosparam file="$(find ubr_calibration)/config/capture.yaml" command="load" />
    <rosparam file="$(find ubr_calibration)/config/calibrate.yaml" command="load" />
  </node>

  <node pkg="rosbag" type="record" name="calibration_bagger" output="screen"
        args="--bz2 -o /tmp/ubr_calibration/calibration_data calibration_data robot_description" >
    <param name="mkdir_tmp" command="mkdir -m 777 -p /tmp/ubr_calibration" />
  </node>

  <include file="$(find ubr1_moveit)/launch/move_group.launch" />
  
</launch>
