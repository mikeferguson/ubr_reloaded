<launch>

  <node pkg="robot_calibration" type="calibrate" name="calibrate"
        args="--from-bag /tmp/ubr_calibration/calibration_data.bag"
        output="screen" required="true">
    <rosparam file="$(find ubr_calibration)/config/calibrate.yaml" command="load" />
  </node>
  
</launch>
