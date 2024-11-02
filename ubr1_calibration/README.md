# UBR-1 Calibration

This is used to calibrate the UBR-1 robot. You probably aren't looking
to do that since you probably don't have a UBR-1 - but you might be
here to see an example of how to use `robot_calibration` in ROS 2.

There are several config files of interest here:

 * `config/capture.yaml` - The capture step configuration. 
 * `config/calibrate.yaml` - The calibrate step configuration.
 * `config/calibration_poses.yaml` - The capture poses to use.
   These have been manually tweaked to be collision free when
   directly interpolating between adjacent poses - so that we
   don't have to rely on MoveIt to run planning. The first pose
   uses a ground plane finder to align the head camera to the
   ground, and then every other pose uses the LED finder.

There are several entry points:

 * `scripts/calibrate_launch.py` - Launch file that I use 99% of the
   time to calibrate the robot.
 * `scripts/calibrate_from_bag` - If the launch file has already
   been run, and I have a bagfile, but want to tweak some aspect of the
   calibration step, this is how to run it.
 * `scripts/manual_calibration` - This runs robot calibration in
   manual mode, where you have to move the robot arm and head yourself.
   I find this is generally only useful for developing new feature
   finders. This is NOT a launch file, since launch would eat our
   input data and we would not be able to hit enter...
 * 

There are additional files that are really maybe not great examples:

 * `config/checkerboard_2d` - this directy contains alternate
   capture and calibration configurations for using the `Checkerboard2d`
   finder. It performs suboptimally to the 3d version for a Primesense
   style camera that is already aligned. This particular configuration
   doesn't actually calibrate the robot, instead it simply finds the
   checkerboard transform to validate the pipeline.
