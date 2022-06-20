# Customized docker image

This is a customized docker image that builds the ubr1_description
package (so we can see the robot model) and provides a way to
access the standard rviz configurations.

sudo apt-get install docker python3-rocker
docker build --tag ubr:main .
sudo rocker --net=host --x11 ubr:main rviz2
