# Fire-node
Detects fire using Python, ROS, and OpenCV

Dependencies: rospy, and std_msgs

The aim of this code is to detect the borders of a fire in thermal images. In order to do so, the node subscribes to the topic flip_image to obtain the images, detects the fire in those images, if they exist, and publishes them in a new topic called fire.

METHOD: Threshold. 

It makes a cut in intensity ignoring all the pixels which does not reach our threshold. We use binary images. As before, the final step is to make and plot the contours of the region of interest in red. Currently, false positives are discarded through an opening. That is, an erosion of the image followed by dilation but more tests are needed.

This node contains also 5 private ROS parameters:

a) path. The path where to save the images.

b) save. If this parameter is 1, the images will be saved taking into account the path parameter. By default, images are not saved.

c) threshval. It makes a threshold on the images through a non-negative integer. By default, 200 is set.

d) rows and cols. Integers which are used to set the number of rows and columns of the matrix for the opening transformation.


Flip image is a topic created from using flip-image ROS node package: https://github.com/robotics-upo/flip_image
