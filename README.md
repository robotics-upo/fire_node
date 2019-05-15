# Fire-node
Detect fire using Python, ROS, and OpenCV

Dependencies: rospy, and std_msgs

The aim of this code is to detect the borders of a fire in thermal images. In order to do so, the node subscribes to the topic flip_image, getting the images, detects the fire in those images, if they exist, and publishes them in a new topic called fire.

This code contains two algorithms to detect it:

1 Sobel operators. They let you obtain the border of the objects in a image, including the fire borders. After that, we seek for the strongest gradients and we plot it on the image in red. The final step is to make and plot in green the contours. This requires binary images, so a threshold is needed.

2 Threshold. It makes a cut in temperature (intensity of the image) ignoring all the pixels which does not reach our threshold. Once again, we need binary images. As before, the final step is to make and plot the contours of the region of interest in green.

Since this method is simpler than the previous one, this algorithm is selected by default. However, they can be chosen through 2 private ROS parameters: method ('thresh' by default and 'sobel') and threshold (200 by default but it can be any positive integer).

Optionally, you can resize the image from the first topic
