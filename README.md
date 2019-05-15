# Fire-node
Detects fire using Python, ROS, and OpenCV

Dependencies: rospy, and std_msgs

The aim of this code is to detect the borders of a fire in thermal images. In order to do so, the node subscribes to the topic flip_image to obtain the images, detects the fire in those images, if they exist, and publishes them in a new topic called fire.

This code contains two methods to detect it:

1) Sobel. First, it obtains the border of the objects in a image, including the fire borders, through Sobel operators. After that, we seek for the strongest gradients and we plot it on the image in red colour. The final step is to make and plot the contours in green colour. Making contours requires binary images, so a threshold is needed.

2) Threshold. It makes a cut in temperature (intensity of the image) ignoring all the pixels which does not reach our threshold. Once again, we need binary images. As before, the final step is to make and plot the contours of the region of interest in green.

Since the Sobel method is more complex than the threshold method, the last one is selected by default. In addition, this node contains 2 private ROS parameters which are method and threshold.

a) method. It chooses the method which we want to use through a string. To select the Sobel method, you have to set 'sobel'. To select the threshold method, you have to set 'thresh', which is used by default.

b) threshold. It chooses the threshold to the images through a non-negative integer. By default, 200 is set.

Optionally, you can resize the image from the flip_image topic.
