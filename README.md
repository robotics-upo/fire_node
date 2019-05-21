# Fire-node
Detects fire using Python, ROS, and OpenCV

Dependencies: rospy, and std_msgs

The aim of this code is to detect the borders of a fire in thermal images. In order to do so, the node subscribes to the topic flip_image to obtain the images, detects the fire in those images, if they exist, and publishes them in a new topic called fire.

METHOD(S)

1) Threshold. It makes a cut in temperature ignoring all the pixels which does not reach our threshold. Once again, we need binary images. As before, the final step is to make and plot the contours of the region of interest in green. Currently, false positives are discarded through an opening. That is, an erosion of the image followed by dilation.

This node contains also 6 private ROS parameters:

a) resize. It lets you modify the size of the image. By default, it is 1.00, maintining the original size.

b) path. The path where to save the images.

c) save. If this parameter is 'yes', the images will be saved taking into account the path parameter. By default, images are not saved.

d) threshval. It makes a threshold on the images through a non-negative integer. By default, 200 is set.

e) rows and cols. They are integers which are used to set the number of rows and columns of the matrix.  opening transformation.


