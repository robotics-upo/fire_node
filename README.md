# Fire-node
Detects fire using Python, ROS, and OpenCV

Dependencies: rospy, and std_msgs

The aim of this code is to detect the borders of a fire in thermal images. In order to do so, the node subscribes to the topic flip_image to obtain the images, detects the fire in those images, if they exist, and publishes them in a new topic called fire.

This code contains two methods to detect it:

1) Sobel. First, it obtains the border of the objects in a image, including the fire borders, through Sobel operators. After that, we seek for the strongest gradients and we plot it on the image in red colour. The final step is to make and plot the contours in green colour. Making contours requires binary images, so a threshold is needed. Currently, falses positives are NOT discarded.

2) Threshold. It makes a cut in temperature ignoring all the pixels which does not reach our threshold. Once again, we need binary images. As before, the final step is to make and plot the contours of the region of interest in green. Currently, false positives are discarded through an opening. That is, an erosion of the image followed by dilation.

Since the Sobel method is more complex than the threshold method, the last one is selected by default. In addition, this node contains 7 private ROS parameters:

a) method. It chooses the method which we want to use through a string. To select the Sobel method, you have to set 'sobel'. To select the threshold method, you have to set 'thresh'.

b) resize. It lets you modify the size of the image. By default, it is 1.00, maintining the original size.

c) path. The path where to save the images.

d) save. If this parameter is 'yes', the images will be saved taking into account the path parameter. By default, images are not saved.

e) threshval. It makes a threshold on the images through a non-negative integer. By default, 200 is set.

f) rows and cols. They are integers which are used to set the number of rows and columns of the kernel, which is used to make the opening transformation.


