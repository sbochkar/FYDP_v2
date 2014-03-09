This toolbox is an extension of the "Camera Calibration Toolbox" of Jean-Yves Bouguet. It permits to easily calibrate a projector-camera system by means of a plane-based method.

Authors: Gabriel Falcao, Natalia Hurtos, Joan Massich

Revised by alphashi(yuexinshi@gmail.com)
12/20/2012

The revised version was run on the MATLAB(v7.12),with the resolution of the camera and projector are 640*480,1280*1024 respectively.


This time solve those problems below:

1.The ill-condition problem below.

Warning: View #1 ill-conditioned. This image is now set inactive. (note: to disactivate this option, 
set check_cond=0)
5...Intrinsic parameters at frame 1 do not exist
List of images left desactivated: 1

2.The active_images are always equal to 1, can't add new images to recalibration.


Attention£ºThe images got for calibrating the projector just need to change the distance or angular for the projector. And the location of the camera and projcetor need not to be fixed.







