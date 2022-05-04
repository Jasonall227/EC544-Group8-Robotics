# EC544 Robotic Arm
# Introduction
This is a project of smart robotic arm. 
**![](https://lh4.googleusercontent.com/YnqGPMua5DKG1GqPgmRCG3MxCVGu-gGpq4eCnYxumqbVkMMONBCr_ek1VuqSc3QMNBCitZd7FmIyc9wDpNZTqYJYMmBO0uoyD6oLdUBWaTS-rdn70jQ4QQ5vXQgCIPnEfwX_qf6FZnnuvs_uUA)
# Image Processing
The goal of the image processing unit is to be able to capture images from the camera and process the image to detect if the object appearing in the frame is the object that we want. 

The image processing unit consists of a raspberry pi and a pi camera. The specific Pi model is Raspberry Pi 3B+, with Debian bullseye installed as OS. The picamera we used is the Arducam B0033 module. It is a 5MP camera with OV5647 sensor. It has an angle of view of  54 x 41 degrees and a field of view of 2.0 x 1.33m1, which can generally cover a large area for object detection. Although it has a fixed focal length, the field of depth is generally large so can gather images we need. 

The Arducam B0033 camera module is connected to Raspberry Pi with the camera cable inserting to the CSI camera port. The Raspberry Pi configuration is modified to enable camera function. The Picamera uses V4L2 camera driver. By default it is installed in the kernel, however, we have to change the configuration to set up a video overlay to enable the normal function of the camera. 

The object detection algorithm uses openCV. It uses openCV’s DNN module. The algorithm uses the pre-trained models, the MS COCO dataset. COCO supports detection with a very large range of objects. A separate output window is shown in the test case to give a more straightforward feedback. Once it detects an object, a rectangle will be drawn on the object and the object name as well as the confidence. The box color is coded to correspond to the object type so that it will indicate the specific type of the object. The ObjectID of the detected object will be returned and accepted and analyzed. If it matches with what we want, it will execute the command to call controller and give a signal to the robotic arm. 

Due to some unexpected circumstances, our SD card on the raspberry pi broke into two pieces. It might be caused because of external forces. Though we have the backup updated to the realization of the object detection, we have to configure the camera setting. This will be further discussed in the later section of the evaluation part
