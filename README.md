# HATLab Body Tracking Application  

The following scripts make up the backend service of the `Body Tracking Application`. These include scripts from the Openpose library for extracting body joint coordinates of people from images, as well as custom scripts used for running the backend TCP server and client as well as performing the image processing pipeline for extracting and transmitting body joint coordinates to unity application.  

The pipeline includes processes such as:  

* Stereo camera calibration
* Recieving images from TCP
* Image rectification
* joint coordinate extraction from images
* Depth extraction of joint coordinates
* Transmission of 3D joint coordinates over TCP  

### Script Designed by Us  

|Script|Desc|
|------|----|
|[TCPClient.py](./TCPClient.py)|Responsible for transmitting the extracted body joint coordinates back to the unity application to be displayed for the user.|
|[TCPServer.py](./TCPServer.py)|Responsible for receiving the stream of images sent by the unity application, and enacting the processing pipeline for processing the images to extract the body joint coordinates.|
|[HLCameraCalibration.py](./HLCameraCalibration.py)|Used for calibrating the front facing stereo cameras on the HoloLens 2 device, as well as performing the depth calculation for determining the depth of each independent joint.|
|[CoordinateLogging.py](./CoordinateLogging.py)|Used for logging the extracted body joint coordinates to view changes between coordinate frames.|  

Note: `ImageRectification.py`, `Triangulation.py`, and the main components of `HLCameraCalibration.py` come from the the repository [(StereoVisionDepthEstimation)](https://github.com/niconielsen32/ComputerVision/tree/master/StereoVisionDepthEstimation)
