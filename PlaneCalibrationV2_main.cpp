#include "PlaneCalibrationV2.h"
#include "KinectSensor.h"

int main()
{
	KinectSensor kinect1, kinect2;

	kinect1.initDevice(1, 2, true);//, "D:\\TwoKinectsRecording\\cam1.oni");
	kinect2.initDevice(2, 2, true);//, "D:\\TwoKinectsRecording\\cam2.oni");

	PlaneCalibrationV2 calibTool;

	list<Plane> planes1;
	list<Plane> planes2;
	calibTool.retrievePlaneCorrespondences(kinect1, kinect2, planes1, planes2);

	return 0;
}