#include "PlaneCalibrationV2.h"
#include "KinectSensor.h"

int main()
{
	KinectSensor kinect, kinectRef;

	kinect.initDevice(1, 2, true);//, "D:\\TwoKinectsRecording\\cam1.oni");
	kinectRef.initDevice(2, 2, true);//, "D:\\TwoKinectsRecording\\cam2.oni");

	PlaneCalibrationV2 calibTool;

//	list<Plane> planes1;
//	list<Plane> planes2;
	Plane* planes = new Plane[MAX_PLANES];
	Plane* planesRef = new Plane[MAX_PLANES];

	calibTool.retrievePlaneCorrespondences(kinect, kinectRef, planes, planesRef);

	return 0;
}