#pragma once
#include "KinectSensor.h"
#include "Plane.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <list>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <ctype.h>
#include "XnCppWrapper.h"

#define MAX_DEPTH 10000
#define NO_TRACKING	0
#define BUILD_MODEL	1
#define TRACK_MODEL	2

#define DISPLAY_FRAME 1
#define DISPLAY_BPROJ 0

using namespace std;
using namespace cv;
using namespace xn;

class PlaneCalibrationV2
{
public:
	PlaneCalibrationV2(void);
	~PlaneCalibrationV2(void);

	void retrievePlaneCorrespondences(KinectSensor& kinect1, KinectSensor& kinect2, list<Plane>& planes1, list<Plane>& planes2);

	void calibrateKinects(const list<Plane> planes_Ref, const list<Plane> planes_NoRef, Matx33d& rotation, Matx31d translation);

private:

	void PlaneCalibrationV2::savePlaneData(int idCam, const Matx31f& n1, int idPlane, const char* rootPath, const char* type);
	void saveExtrinsics(int from, int to, const Matx33f& rotation, const Matx31f& translation);
	void calcualterExtrinsicInverse(const Matx33f& rotation, const Matx31f& translation, Matx33f *rot_inv, Matx31f *trans_inv);
	void updateResultImage(Mat results, double degrees, const  Matx31f& c1Rot, const Matx31f& c2, const KinectSensor& kinect, const char* txtRef, const char* txtNoRef, const char* txtAngle);
	double calculateRotation(const vector<Matx31f>&, const vector<Matx31f>&, const Matx31f&, const Matx31f&, int, Matx33f* rotation);
	Matx31f calculateTranslation(const list<Plane>& planes_Ref, const list<Plane>& planes_NoRef, Matx33f rotation);
	double checkTranslation(const list<Plane>& planes_Ref, const list<Plane>& planes_NoRef, const Matx33f rotation, const Matx31f translation);
};

