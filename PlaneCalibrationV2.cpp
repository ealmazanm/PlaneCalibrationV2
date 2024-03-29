#include "PlaneCalibrationV2.h"


ofstream outDebug("D:\\debug.txt", ios::out);

PlaneCalibrationV2::PlaneCalibrationV2(void)
{
}


PlaneCalibrationV2::~PlaneCalibrationV2(void)
{
}

double TukeyBiweight(double e, double c)  
{
	if(fabs(e)<c)
		return(powf(1.0-powf(e/3.0,2.0),2.0)); //?
	else
		return(0.0);
}

void ConvertXnRGB24PixelToFrame(const XnRGB24Pixel *P, Mat M)
{
	uchar *imagePtr = (uchar*)M.data;
	for (int y=0; y<XN_VGA_Y_RES*XN_VGA_X_RES; y++)
	{
		imagePtr[3*y]   = P->nBlue;
		imagePtr[3*y+1] = P->nGreen;
		imagePtr[3*y+2] = P->nRed;
		P++;
	}
}

void writeMatrixValues(const Mat& m)
{
	int nl = m.rows;
	int nc = m.cols;
	
	for (int r = 0; r < nl; r++)
	{
		const float* dataPtr = m.ptr<float>(r);
		for (int c = 0; c < nc; c++)
		{
			outDebug << *dataPtr << " ";
			dataPtr++;
		}
		outDebug << endl;
	}
}

void PlaneCalibrationV2::retrievePlaneCorrespondences(KinectSensor& kinect, KinectSensor& kinectRef, Plane* planes, Plane* planesRef)
{
	Matx33d rotation;
	Matx31d translation;

	// Control variables
	bool bContinueLoop = true;
	int err1, err2, trackingMode = NO_TRACKING;
	int displayMode = DISPLAY_FRAME;

	// Image Variables
	unsigned short Mapping[MAX_DEPTH];
	Mat Frame1(XN_VGA_Y_RES, XN_VGA_X_RES,CV_8UC3);
	Mat Frame2(XN_VGA_Y_RES, XN_VGA_X_RES,CV_8UC3);
	
	// Tracking variables
	Point origin(3*XN_VGA_X_RES/8,3*XN_VGA_Y_RES/8), corner(5*XN_VGA_X_RES/8,5*XN_VGA_Y_RES/8);
	Rect selectWindow;

	// Set selection window
	selectWindow.x = origin.x;
    selectWindow.y = origin.y;
    selectWindow.width = corner.x-origin.x;
    selectWindow.height = corner.y-origin.y;

	// Plane variables 
	float minDepth=500, maxDepth=2000;

	cvNamedWindow("Image Frame 1", 1);
	cvNamedWindow("Image Frame 2", 1);
	
	int numPlanes = 0;
	kinect.startDevice();
	kinectRef.startDevice();

	Mat WeighImg1;
	Mat WeighImg2;

	double finalError = -1;
	double rotError = -1;
	Plane plane, planeRef;
	//Start and wait cameras
	while(bContinueLoop && numPlanes < MAX_PLANES && (numPlanes < 50 || abs(finalError-GROUNDTRUTH) > MAXIMUM_ERROR))
	{
		WeighImg1 = Mat::zeros(XN_VGA_Y_RES, XN_VGA_X_RES,CV_32FC1);
		WeighImg2 = Mat::zeros(XN_VGA_Y_RES, XN_VGA_X_RES,CV_32FC1);

		kinect.waitAndUpdate();
		kinectRef.waitAndUpdate();

		//retrieve data
		const XnDepthPixel* depthMap1 = kinect.getDepthMap();
		const XnDepthPixel* depthMap2 = kinectRef.getDepthMap();
		const XnRGB24Pixel* rgbMap1 = kinect.getRGBMap();
		const XnRGB24Pixel* rgbMap2 = kinectRef.getRGBMap();

		// Copy RGB Generator data into Mat Frame (For display only)
		ConvertXnRGB24PixelToFrame(rgbMap1,Frame1);
		ConvertXnRGB24PixelToFrame(rgbMap2,Frame2);

		if( trackingMode!=NO_TRACKING )
		{
			// Build model
			if( trackingMode==BUILD_MODEL )
			{
				err1 = plane.frontoPlaneFit( depthMap1, &kinect, selectWindow, minDepth,maxDepth );
				err2 = planeRef.frontoPlaneFit( depthMap2, &kinectRef, selectWindow, minDepth,maxDepth );

				if((err1!=0)||(plane.getFitWindow().area()<=1)||(err2!=0)||(planeRef.getFitWindow().area()<=1))
				{
					trackingMode=NO_TRACKING;
					cout << "No initial plane found";
					if (err1 != 0) cout << " in cam1 ";
					if (err2 != 0) cout << " and cam2. " << endl;
					else
						cout << endl;
				}
				else trackingMode=TRACK_MODEL;
			}
			// Track Model
			else
			{
				err1 = plane.updatePlaneFit( depthMap1, &kinect, WeighImg1, rgbMap1);
				err2 = planeRef.updatePlaneFit( depthMap2, &kinectRef, WeighImg2, rgbMap2);
//Debug
rectangle(Frame1, Point(plane.getFitWindow().x, plane.getFitWindow().y), Point(plane.getFitWindow().x+plane.getFitWindow().width, plane.getFitWindow().y+plane.getFitWindow().height),Scalar(255,255,255), 1,1,0);			
rectangle(Frame2, Point(planeRef.getFitWindow().x, planeRef.getFitWindow().y), Point(planeRef.getFitWindow().x+planeRef.getFitWindow().width, planeRef.getFitWindow().y+planeRef.getFitWindow().height),Scalar(255,255,255), 1,1,0);			
rectangle(WeighImg1, Point(plane.getFitWindow().x, plane.getFitWindow().y), Point(plane.getFitWindow().x+plane.getFitWindow().width, plane.getFitWindow().y+plane.getFitWindow().height),Scalar(255,255,255), 1,1,0);			
rectangle(WeighImg2, Point(planeRef.getFitWindow().x, planeRef.getFitWindow().y), Point(planeRef.getFitWindow().x+planeRef.getFitWindow().width, planeRef.getFitWindow().y+planeRef.getFitWindow().height),Scalar(255,255,255), 1,1,0);			

				if((err1!=0)||(plane.getFitWindow().area()<=1)||(err2!=0)||(planeRef.getFitWindow().area()<=1))
				{
					cout << " Error=" << err1 << ", Area=" << plane.getFitWindow().area() << ", ";
					cout << "[" << plane.getFitWindow().width << "X" << plane.getFitWindow().height << "] at (" << plane.getFitWindow().x << "X" << plane.getFitWindow().y << ") ";
					trackingMode=NO_TRACKING;
					plane.setResidualVariance(900);
					planeRef.setResidualVariance(900);
					cout << "Tracked plane lost\n";
					numPlanes -= 4;
				}
//				else
//					cout << " StdDev=" << sqrt(plane.getResidualVariance());
			}

			// Image DISPLAY
			Point centroid1 = kinect.pointProject(*plane.getCentroid());
			Point normal1   = kinect.pointProject(*plane.getCentroid() + 300*(*plane.getNormal()));
			line(Frame1, centroid1, normal1, Scalar(0,0,255), 2, 3, 0);

			Point centroid2 = kinectRef.pointProject(*planeRef.getCentroid());
			Point normal2   = kinectRef.pointProject(*planeRef.getCentroid() + 300*(*planeRef.getNormal()));
			line(Frame2, centroid2, normal2, Scalar(0,0,255), 2, 3, 0);
		}

		if (trackingMode != NO_TRACKING)
		{
			planes[numPlanes] = plane;
			planesRef[numPlanes++] = planeRef;

			if (numPlanes >= MIN_PLANES)
			{
				rotError = calculateRotation(planes, planesRef, rotation, numPlanes);
				finalError = calculateTranslation(planes, planesRef, &rotation, translation, numPlanes);
				cout << "Total Error: " << abs(finalError-GROUNDTRUTH) << endl;
			}

		}
			// Display
		rectangle(Frame1, origin, corner, Scalar(255,0,0), 1,1,0);
		rectangle(Frame2, origin, corner, Scalar(255,0,0), 1,1,0);
		imshow("Image Frame 1", Frame1);
		imshow("Image Frame 2", Frame2);
		imshow("Weigh Frame1", WeighImg1);
		imshow("Weigh Frame2", WeighImg2);
		// Control
		int	keyValue = cvWaitKey(1);
		if (keyValue==27) bContinueLoop = false;
		if (keyValue==13) trackingMode = BUILD_MODEL;
	}

	kinect.stopDevice();
	kinectRef.stopDevice();

	cout << "Translation: " << translation(0) << ", " << translation(1) << ", " << translation(2) << endl;
	cout << "Rotation Error: " << rotError << endl;
	cout << "Number of Planes: " << numPlanes << endl;

	//Save in disk extrinsics from NoRef to Ref
	saveExtrinsics(kinect.getIdCam(), kinect.getIdRefCam(), rotation, translation);

	Matx31f trans_inv;
	Matx33f rot_inv;
	calcualterExtrinsicInverse(rotation, translation, &rot_inv, &trans_inv);
	//Save in disk extrinsics from Ref to NoRef
	saveExtrinsics(kinectRef.getIdRefCam(), kinectRef.getIdCam(), rot_inv, trans_inv);	



}

void PlaneCalibrationV2::savePlaneData(int idCam, const Matx31f& n1, int idPlane, const char* rootPath, const char* type)
{
	char idCamStr[10], idPlaneStr[10];
	itoa(idCam, idCamStr, 10);
	itoa(idPlane, idPlaneStr, 10);

//	char *path = "D:\\CameraCalibrations\\dataCalib\\normals\\cam";

	//create filename
	char fileNamePlane[150];
	strcpy(fileNamePlane, rootPath);
	strcat(fileNamePlane, idCamStr);
	strcat(fileNamePlane, "\\");
	strcat(fileNamePlane, type);
	strcat(fileNamePlane, idCamStr);
	strcat(fileNamePlane, "_");
	strcat(fileNamePlane, idPlaneStr);
	strcat(fileNamePlane, ".yml");

	FileStorage fsNorm(fileNamePlane, FileStorage::WRITE);

	fsNorm << type << Mat(n1);

	fsNorm.release();
}

void PlaneCalibrationV2::saveExtrinsics(int from, int to, const Matx33f& rotation, const Matx31f& translation)
{
	char fromStr[10], toStr[10];
	itoa(from, fromStr, 10);
	itoa(to, toStr, 10);

	char *path = "D:\\CameraCalibrations\\extrinsics\\";
	//create filename
	char fileNameRot[150], fileNameTrans[150];
	char common[50];
	strcpy(common, fromStr);
	strcat(common, toStr);
	strcat(common, ".yml");

	strcpy(fileNameRot, path);
	strcpy(fileNameTrans, path);
	strcat(fileNameRot, "rotation_");
	strcat(fileNameTrans, "translation_");
	strcat(fileNameRot, common);
	strcat(fileNameTrans, common);

	FileStorage fsRot(fileNameRot, FileStorage::WRITE);
	FileStorage fsTra(fileNameTrans, FileStorage::WRITE);

	fsRot << "Rotation" << Mat(rotation);
	fsTra << "Translation" << Mat(translation);

	fsRot.release();
	fsTra.release();
}

void PlaneCalibrationV2::calcualterExtrinsicInverse(const Matx33f& rotation, const Matx31f& translation, Matx33f *rot_inv, Matx31f *trans_inv)
{
	//Rotation inverse
	*rot_inv = rotation.t();
	
	//translation inverse	
	*trans_inv = (*rot_inv)*translation;
	for (int i = 0; i < 3; i++)
		(*trans_inv)(i) = -(*trans_inv)(i);
}

double PlaneCalibrationV2::calculateRotation(const Plane* planes, const Plane* planesRef, Matx33d& rotation, const int numPlanes)
{
	// d x n matrices
	Mat X (3, numPlanes, CV_64F); // No Ref.
	Mat Y (3, numPlanes, CV_64F); // Reference.

	/*calculate the shifted normals and fill X and Y with the normals shifted as columns*/
	for (int i = 0; i < numPlanes; i++)
	{
		Mat(*(planes[i].getNormal())).copyTo(X.col(i));
		Mat(*(planesRef[i].getNormal())).copyTo(Y.col(i));
	}

	//calculate S and SVD(S)
	SVD svd(X*Y.t());
	Mat rot = svd.vt.t()*svd.u.t();
	double det = determinant(rot);
	if (det != 1)
	{
		Mat i = Mat::eye(3, 3, CV_64F);
		i.at<double>(2, 2) = det;
		rot = svd.vt.t()*i*svd.u.t();
	}
	rotation = rot;
	double sumRads = 0.0;
	//Evaluate the rotation error;
	for (int i = 0; i < numPlanes; i++)
		sumRads += acos((rot*X.col(i)).dot(Y.col(i)));
	
	return (sumRads/double(numPlanes))*180.0/CV_PI;
}

double PlaneCalibrationV2::calculateRotation(const vector<Matx31f>& normalsNoRef, const vector<Matx31f>& normalsRef, const Matx31f& nCentrNoRef, const Matx31f& nCentrRef, int nPlanes, Matx33f* rotation)
{
	Mat X(nPlanes, 3, CV_32F);
	Mat Y(nPlanes, 3, CV_32F);
	vector<Matx31f>::const_iterator nNoRef_iter = normalsNoRef.begin();
	vector<Matx31f>::const_iterator nRef_iter = normalsRef.begin();
	int cont = 0;

	//to evaluate rotation
	Mat totalNoRef(3, nPlanes, CV_32F); 
	Mat totalRef(3, nPlanes, CV_32F); 
	Mat totalNoRef_Rot(3, nPlanes, CV_32F); 

	//Group normals
	for (nNoRef_iter; nNoRef_iter != normalsNoRef.end(); nNoRef_iter++, nRef_iter++, cont++)
	{
		float* dataNoRef_Ptr = X.ptr<float>(cont);
		float* dataRef_Ptr = Y.ptr<float>(cont);
		//TODO: CHECK
		Matx31f centeredNoRef = (*nNoRef_iter)-nCentrNoRef;
		Matx31f centeredRef = (*nRef_iter)-nCentrRef;
		for (int i = 0; i < 3; i++)
		{
			*dataNoRef_Ptr++ = centeredNoRef(i);
			*dataRef_Ptr++ = centeredRef(i);

			//TODO: CHECK THIS PART
			float* totalNoRefPtr = totalNoRef.ptr<float>(i); 
			float* totalRefPtr = totalRef.ptr<float>(i); 
			totalNoRefPtr[cont] = (*nNoRef_iter)(i);
			totalRefPtr[cont] = (*nRef_iter)(i);
		}
	}
	Mat S = X.t()*Y;
	SVD svd(S);
	Mat rot = svd.vt.t()*svd.u.t();
	double det = determinant(rot);
	if (det != 1)
	{
		Mat i = Mat::eye(S.rows, S.cols, CV_32F);
		i.at<float>(S.rows-1, S.cols-1) = det;
		rot = svd.vt.t()*i*svd.u.t();
	}

	//Evaluate rotation
	totalNoRef_Rot = rot*totalNoRef;
	double sumRads = 0.0;
	outDebug << "radiands" << endl;
	for (int i = 0; i < nPlanes; i++)
	{
		double t = acos(totalNoRef_Rot.col(i).dot(totalRef.col(i)));
		outDebug << t << endl;
		sumRads += t;
	}
	outDebug << "Total Radians: " << sumRads << endl;
	outDebug << "Num planes: " << nPlanes << endl;
	*rotation=Matx33f(rot);
	
	return (sumRads/double(nPlanes))*180.0/CV_PI;
}


void PlaneCalibrationV2::updateResultImage(Mat results, double degrees, const  Matx31f& c1Rot, const Matx31f& c2, const KinectSensor& kinect, const char* txtRef, const char* txtNoRef, const char* txtAngle)
{
	Matx31d point2d_0(120,100,700); //origin 2d
	Matx31f point3d_0 = kinect.pointBackproject(point2d_0); //origin 3D
	Point point2d_1 = kinect.pointProject(point3d_0 + 300*c2); //dest2 2D
	Point point2d_2 = kinect.pointProject(point3d_0 + 300*c1Rot); //dest1 2D
	//Draw the two normals and the origin
	line(results, Point(point2d_0(0),point2d_0(1)), point2d_1, Scalar(0,0,255), 2, 3, 0);
	line(results, Point(point2d_0(0),point2d_0(1)), point2d_2, Scalar(255,0,0), 2, 3, 0);
	circle(results, Point(point2d_0(0), point2d_0(1)),1,Scalar(0,0,0), 2);
	//Create text with the angle between normals.
	char text[50];
	strcpy(text, txtAngle);
	char deg[20];
	sprintf(deg,"%3.2f",degrees);
	strcat(text, deg);
	outDebug << "Degrees: " << degrees << ". Degrees(char): " << deg << endl;
		
	//Draw legend
	circle(results, Point(240,50), 2, Scalar(0,0,255), 2);
	putText(results, txtRef, Point(250,55),FONT_HERSHEY_PLAIN, 0.8, Scalar::all(0));
	circle(results, Point(240,80), 2, Scalar(255,0,0), 2);
	putText(results, txtNoRef, Point(250,85),FONT_HERSHEY_PLAIN, 0.8, Scalar::all(0));
	rectangle(results, Point(230, 40), Point(395, 100), Scalar::all(0));
	putText(results, text, Point(230,180),FONT_HERSHEY_PLAIN, 0.8, Scalar::all(0));

}

double PlaneCalibrationV2::calculateTranslation(const Plane* planes, const Plane* planesRef, const Matx33d* rotation, Matx31d& translation, const int numPlanes)
{
	//Create a syste N*t=D
	//N is a nxd matrix with the normals of planesRef as its rows.
	//D is a dx1 matrix. Every row is equal to distanceRef - distance*(normalRef_i*normal_i)

	double sumXX, sumXY, sumXZ, sumYY, sumYZ, sumZZ;
	double ndX, ndY, ndZ;
	sumXX = sumXY = sumXZ = sumYY = sumYZ = sumZZ = 0.0;
	ndX = ndY = ndZ = 0.0;

	for (int i = 0; i < numPlanes; i++)
	{
		Matx31d n = *(planesRef[i].getNormal());
		sumXX += n(0)*n(0);
		sumXY += n(0)*n(1);
		sumXZ += n(0)*n(2);
		sumYY += n(1)*n(1);
		sumYZ += n(1)*n(2);
		sumZZ += n(2)*n(2);

		double dotNormals = planesRef[i].getNormal()->dot(*rotation * *(planes[i].getNormal()));
		double d = planesRef[i].getDistance() - planes[i].getDistance()*dotNormals;
		ndX += n(0)*d;
		ndY += n(1)*d;
		ndZ += n(2)*d;
	}

	Matx33d LeftMatrix (sumXX,sumXY,sumXZ,sumXY,sumYY,sumYZ,sumXZ,sumYZ,sumZZ);
	Matx31d RightMatrix (ndX, ndY, ndZ); 
	translation = LeftMatrix.inv()*RightMatrix;

	//Check Translation accuracy
	return checkTranslation(planes, planesRef, rotation, &translation, numPlanes);
}

Matx31f PlaneCalibrationV2::calculateTranslation(const list<Plane>& planes_Ref, const list<Plane>& planes_NoRef, const Matx33f rotation)
{
	Matx31f translation;


	//Rotate normals from NoRef to Ref using rotation
	int numPlanes = planes_NoRef.size();
	vector<Matx31f> normals_noRef_Rot(numPlanes);
	list<Plane>::const_iterator pNoRef_iter = planes_NoRef.begin();
	int cont = 0;
	for (pNoRef_iter; pNoRef_iter != planes_NoRef.end(); pNoRef_iter++)
	{
		Matx31f n = *((*pNoRef_iter).getNormal());
		Matx31f nR = rotation*n;
		normals_noRef_Rot[cont++] = nR;
	}
	//Create matrices and equation system. Nt = D. Where

	Mat N(numPlanes, 3, CV_32F);
	int contR = 0;
	//N(nx3) = with the normals of NoRef.
	for (int i = 0; i < numPlanes; i++)
	{
		Matx31f n = normals_noRef_Rot[i];		
		float* N_ptr = N.ptr<float>(contR++);
		for (int nC = 0; nC < N.cols; nC++)
			*N_ptr++ = n(nC);
	}
	
	//D(nx1) = distance(NoRef) - distance(Ref)*(normal(NoRef_Rot)'*normal(Ref)). Each row is one plane.
	Mat D(numPlanes, 1, CV_32F);
	list<Plane>::const_iterator pRef_iter = planes_Ref.begin();
	vector<Matx31f>::iterator noRefRot_iter = normals_noRef_Rot.begin();
	pNoRef_iter = planes_NoRef.begin();
	contR = 0;
	for (pRef_iter; pRef_iter != planes_Ref.end(); pRef_iter++, noRefRot_iter++, pNoRef_iter++)
	{
		float* D_ptr = D.ptr<float>(contR++);
		const Matx13d noRefRT = (*noRefRot_iter).t();
		const Matx13d ref = (*pRef_iter).getNormal()->t();
		double t = noRefRT.dot(ref);
		*D_ptr++ = pNoRef_iter->getDistance() - pRef_iter->getDistance()*t;
	}

	Mat N_Pseudo(3, numPlanes, CV_32F);
	if (numPlanes == 3 && determinant(N) != 0) // no singular
		N_Pseudo = N.inv(DECOMP_LU);
	else
		N_Pseudo = N.inv(DECOMP_SVD); //singular

	Mat t = N_Pseudo*D;
	translation = t;

	//Check Translation accuracy
	double transAccuracy = checkTranslation(planes_Ref, planes_NoRef, rotation, translation);


	return translation;
}

//TODO Check with Graeme
double PlaneCalibrationV2::checkTranslation(const Plane* planes, const Plane* planesRef, const Matx33d* rotation, const Matx31d* translation, const int numPlanes)
{
	/*rotate and translate the centroids of planes and calculate the distance with the corresponding centroids in planesRef*/
	double d, dd;
	d = 0.0; dd = 0.0;
	for (int i = 0; i < numPlanes; i++)
	{
		Matx31d centrTrans =  *rotation * *(planes[i].getCentroid()) + *translation;
		double sum = powf(planesRef->getCentroid()->val[0] - centrTrans(0), 2) + powf(planesRef->getCentroid()->val[1] - centrTrans(1), 2) + powf(planesRef->getCentroid()->val[2] - centrTrans(2), 2);
		double dist = sqrtf(sum);
		dd += dist*dist;
		d += dist;
	}
	double mean = d/numPlanes;
	double sigma = powf( dd/numPlanes - powf(mean,2), 0.5);
//	cout << "Sigma Distance: " << sigma << endl;
	return mean;
}

double PlaneCalibrationV2::checkTranslation(const list<Plane>& planes_Ref, const list<Plane>& planes_NoRef, const Matx33f rotation, const Matx31f translation)
{
	//rotate the centroids from NoRef to Ref
	vector<Matx31d> centroidsNoRef_Rot(planes_NoRef.size());
	list<Plane>::const_iterator iterNoRef = planes_NoRef.begin();
	vector<Matx31d>::iterator iterCentrNoRef = centroidsNoRef_Rot.begin();

	while(iterNoRef != planes_NoRef.end())
	{
		*iterCentrNoRef++ = rotation * (Matx31f)(*iterNoRef->getCentroid()) + translation;
		iterNoRef++;
	}

	//calculate distances
	double sumDist = 0.0;
	list<Plane>::const_iterator iterRef = planes_Ref.begin();
	iterCentrNoRef = centroidsNoRef_Rot.begin();
	while (iterCentrNoRef != centroidsNoRef_Rot.end() && iterRef != planes_Ref.end())
	{
		sumDist += sqrt(powf((*iterCentrNoRef)(0)-(*iterRef->getCentroid())(0), 2) + powf((*iterCentrNoRef)(1)-(*iterRef->getCentroid())(1), 2) + powf((*iterCentrNoRef)(2)-(*iterRef->getCentroid())(2), 2));
		iterCentrNoRef++;
		iterRef++;
	}
	return sumDist/planes_NoRef.size();
}

void PlaneCalibrationV2::calibrateKinects(const list<Plane> planes_Ref, const list<Plane> planes_NoRef, Matx33d& rotation, Matx31d translation)
{
}