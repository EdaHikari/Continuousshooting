package com.example.eda.continuousshooting;

import android.content.Context;
import android.graphics.Bitmap;
import android.widget.Toast;

import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.calib3d.StereoSGBM;
import org.opencv.core.Core;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgproc.Imgproc;
import org.opencv.photo.Photo;
import org.opencv.video.Video;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.opencv.android.Utils.matToBitmap;
import static org.opencv.calib3d.Calib3d.RANSAC;
import static org.opencv.calib3d.Calib3d.Rodrigues;
import static org.opencv.calib3d.Calib3d.calibrateCamera;
import static org.opencv.calib3d.Calib3d.drawChessboardCorners;
import static org.opencv.calib3d.Calib3d.findEssentialMat;
import static org.opencv.calib3d.Calib3d.getOptimalNewCameraMatrix;
import static org.opencv.calib3d.Calib3d.initUndistortRectifyMap;
import static org.opencv.calib3d.Calib3d.recoverPose;
import static org.opencv.calib3d.Calib3d.solvePnP;
import static org.opencv.calib3d.Calib3d.solvePnPRansac;
import static org.opencv.calib3d.Calib3d.stereoRectify;
import static org.opencv.calib3d.Calib3d.triangulatePoints;
import static org.opencv.core.Core.normalize;
import static org.opencv.core.CvType.CV_32F;
import static org.opencv.core.CvType.CV_32FC1;
import static org.opencv.core.CvType.CV_64F;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.core.CvType.CV_8UC1;
import static org.opencv.features2d.Features2d.drawMatches;
import static org.opencv.imgproc.Imgproc.INTER_AREA;
import static org.opencv.imgproc.Imgproc.INTER_LINEAR;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.getRotationMatrix2D;
import static org.opencv.imgproc.Imgproc.line;
import static org.opencv.imgproc.Imgproc.warpAffine;

public class UseOpenCV {

    static {
        System.loadLibrary("opencv_java3");
    }

    //small tool
    public Mat onImagemaGray(Mat mat) {
        Mat gray = Mat.zeros(mat.width(),mat.height(),CV_8U);
        cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY);
        normalize(gray, gray, 0, 255, Core.NORM_MINMAX);
        return gray;
    }

    //1step Feature Detection
    public void imageStore(Mat inputImage, MatOfPoint2f inputKeyImage) {
        //System.out.println("inputImage"+inputImage.get(0,0)[0]);
        MatOfKeyPoint key1= new MatOfKeyPoint();
        FeatureDetector detector = FeatureDetector.create(FeatureDetector.AKAZE);
        detector.detect(inputImage,key1);

        MatOfPoint2f pts_prev = new MatOfPoint2f();
        KeyPoint[] keya = key1.toArray();
        for (int i = 0; i < key1.rows(); i++) {
            inputKeyImage.push_back(new MatOfPoint2f(keya[i].pt));
        }
        //if (keya.length>0)System.out.println("keya[i].pt"+keya[0].pt);
    }

    //1.5step rotate Image with gyro for get parallax average
    public Mat imageRotation(Mat inputImage, float angle){
       Point  center = new Point(inputImage.rows()/2, inputImage.cols()/2);
       Mat trans = getRotationMatrix2D(center, angle , 1.0f);
       Mat canvas = inputImage.clone();    //((int) (inputImage.rows()*1.4), (int) (inputImage.rows()*1.4),inputImage.type());
       Mat dst = new Mat();
       Imgproc.warpAffine(canvas, dst, trans, new Size(canvas.width(), canvas.height()));
       return dst;
    }

    //2step Get　First Exparameter
    public boolean opticalFlow(Mat LastImage, Mat currentImage, MatOfPoint2f pts_prev,MatOfPoint2f lastkey, MatOfPoint2f currentkey ,Mat rotation,Mat translation){

        MatOfPoint2f pts_next;
        pts_next = new MatOfPoint2f();

        MatOfByte status = new MatOfByte();
        MatOfFloat err = new MatOfFloat();

        //System.out.println("KeyImage.get(MAX_KEY-1)"+KeyFeatures.get(MAX_KEY-1).get(0,0));
        //System.out.println("KeyImage.get(MAX_KEY-2)"+KeyImage.get(MAX_KEY-2).get(0,0));

        Video.calcOpticalFlowPyrLK(LastImage, currentImage, pts_prev, pts_next, status, err);

        // 表示
        long flow_num = status.total();
        int num = 0;
        if (flow_num > 0) {
            List<Byte> list_status = status.toList();

            for (int i = 0; i < flow_num; i++) {
                if (list_status.get(i) == 1) {
                    num++;
                    Point p1 = new Point();
                    p1.x = pts_prev.get(i,0)[0];
                    p1.y = pts_prev.get(i,0)[1];
                    lastkey.push_back(new MatOfPoint2f(p1));

                    Point p2 = new Point();
                    p2.x = pts_next.get(i,0)[0];
                    p2.y = pts_next.get(i,0)[1];
                    currentkey.push_back(new MatOfPoint2f(p2));
                }
            }
        }
        if (num < 30)return false;
        Mat mask = new Mat();
        Point pP = new Point(1039.942975285893,487.6487048132928);
        double focal = 1650.632164644322;
        Mat essentialMat = findEssentialMat(lastkey, currentkey, focal, pP, RANSAC, 0.9999, 0.003, mask);
        recoverPose(essentialMat, lastkey, currentkey, rotation, translation);
        return true;
    }

    //2.5 step Calculate Exparameter
    public void calculateTrajectory(Mat currentR,Mat currentT,Mat trackR,Mat trackT){

        //Rodrigues(currentR,currentR);
        //Rodrigues(trackR,trackR);
        //System.out.println("inputR"+currentR.dump());
        //System.out.println("mImageFeaturesStore.currentR"+trackR.dump());

        for (int i= 0;i<3;i++){
            for (int j= 0;j<3;j++){
                trackR.put(i,j,
                        currentR.get(0,j)[0]*trackR.get(i,0)[0]
                                +currentR.get(1,j)[0]*trackR.get(i,1)[0]
                                +currentR.get(2,j)[0]*trackR.get(i,2)[0]
                );
            }
        }

        //回転後に移動?

        for (int i= 0;i<3;i++){
            trackT.put(i,0,
                    currentR.get(0,0)[0]*trackR.get(i,0)[0]
                            +currentR.get(1,0)[0]*trackR.get(i,1)[0]
                            +currentR.get(2,0)[0]*trackR.get(i,2)[0]
                            +currentT.get(i,0)[0]
            );
        }

    }

    //3step Make bundle adjustment to sliding window
    public void PnP(Mat lastImage,Mat currentImage,MatOfPoint2f lastkey,MatOfPoint2f currentkey,Mat currentR,Mat currentT,Mat cameraMatrix,Mat distcofes) {

        //外部パラメータの合体
        Mat prjMat1, prjMat2;
        prjMat1 = Mat.eye(3, 4, CV_64F);
        prjMat2 = Mat.eye(3, 4, CV_64F);
        for (int i = 0; i < 3; i++) {
            prjMat2.put(i,3,currentT.get(i,0)[0]);
            for (int j = 0; j < 3; j++) {
                prjMat2.put(i,j,currentR.get(i,j)[0]);
            }
        }

        MatOfPoint3f points4D = new MatOfPoint3f();
        triangulatePoints(prjMat1, prjMat2,lastkey,currentkey,points4D);
        MatOfPoint3f points3D = new MatOfPoint3f();
        double x, y, z;
        for (int i = 0; i < points4D.cols(); i++)
        {
            x = points4D.get(0,i)[0]/points4D.get(3,i)[0];
            y = points4D.get(1,i)[0]/points4D.get(3,i)[0];
            z = points4D.get(2,i)[0]/points4D.get(3,i)[0];
            points3D.push_back(new MatOfPoint3f(new Point3(x, y, z)));
        }

        MatOfDouble D = new MatOfDouble();
        D.push_back(distcofes);
        Mat temp = new Mat();
        solvePnPRansac(points3D,lastkey,cameraMatrix,D,temp,currentT);
        //System.out.println("currentR"+currentR.dump());
        Rodrigues(temp,currentR);
    }

    //4step Loop Closing Detection
    public void matchingImages(Mat LastImage,Mat currentImage,Mat LastDescrip,Mat currentDescrip, MatOfKeyPoint Lastkey, MatOfKeyPoint currentkey,Mat rotation,Mat translation){
        DescriptorMatcher matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE);
        MatOfDMatch matche = new MatOfDMatch();
        MatOfDMatch dmatche12 = new MatOfDMatch();
        MatOfDMatch dmatche21 = new MatOfDMatch();
        List<DMatch> dmatch = new ArrayList<DMatch>();

        matcher.match(currentDescrip, LastDescrip,dmatche12);
        matcher.match( LastDescrip,currentDescrip,dmatche21);
        List<DMatch> ldm_srcToBase = dmatche12.toList();
        List<DMatch> ldm_baseToSrc = dmatche21.toList();

        for(int i=0; i<ldm_srcToBase.size(); i++) {

            DMatch forward = ldm_srcToBase.get(i);
            DMatch backward = ldm_baseToSrc.get(forward.trainIdx);

            if (backward.trainIdx == forward.queryIdx) {
                dmatch.add(forward);
            }
        }
        matche.fromList(dmatch);

        Mat output = new Mat(LastImage.rows()*2,LastImage.cols()*2,LastImage.type());
        //drawMatches(KeyImage.get(KeyImage.size()-1), KeyFeatures2.get(KeyImage.size()-1), KeyImage.get(KeyImage.size()-2),  KeyFeatures2.get(KeyImage.size()-2), matche, output);

        if(dmatch.size()<30) return;

        List<KeyPoint> keypoints1_l = currentkey.toList();
        List<KeyPoint> keypoints2_l = Lastkey.toList();

        MatOfPoint2f pP1 = new MatOfPoint2f();
        MatOfPoint2f pP2 = new MatOfPoint2f();

        for (int i = 0; i < dmatch.size(); i++) {
            DMatch forward = dmatch.get(i);
            pP1.push_back(new MatOfPoint2f(keypoints1_l.get(forward.queryIdx).pt));
            pP2.push_back(new MatOfPoint2f(keypoints2_l.get(forward.trainIdx).pt));
        }

        Mat mask = new Mat();
        Point pP = new Point(1039.942975285893,487.6487048132928);
        double focal = 1650.632164644322;
        Mat essentialMat = findEssentialMat(pP1, pP2, focal, pP, RANSAC, 0.9999, 1, mask);
        recoverPose(essentialMat, pP1, pP2, rotation, translation);
    }

    //Advance preparation 1step
    public Mat onCalibrate(Mat inputFrame, Mat cameraMatrix,Mat distcofes) {
        int horizonalCrossCount = 7;
        int verticalCrossCount = 10;

        List<Mat> image_point = new ArrayList<Mat>();
        MatOfPoint3f objects = new MatOfPoint3f();
        List<Mat> objcts_point = new ArrayList<Mat>();
        for (int i = 0; i <verticalCrossCount; i++) {
            for (int j = 0; j < horizonalCrossCount; j++) {
                objects.push_back(new MatOfPoint3f(new Point3(i*20, j *20, 0.0f)));
            }
        }

        MatOfPoint2f mCorners = new MatOfPoint2f();
        TermCriteria criteria = new TermCriteria(TermCriteria.MAX_ITER | TermCriteria.EPS, 30, 0.1);
        Size sizea = new Size(horizonalCrossCount, verticalCrossCount);
        Size sizeb = new Size(11, 11);
        Size sizec = new Size(-1, -1);
        List<Mat> t = new ArrayList<>();
        List<Mat> r = new ArrayList<>();

        boolean found = Calib3d.findChessboardCorners(inputFrame, sizea, mCorners);

        if (!found) return inputFrame;
        else {
            //Imgproc.cornerSubPix(inputFrame, mCorners, sizeb, sizec, criteria);
            image_point.add(mCorners);
            objcts_point.add(objects);
            drawChessboardCorners(inputFrame, sizea, mCorners, found);

            double rms = calibrateCamera(objcts_point, image_point, inputFrame.size(), cameraMatrix, distcofes, r, t);//CALIB_USE_INTRINSIC_GUESS
            cameraMatrix.put(5,1,0);
            Mat undisort = new Mat();
            Imgproc.undistort(inputFrame, undisort, cameraMatrix, distcofes, cameraMatrix);
            return undisort;
        }
    }

    //Advance preparation 2step
    public Mat finalCalibrate(Context mContext, Mat inputFrame, Mat cameraMatrix,Mat distcofes, int number) {

        int horizonalCrossCount = 7;
        int verticalCrossCount = 10;
        int Flags = Calib3d.CALIB_FIX_PRINCIPAL_POINT +
                Calib3d.CALIB_ZERO_TANGENT_DIST +
                Calib3d.CALIB_FIX_ASPECT_RATIO +
                Calib3d.CALIB_FIX_K4 +
                Calib3d.CALIB_FIX_K5;

        List<Mat> image_point = new ArrayList<Mat>();
        MatOfPoint3f objects = new MatOfPoint3f();
        List<Mat> objcts_point = new ArrayList<Mat>();

        for (int i = 0; i <verticalCrossCount; i++) {
            for (int j = 0; j < horizonalCrossCount; j++) {
                objects.push_back(new MatOfPoint3f(new Point3(i*20, j *20, 0.0f)));
            }
        }

        MatOfPoint2f corners = new MatOfPoint2f();
        distcofes = Mat.zeros(1, 4, CV_32F);
        cameraMatrix = Mat.zeros(3, 3, CV_32F);
        List<Mat> t = new ArrayList<>();
        List<Mat> r = new ArrayList<>();

        DataRecode dataMethod = new DataRecode(mContext,"imagePointsFile");
        for (int i= 1;i<number;i++) {
            Mat co = dataMethod.loadData("imagePoints"+Integer.toString(i));
            corners = new MatOfPoint2f(co);
            image_point.add(corners);
            objcts_point.add(objects);
        }
        double rms = calibrateCamera(objcts_point, image_point, inputFrame.size(), cameraMatrix, distcofes, r, t);
        Mat undisort = new Mat();
        Imgproc.undistort(inputFrame, undisort, cameraMatrix, distcofes, cameraMatrix);

        dataMethod = new DataRecode(mContext,"caliblationParametetFile");
        dataMethod.writeData("cameraMatrix",cameraMatrix);
        dataMethod.appendData("distcofes",distcofes);
        return undisort;

    }

    public Mat onStereo(Mat left,Mat right) {
        Mat rightgray =  onImagemaGray(right);
        Mat leftgray =  onImagemaGray(left);

        Mat mdisparity = Mat.zeros(rightgray.width(),rightgray.height(),CV_8U);

        Mat undistortright = rightgray.clone();
        Mat undistortleft = leftgray.clone();

        StereoSGBM stereo = StereoSGBM.create(

                0,  //一般には０
                16, //16の倍数
                11, //3~11の奇数
                0,//デフォルトが良い　0
                0,//デフォルトが良い　0
                1,//デフォルトが良い　0
                0,//デフォルトが良い　0
                0,//デフォルトが良い　0
                3,//デフォルトの0は使用しない意、使って効果あり
                1,//デフォルトが良い　1
                StereoSGBM.MODE_SGBM

        );
        Imgproc.resize(undistortleft, undistortleft, undistortright.size());
        Photo.fastNlMeansDenoising(undistortright,undistortright);
        Photo.fastNlMeansDenoising(undistortleft,undistortleft);
        Imgproc.resize(mdisparity, mdisparity, undistortright.size());
        stereo.compute(undistortleft, undistortright, mdisparity);

        normalize(mdisparity,undistortright,0,255, Core.NORM_MINMAX,CV_8UC1);
        // display the result
        cvtColor(undistortright, undistortleft, Imgproc.COLOR_GRAY2BGRA, 4);
        Imgproc.resize(undistortleft, undistortright, undistortright.size());
        return undistortleft;
    }

    public Mat foundEssentialMat(Mat left,Mat right,Mat rotation,Mat translation) {

        FeatureDetector detector = FeatureDetector.create(FeatureDetector.AKAZE);
        DescriptorExtractor extractor = DescriptorExtractor.create(DescriptorExtractor.AKAZE);
        DescriptorMatcher matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);
        MatOfKeyPoint key1 = new MatOfKeyPoint();
        MatOfKeyPoint key2 = new MatOfKeyPoint();
        Scalar color = new Scalar(0,0,255);

        Mat description1 = new Mat(right.rows(),right.cols(),right.type());
        Mat description2 = new Mat(left.rows(),left.cols(),left.type());

        detector.detect(right,key1);
        detector.detect(left,key2);
        extractor.compute(right,key1,description1);
        extractor.compute(left,key2,description2);

        MatOfDMatch matche = new MatOfDMatch();
        MatOfDMatch dmatche12 = new MatOfDMatch();
        MatOfDMatch dmatche21 = new MatOfDMatch();
        List<DMatch> dmatch = new ArrayList<DMatch>();

        matcher.match(description1,description2,dmatche12);
        matcher.match(description2,description1,dmatche21);
        List<DMatch> ldm_srcToBase = dmatche12.toList();
        List<DMatch> ldm_baseToSrc = dmatche21.toList();

        for(int i=0; i<ldm_srcToBase.size(); i++) {

            DMatch forward = ldm_srcToBase.get(i);
            DMatch backward = ldm_baseToSrc.get(forward.trainIdx);

            if (backward.trainIdx == forward.queryIdx) {
                dmatch.add(forward);
            }
        }
        matche.fromList(dmatch);

        Mat output = new Mat(right.rows()*2,right.cols()*2,right.type());
        drawMatches(right, key1, left, key2, matche, output);

        List<KeyPoint> keypoints1_l = key1.toList();
        List<KeyPoint> keypoints2_l = key2.toList();

        MatOfPoint2f pP1 = new MatOfPoint2f();
        MatOfPoint2f pP2 = new MatOfPoint2f();
        Point p = new Point();
        Mat ip =  new Mat(3,1, CV_32F);
        //Toast.makeText(mContext,"特徴点の数"+dmatch.size(),Toast.LENGTH_LONG).show();

        if(dmatch.size()<200) return output;

        for (int i = 0; i < dmatch.size(); i++) {
            DMatch forward = dmatch.get(i);
            pP1.push_back(new MatOfPoint2f(keypoints1_l.get(forward.queryIdx).pt));
            pP2.push_back(new MatOfPoint2f(keypoints2_l.get(forward.trainIdx).pt));
        }

        Mat mask = new Mat();
        Point pP = new Point(1039.942975285893,487.6487048132928);
        double focal = 1650.632164644322;
        Mat essentialMat = findEssentialMat(pP1, pP2, focal, pP, RANSAC, 0.9999, 1, mask);
        recoverPose(essentialMat, pP1, pP2, rotation, translation);
        //solvePnp
        /*
        Mat rvec= Mat.zeros(3, 1, CV_32F);
        Rodrigues(lastR,rvec);
        Mat prjMat1, prjMat2;
        prjMat1 = Mat.eye(3, 4, CV_64F);
        prjMat2 = Mat.eye(3, 4, CV_64F);
        for (int i = 0; i < 3; i++) {
            prjMat2.put(i,3,lastT.get(i,0)[0]);
            for (int j = 0; j < 3; j++) {
                prjMat2.put(i,j,rvec.get(i,0)[0]);
            }
        }

        MatOfPoint3f points4D = new MatOfPoint3f();
        triangulatePoints(prjMat1, prjMat2,pP1,pP2,points4D);
        MatOfPoint3f points3D = new MatOfPoint3f();
        double x, y, z;
        for (int i = 0; i < points4D.cols(); i++)
        {
            x = points4D.get(0,i)[0]/points4D.get(3,i)[0];
            y = points4D.get(1,i)[0]/points4D.get(3,i)[0];
            z = points4D.get(2,i)[0]/points4D.get(3,i)[0];
            points3D.push_back(new MatOfPoint3f(new Point3(x, y, z)));
        }

        MatOfDouble D = new MatOfDouble();
        D.push_back(distcofes);
        solvePnP(points3D,pP2,cameraMatrix,D,rvec,lastT);
        Rodrigues(rvec,lastR);
        //solvePnp
*/
        //return forEssentialStereo(left,right,lastT,lastR, cameraMatrix,distcofes);
        return output;
    }

    public Mat forEssentialStereo(Mat left,Mat right,Mat translation,Mat rotation,Mat cameraMatrix,Mat distcofes) {

        Mat R1= new Mat(),R2= new Mat(),P1= new Mat(),P2= new Mat(),Q = new Mat();
        stereoRectify(cameraMatrix,distcofes,cameraMatrix,distcofes,right.size(),rotation,translation,R1,R2,P1,P2,Q, 1);
        Mat mapx= new Mat(), mapy = new Mat();
        initUndistortRectifyMap(cameraMatrix,distcofes,R1,P1,left.size(),CV_32FC1,mapx,mapy);

        Mat undistort = new Mat();
        Imgproc.remap(left, undistort, mapx, mapy, INTER_LINEAR);
        /*
        Mat mdisparity = Mat.zeros(left.width(),left.height(),CV_8U);

        StereoSGBM stereo = StereoSGBM.create(

                0,//0 視差の下限
                16,//16　視差の上限
                11,//11　SADの窓のサイズ
                0,//0　視差の滑らかさを制御するパラメータ1
                0,//0　視差の滑らかさを制御sるパラメータ2
                1,//0 視差チェックにおいて許容される最大の差
                0,
                0,//10　パーセント単位で表現されるマージン
                3,//100　視差領域の最大のサイズ
                1,//0　それぞれの連結成分における最大の差
                StereoSGBM.MODE_SGBM
        );

        Size sz = new Size();
        //Imgproc.resize(undistortright, undistortright,sz,0.4,0.6,INTER_AREA);
        Imgproc.resize(left, left, undistort.size());
        Imgproc.resize(mdisparity, mdisparity, undistort.size());
        stereo.compute(left, undistort, mdisparity);
        Core.normalize(mdisparity,undistort,0,255, Core.NORM_MINMAX,CV_8UC1);
         */

        return undistort;
    }

    public Mat CenterOfGravity(Mat src) {
        List contours = new ArrayList();
        Mat hierarchy = Mat.zeros(new Size(5, 5), CV_8UC1);
        Imgproc.threshold(src, src, 100, 255, THRESH_BINARY);
        findContours(src, contours, hierarchy,Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_TC89_L1);
        return src;
    }

    public Mat Mask(Mat stereoImg,Mat inputImg){

        Imgproc.resize(inputImg, inputImg, inputImg.size());
        Imgproc.resize(stereoImg, stereoImg, inputImg.size());


        Mat mask = Mat.zeros(inputImg.width(),inputImg.height(),CV_8U);
        Scalar low = new Scalar( 0,0,180);//下限(H,S,V)
        Scalar high = new Scalar(0,0,245);//上限(H,S,V)
        cvtColor(inputImg, mask, Imgproc.COLOR_RGB2HSV);
        Core.inRange( mask,  low,  high , mask);
        //Imgproc.cvtColor(inputFrame, mask, Imgproc.);
        //threshold(inputFrame,mask,180,255,THRESH_BINARY);

        erode(mask,mask,new Mat());
        dilate(mask,mask,new Mat());

        inputImg.copyTo(stereoImg,mask);

        return mask;
    }
    /*
    public Bitmap startOpenCV(Bitmap bmp) {
        Mat inputImg = new Mat();
        Utils.bitmapToMat(bmp, inputImg);

        //Imgproc.resize(inputImg, inputImg,new Size(),0.1,0.1,INTER_AREA);

        //Mat outputImg = onCalibrate(inputImg);
        Mat outputImg = finalCalibrate(inputImg,15);
        //Mat maskImg = Mask(stereoImg);

        Bitmap mbitmap = Bitmap.createBitmap(outputImg.width(), outputImg.height(), Bitmap.Config.ARGB_8888);
        matToBitmap(outputImg,mbitmap);
        return mbitmap;
    }

 */

/*
    public Bitmap stereoReady(Mat lastImage) {
        Mat outputImg;

            //Imgproc.resize(inputImg, inputImg,new Size(),0.1,0.1,INTER_AREA);
            //Mat outputImg = onStereo(inputLImg,inputRImg);
            //outputImg = foundEssentialMat(lastImage,currentImage);
            outputImg = opticalFlow();
            //outputImg = imageRotation(currentImage,agree);
            //outputImg = onStereo(lastImage,outputImg);

        Bitmap mbitmap = Bitmap.createBitmap(outputImg.width(), outputImg.height(), Bitmap.Config.ARGB_8888);
        matToBitmap(outputImg,mbitmap);
        return mbitmap;
    }

 */
}
