package com.example.eda.continuousshooting;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.widget.ImageView;

import org.opencv.android.Utils;
import org.opencv.calib3d.StereoSGBM;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.photo.Photo;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.opencv.android.Utils.matToBitmap;
import static org.opencv.core.Core.normalize;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.core.CvType.CV_8UC1;
import static org.opencv.imgproc.Imgproc.INTER_AREA;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;

public class UseOpenCV {
    private File mFile;
    static {
        System.loadLibrary("opencv_java3");
    }


    public Bitmap startOpenCV(Bitmap bmp) {

        Mat inputImg = new Mat();
        Utils.bitmapToMat(bmp, inputImg);

        Imgproc.resize(inputImg, inputImg,new Size(),0.1,0.1,INTER_AREA);
        Mat stereoImg = onStereo(inputImg,inputImg);
        //Mat maskImg = Mask(stereoImg);

        Bitmap mbitmap = Bitmap.createBitmap(stereoImg.width(), stereoImg.height(), Bitmap.Config.ARGB_8888);
        matToBitmap(inputImg,mbitmap);
        return mbitmap;
    }

    public Mat onImagemaGray(Mat mat) {
        Mat gray = Mat.zeros(mat.width(),mat.height(),CV_8U);
        cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY);
        normalize(gray, gray, 0, 255, Core.NORM_MINMAX);
        return gray;
    }

    public Mat onStereo(Mat right,Mat left) {
        Mat rightgray =  onImagemaGray(right);
        Mat leftgray =  onImagemaGray(left);

        Mat mdisparity = Mat.zeros(rightgray.width(),rightgray.height(),CV_8U);

        Mat undistortright = new Mat();
        Mat undistortleft = new Mat();
        undistortright = rightgray;
        undistortleft = leftgray;

        StereoSGBM stereo = StereoSGBM.create(

                0,  //一般には０
                96, //16の倍数
                3, //3~11の奇数
                480,//デフォルトが良い　0
                240,//デフォルトが良い　0
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
}
