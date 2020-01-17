package com.example.eda.continuousshooting;

import android.graphics.Bitmap;
import android.os.AsyncTask;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import static org.opencv.core.CvType.CV_32FC1;
import static org.opencv.imgproc.Imgproc.INTER_AREA;

public class TrackingThred extends AsyncTask {

    public static Bitmap mBitmap;
    public static UseOpenCV useOpenCV;
    //Mat description1;
    Mat currentImage;
    ImageFeaturesStore mStore;

    private Listener listener;

    public TrackingThred(Bitmap bmp,ImageFeaturesStore store){
        mBitmap = bmp;
        mStore = store;
        useOpenCV = new UseOpenCV();
        //description1 = inputDesc;
        currentImage = new Mat();
        Utils.bitmapToMat(bmp, currentImage);
        Imgproc.resize(currentImage, currentImage,new Size(),0.3,0.3,INTER_AREA);
    }

    @Override
    protected Object doInBackground(Object[] objects) {
        if (!mStore.currentImage.empty()) {
            //System.out.println("mStore.currentImage"+mStore.currentImage.get(0,0)[0]);
            if (mStore.currentFeatures.rows() < 30) {
                useOpenCV.imageStore(mStore.currentImage,mStore.currentFeatures);
            }
            if (mStore.currentFeatures.rows() < 30) {
                copyCurrentData(currentImage,mStore.currentFeatures);
                //System.out.println("mStore.currentFeatures.rows() < 30");
                return null;
            }
            MatOfPoint2f currentKey = new MatOfPoint2f();
            MatOfPoint2f lastKey = new MatOfPoint2f();
            Mat rotation = new Mat();
            Mat translation = new Mat();
            boolean succes = useOpenCV.opticalFlow(mStore.currentImage, currentImage, mStore.currentFeatures,lastKey, currentKey, rotation, translation);
            if (succes == false){
                copyCurrentData(currentImage,currentKey);
                //System.out.println("succes == false");
                return null;
            }
            copyKeyFrame(mStore.currentImage,lastKey);
            copyLastMap(rotation, translation);
            copyCurrentData(currentImage,currentKey);
            listener.setKeyFrame();
        }else {
            MatOfPoint2f newKey= new MatOfPoint2f();
            useOpenCV.imageStore(currentImage,newKey);
            copyCurrentData(currentImage,newKey);
        }
        return null;
    }

    public void copyKeyFrame(Mat inputImage, MatOfPoint2f inputKey) {
        //今後平均視差を求める
        //平均視差が閾値より低ければreturn
        mStore.KeyImage.add(inputImage);
        mStore.KeyFeatures.add(inputKey);
        mStore.KeyRotation.add(mStore.currentR);
        mStore.KeyTranslation.add(mStore.currentT);
    }

    public void  copyLastMap(Mat inputR,Mat inputT){
        if (mStore.currentT.get(0,0)[0]==0) {
            inputT.copyTo(mStore.currentT);
            inputR.copyTo(mStore.currentR);
            //System.out.println("mStore.currentT.get(0,0)[0]==0"+mStore.currentT.dump());
            return;
        }
        if (!inputR.empty() && !inputT.empty())
            useOpenCV.calculateTrajectory(inputR,inputT, mStore.currentR,mStore.currentT);
            //System.out.println("mStore.currentR"+mStore.currentR.dump());
    }

    public void copyCurrentData(Mat inputImage, MatOfPoint2f inputKey) {
        inputImage.copyTo(mStore.currentImage);
        inputKey.copyTo(mStore.currentFeatures);
    }

    void setListener(Listener listener) {
        this.listener = listener;
    }

    interface Listener {
        void setKeyFrame();
    }
}
