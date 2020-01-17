package com.example.eda.continuousshooting;

import android.graphics.Bitmap;
import android.os.AsyncTask;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import static org.opencv.imgproc.Imgproc.INTER_AREA;

public class LocalMapping extends AsyncTask {
    public static UseOpenCV useOpenCV;
    //Mat description1;
    ImageFeaturesStore mStore;
    int LATEST_NUMBER;
    int MAX_SLIDE = 5;
    boolean key_detection = false;

    private TrackingThred.Listener listener;

    public LocalMapping(ImageFeaturesStore store){
        mStore = store;
        useOpenCV = new UseOpenCV();
        LATEST_NUMBER = mStore.KeyImage.size()-1;
    }

    @Override
    protected Object doInBackground(Object[] objects) {
        if (LATEST_NUMBER<7)return null;
        System.out.println("LocalMapping");
            for (int i=1;i<MAX_SLIDE ;i++){
                MatOfPoint2f currentKey = new MatOfPoint2f();
                MatOfPoint2f lastKey = new MatOfPoint2f();
                Mat rotation = new Mat();
                Mat translation = new Mat();
                boolean succes = useOpenCV.opticalFlow(mStore.KeyImage.get(LATEST_NUMBER-i), mStore.KeyImage.get(LATEST_NUMBER), mStore.KeyFeatures.get(LATEST_NUMBER-i),lastKey, currentKey, rotation, translation);
                if (succes == false) continue;
                //今後平均視差が小さい物はreturn

                useOpenCV.PnP(mStore.KeyImage.get(LATEST_NUMBER-i), mStore.KeyImage.get(LATEST_NUMBER), lastKey, currentKey, rotation, translation,mStore.cameraMatrix,mStore.distcofes);
                Mat trackR =new Mat();
                Mat trackT = new Mat();
                mStore.KeyRotation.get(LATEST_NUMBER-i).copyTo(trackR);
                mStore.KeyTranslation.get(LATEST_NUMBER-i).copyTo(trackT);
                changeKeyMap(LATEST_NUMBER-i,rotation,translation,trackR,trackT);
                System.out.println("key_detection = true");
                key_detection = true;
            }
            if ( key_detection == false) removeKeyFrame();
        return null;
    }

    public void removeKeyFrame() {
        mStore.KeyImage.remove(LATEST_NUMBER);
        mStore.KeyFeatures.remove(LATEST_NUMBER);
        mStore.KeyRotation.remove(LATEST_NUMBER);
        mStore.KeyTranslation.remove(LATEST_NUMBER);
    }

    public void  changeKeyMap(int keyNumber,Mat inputR,Mat inputT,Mat trackR,Mat trackT){
        System.out.println("inputR.get(0,0)[0]==0"+inputR.dump());
        System.out.println("inputT.get(0,0)[0]==0"+inputT.dump());
        System.out.println("trackR.get(0,0)[0]==0"+trackR.dump());
        System.out.println("trackT.get(0,0)[0]==0"+trackT.dump());
        useOpenCV.calculateTrajectory(inputR,inputT, trackR,trackT);
        mStore.KeyRotation.set(keyNumber,trackR);
        mStore.KeyTranslation.set(keyNumber,trackT);
        //VRコンテンツに座標値をpostする
    }

    void setListener(TrackingThred.Listener listener) {
        this.listener = listener;
    }

    interface Listener {
        void Jpg2Bitmap(Bitmap bmp);
    }
}
