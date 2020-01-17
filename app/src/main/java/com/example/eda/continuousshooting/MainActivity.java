package com.example.eda.continuousshooting;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.media.Image;
import android.media.ImageReader;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.GLUtils;
import android.opengl.Matrix;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Bundle;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import org.opencv.android.Utils;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Size;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import static org.opencv.imgproc.Imgproc.INTER_AREA;

public class MainActivity extends Activity implements SensorEventListener {

    private Context mContext;
    private TextureView mTextureView;
    private CameraCaptureSession mCameraSession;
    private Image mImage;
    private CameraDevice mCameraDevice;
    private String mCameraId;
    private CaptureRequest.Builder mPreviewRequestBuilder;
    private CaptureRequest mPreviewRequest;
    private TextView rmsText;
    private TextView cameraMatrixText;
    private TextView distcofesText;
    private ImageView mPreviewView;
    private ImageReader mImageReader;
    private CameraCaptureSession mCaptureSession;
    private HandlerThread mBackgroundThread;
    private Handler mBackgroundHandler;
    private int mState = 0;
    private static final int STATE_PREVIEW = 0;
    private static final int STATE_WAITING_LOCK = 1;
    private static final int STATE_WAITING_PRECAPTURE = 2;
    private static final int STATE_WAITING_NON_PRECAPTURE = 3;
    private static final int STATE_PICTURE_TAKEN = 4;
    private Bitmap mBitmap = Bitmap.createBitmap(1080, 1920, Bitmap.Config.ARGB_8888);
    private CreateTexture mCreateTexture;
    private boolean textureFlag = false;
    private TimerTask mCameraTrigger;
    private Timer timer;
    private DataRecode dataMethod;
    private ImageFeaturesStore mImageFeaturesStore;
    private  TrackingThred mTrackinghread;
    private LocalMapping mLocalMapping;

    private SensorManager sensorManager;
    private static final float alpha = 0.8f;
    private static final int MATRIX_SIZE = 16;
    private static final int SENSOR_DELAY = SensorManager.SENSOR_DELAY_FASTEST;
    private float[] rawacc = new float[3];
    private float[] acc  = new float[3];
    private float[] grav = new float[3];
    private float[] mag  = new float[3];
    private float[] lastori  = new float[3];
    private float[] currentori  = new float[3];

    float[] exp  = new float[6];
    List<Float> listX = new ArrayList<>();
    List<Float> listY = new ArrayList<>();
    List<Float> listZ = new ArrayList<>();
    int MAX_LIST = 4;
    float Calpha = 0.3f;
    float Cbeta = (1-alpha)/MAX_LIST;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        mContext = getApplicationContext();
        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mImageFeaturesStore = new ImageFeaturesStore((MainActivity)this);

        for (int i=0; i<MAX_LIST;i++){
            listX.add(0f);
            listY.add(0f);
            listZ.add(0f);
        }

        int layoutId = getResources().getIdentifier("activity_main", "layout", getPackageName());
        setContentView(layoutId);
        int textureId = getResources().getIdentifier("textureView", "layout", getPackageName());
        mTextureView = (TextureView) findViewById(R.id.textureView);
        mPreviewView = (ImageView) findViewById(R.id.imageView);
        rmsText = (TextView) findViewById(R.id.rmsTextView);
        cameraMatrixText = (TextView)findViewById(R.id.cameraMatrixTextView);
        distcofesText = (TextView)findViewById(R.id.distcofesTextView);

        Button tButton = (Button)findViewById(R.id.picture_button);
        tButton.setOnClickListener(mOnClicklistener);
        Button sButton = (Button)findViewById(R.id.save_button);
        sButton.setOnClickListener(mOnClicklistener);

        CameraManager manager = (CameraManager) this.getSystemService(Context.CAMERA_SERVICE);
        try {
            for (String cameraId : manager.getCameraIdList()) {
                CameraCharacteristics chars
                        = manager.getCameraCharacteristics(cameraId);
                Integer facing = chars.get(CameraCharacteristics.LENS_FACING);
                float[] cameraMtrix = chars.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION);
                System.out.println("cameraMtrix!!!!!!!!!!!!!!!!!!"+ Arrays.toString(cameraMtrix));
                if (facing != null && facing ==
                        CameraCharacteristics.LENS_FACING_BACK) {
                    mCameraId = cameraId;
                }
            }
            System.out.println("CameraManager manager!!!!!!!!!!!!!!!!!!");
            if (this.checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {            }
            manager.openCamera(mCameraId, mStateCallback, mBackgroundHandler);
            mImageReader = ImageReader.newInstance(1080, 1920, ImageFormat.JPEG,5);
            mImageReader.setOnImageAvailableListener(mOnImageAvailableListener, mBackgroundHandler);

        } catch (CameraAccessException e) {
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        Sensor accel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        sensorManager.registerListener(this, accel, SensorManager.SENSOR_DELAY_NORMAL);
        Sensor jyro = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        sensorManager.registerListener(this, jyro, SensorManager.SENSOR_DELAY_NORMAL);

        mBackgroundThread = new HandlerThread("CameraBackground");
        mBackgroundThread.start();
        mBackgroundHandler = new Handler(mBackgroundThread.getLooper());
        timer = new Timer();
        mCameraTrigger = new CameraTrigger(MainActivity.this, mBackgroundHandler);
    }

    @Override
    protected void onPause() {
        super.onPause();

        if (mCameraSession != null) {
            try {
                mCameraSession.stopRepeating();
            } catch (CameraAccessException e) {
            }
            mCameraSession.close();
        }

        // カメラデバイスとの切断
        if (mCameraDevice != null) {
            mCameraDevice.close();
        }
    }

    private final CameraDevice.StateCallback mStateCallback = new CameraDevice.StateCallback() {
        @Override
        public void onOpened(CameraDevice cameraDevice) {
            mCameraDevice = cameraDevice;
            startPreview();
        }
        @Override
        public void onDisconnected(CameraDevice cameraDevice) {
            mCameraDevice.close();
            mCameraDevice = null;
        }
        @Override
        public void onError(CameraDevice cameraDevice, int error) {
            mCameraDevice.close();
            mCameraDevice = null;
        }
    };

    private CameraCaptureSession.CaptureCallback mCaptureCallback
            = new CameraCaptureSession.CaptureCallback() {

        private void process(CaptureResult result) {
            if(mState == STATE_PICTURE_TAKEN)  takepicture();
        }

        @Override
        public void onCaptureProgressed(CameraCaptureSession session, CaptureRequest request, CaptureResult partialResult) {
            process(partialResult);
        }

        @Override
        public void onCaptureCompleted(CameraCaptureSession session,CaptureRequest request,TotalCaptureResult result) {
            process(result);
        }
    };

    private CameraCaptureSession.StateCallback mCaptureSessionCallback
            = new CameraCaptureSession.StateCallback() {
        @Override
        public void onConfigured( CameraCaptureSession cameraCaptureSession) {
            if (null == mCameraDevice)  return;

            mCaptureSession = cameraCaptureSession;
            statejudge();
            timer.scheduleAtFixedRate(mCameraTrigger, 0, 200);
        }
        @Override
        public void onConfigureFailed(
                CameraCaptureSession cameraCaptureSession) {}
    };


    View.OnClickListener mOnClicklistener =new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            switch ( view.getId() ){
                case R.id.picture_button:{
                    mState = STATE_PICTURE_TAKEN;
                    statejudge();
                    String strTmp = "回転\n"
                            + " X: " + (acc[0]) + "\n"
                            + " Y: " + (acc[1]) + "\n"
                            + " Z: " + (acc[2]);
                    rmsText.setText(strTmp);
                    break;
                }
                case R.id.save_button:{
                    //postExternalParameters();
                    String strTmp = "回転\n"
                            + " X: " + (mImageFeaturesStore.currentT.get(0,0)[0]) + "\n"
                            + " Y: " + (mImageFeaturesStore.currentT.get(1,0)[0]) + "\n"
                            + " Z: " + (mImageFeaturesStore.currentT.get(2,0)[0]);
                    cameraMatrixText.setText(strTmp);
                    //mOpen.agree = currentori[2]-lastori[2];
                    //lastori = currentori.clone();
                    //mPreviewView.setImageBitmap(mOpen.stereoReady());
                    //rmsText.setText("rotation" + mOpen.lastR.dump());
                    //cameraMatrixText.setText("translation" + mOpen.lastT.dump());

                    //dataMethod = new DataRecode(MainActivity.this,"imagePointsFile");
                    //dataMethod.appendData("imagePoints",mOpen.mCorners);
                    //Toast.makeText(MainActivity.this,"save imagePoints",Toast.LENGTH_LONG).show();
                    break;
                }
            }
        }
    };

    private void startPreview() {

        if (null == mCameraDevice)  return;

        try {
            mPreviewRequestBuilder = mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);
            mPreviewRequestBuilder.addTarget(mImageReader.getSurface());
            mCameraDevice.createCaptureSession(Arrays.asList(mImageReader.getSurface()),mCaptureSessionCallback,null);

        } catch (CameraAccessException e) {
        }
    }

    ////////////////////////撮影まじか//////////////////////////////////////////////////////////////////////////////
    //lookfocusに相当
    public void statejudge(){
        try {
            mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AF_TRIGGER, CameraMetadata.CONTROL_AF_TRIGGER_START);
            mState = STATE_PICTURE_TAKEN;
            mCaptureSession.capture(mPreviewRequestBuilder.build(), mCaptureCallback, mBackgroundHandler);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    //captureStillPictureに相当
    private void takepicture(){
        try {
            if (null == mCameraDevice) {
                return;
            }

            final CaptureRequest.Builder captureBuilder =
                    mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_STILL_CAPTURE);
            captureBuilder.addTarget(mImageReader.getSurface());

            captureBuilder.set(CaptureRequest.CONTROL_AF_MODE,CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);

            CameraCaptureSession.CaptureCallback CaptureCallback
                    = new CameraCaptureSession.CaptureCallback() {
                @Override
                public void onCaptureCompleted(CameraCaptureSession session,CaptureRequest request,TotalCaptureResult result) {
                    try {
                        mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AF_TRIGGER,CameraMetadata.CONTROL_AF_TRIGGER_CANCEL);
                        mCaptureSession.capture(mPreviewRequestBuilder.build(), mCaptureCallback,mBackgroundHandler);
                        mState = STATE_PREVIEW;
                    } catch (CameraAccessException e) {
                        e.printStackTrace();
                    }
                }
            };
            mCaptureSession.abortCaptures();
            mCaptureSession.capture(captureBuilder.build(), CaptureCallback, mBackgroundHandler);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    ///////////////////////////////////Runnable//////////////////////////////////////////////////////////
    private final ImageReader.OnImageAvailableListener mOnImageAvailableListener
            = new ImageReader.OnImageAvailableListener() {
        @Override
        public void onImageAvailable(ImageReader reader) {
            mBackgroundHandler.post(new ImageSaver(reader.acquireNextImage()));
            if (ImageSaver.bmp != null) {
                //mPreviewView.setImageBitmap(ImageSaver.bmp);
                mTrackinghread = new TrackingThred(ImageSaver.bmp,mImageFeaturesStore);
                mTrackinghread.setListener(createListener());
                mTrackinghread.execute(0);
            }
            //mImageSaver = new ImageSaver(reader.acquireNextImage());
            //mImageSaver.setListener(imageListener());
            //mImageSaver.execute(0);
        }

    };

    private static class ImageSaver implements Runnable {

        private final Image mImage;
        public static Bitmap bmp;

        ImageSaver(Image image) {
            mImage = image;
        }
        @Override
        public void run() {
            ByteBuffer buffer = mImage.getPlanes()[0].getBuffer();
            byte[] bytes = new byte[buffer.remaining()];
            buffer.get(bytes);
            mImage.close();
            bmp= BitmapFactory.decodeByteArray(bytes,0,bytes.length);
        }
    }


/*
    private static class  FeatureStore implements Runnable {

        public static Bitmap mBitmap;
        public static UseOpenCV useOpenCV;
        MatOfPoint2f lastKey;
        //Mat description1;
        Mat lastImage;
        Mat currentImage;

        private Listener listener;

        public FeatureStore(Bitmap bmp,Mat inputImage,MatOfPoint2f inputKey){
            mBitmap = bmp;
            useOpenCV = new UseOpenCV();
            //description1 = inputDesc;
            currentImage = new Mat();
            Utils.bitmapToMat(bmp, currentImage);
            Imgproc.resize(currentImage, currentImage,new Size(),0.3,0.3,INTER_AREA);
            lastKey =inputKey;
            lastImage = inputImage;
        }

        @Override
        public void run() {
            if (!lastImage.empty()) {

                if (lastKey.rows() < 30) {
                    useOpenCV.imageStore(lastImage, lastKey);
                }

                MatOfPoint2f currentKey = new MatOfPoint2f();
                //Mat description1 = new Mat();
                Mat rotation = new Mat();
                Mat translation = new Mat();
                useOpenCV.opticalFlow(lastImage, currentImage, lastKey, currentKey, rotation, translation);
                listener.copyLastMap(lastImage,lastKey);
                listener.copyLastMap(rotation, translation);
                listener.copyCurrentData(currentImage,currentKey);
            }else {
                MatOfPoint2f currentKey = new MatOfPoint2f();
                useOpenCV.imageStore(currentImage, currentKey);
                listener.copyCurrentData(currentImage,currentKey);
            }
        }
        void setListener(Listener listener) {
            this.listener = listener;
        }

        interface Listener {
            //キーフレームの画像、特徴点、座標の保存
            void copyKeyFrame(Mat inputImage, MatOfPoint2f inputKey);
            //これまでの軌跡の計算
            void copyLastMap(Mat inputR,Mat inputT);
            //最新画像、特徴点の更新
            void copyCurrentData(Mat inputImage,MatOfPoint2f inputKey);
        }
    }

 */

    ///////////////////TrackingThredListener//////////////////////////
 /*
    private ImageSaver.Listener imageListener(){
        return new ImageSaver.Listener() {
            @Override
            public void Jpg2Bitmap(Bitmap bmp) {
                mTrackinghread = new TrackingThred(ImageSaver.bmp,mImageFeaturesStore.currentImage,mImageFeaturesStore.currentFeatures);
                mTrackinghread.setListener(createListener());
                mTrackinghread.execute(0);
            }
        };
    }

  */


    private TrackingThred.Listener createListener() {
        return new TrackingThred.Listener() {
            @Override
            public void setKeyFrame() {
                //System.out.println("setKeyFrame");
                mLocalMapping = new LocalMapping(mImageFeaturesStore);
                mLocalMapping.execute(0);
            }
        };
    }



    ///////////////////////////////////post//////////////////////////////////////////////////////////
    public void postExternalParameters() {

        /*
        float x=mOpen.exlPara[3]*Calpha
                ,y=mOpen.exlPara[4]*Calpha
                ,z=mOpen.exlPara[5]*Calpha;

        for (int i=0; i<MAX_LIST;i++){
            x= x + listX.get(i)*Cbeta;
            y= y + listY.get(i)*Cbeta;
            z= z + listZ.get(i)*Cbeta;
        }
        exp[3] = x;
        exp[4] = y;
        exp[5] = z;
        listX.remove(0);
        listY.remove(0);
        listZ.remove(0);
        listX.add( mOpen.exlPara[3]);
        listY.add( mOpen.exlPara[4]);
        listZ.add( mOpen.exlPara[5]);

         */
    }

    public void postTexture(Bitmap bmp) {
        int textureID;
        mCreateTexture = new CreateTexture(mBitmap);
        textureID = mCreateTexture.returnTextureID();
        if (textureID != 0) System.out.println("成功");
    }

    public void updateTexture(){
        mCreateTexture.updateTexture(mBitmap);
        int textureID  = mCreateTexture.returnTextureID();
        if (textureID != 0) System.out.println("成功");
    }

    //////////////////////////////////////////////センサ
    @Override
    public void onSensorChanged(SensorEvent event) {
        switch(event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                rawacc = event.values.clone();
                lowpassFilter(grav, rawacc);
                break;

            case Sensor.TYPE_MAGNETIC_FIELD:
                lowpassFilter(mag, event.values.clone());
                break;

            default:
                return;
        }

        if (rawacc != null && mag != null) {
            float[] R  = new float[MATRIX_SIZE];
            float[] I  = new float[MATRIX_SIZE];
            float[] rR = new float[MATRIX_SIZE];
            float[] oriRad = new float[3];
            SensorManager.getRotationMatrix(R, I, grav, mag);
            SensorManager.remapCoordinateSystem(R, SensorManager.AXIS_X, SensorManager.AXIS_Z, rR);
            SensorManager.getOrientation(rR, oriRad);
            currentori = rad2deg(oriRad);

            float[] accNoGrav = new float[4];
            for (int i=0; i<3; i++) accNoGrav[i] = rawacc[i] - grav[i];

            float[] invertR = new float[16];
            Matrix.invertM(invertR, 0, R, 0);

            float[] acc4 = new float[4];
            Matrix.multiplyMV(acc4, 0, invertR, 0, accNoGrav, 0);

            for (int i=0; i<3; i++) acc[i] = acc4[i];
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {    }

    private void lowpassFilter(float[] vecPrev, float[] vecNew) {
        for (int i=0; i<vecNew.length; i++) {
            vecPrev[i] = alpha * vecPrev[i] + (1-alpha) * vecNew[i];
        }
    }

    private float[] rad2deg(float[] vec) {
        int VEC_SIZE = vec.length;
        float[] retvec = new float[VEC_SIZE];
        for (int i=0; i<VEC_SIZE; i++) {
            retvec[i] = vec[i]/(float)Math.PI*180;
        }
        return retvec;
    }
}
