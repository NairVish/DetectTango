package org.tensorflow.demo;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.graphics.Matrix;
import android.graphics.Point;
import android.graphics.PointF;
import android.opengl.GLSurfaceView;
import android.util.Log;
import android.view.Display;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.experimental.TangoImageBuffer;
import com.google.tango.depthinterpolation.TangoDepthInterpolation;
import com.google.tango.support.TangoPointCloudManager;
import com.google.tango.support.TangoSupport;

import org.tensorflow.demo.env.ImageUtils;

import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

/**
 * Created by manjekarbudhai on 7/27/17.
 */

public class DetectionEngine {

    private static final String TAG = "DetectionEngine";

    private Context mContext;
    private Activity mActivity;

    private Tango tango_;
//    private TangoConfig tangoConfig_;
    private volatile boolean tangoConnected_ = false;
    private TangoPointCloudManager mPointCloudManager;
    HashMap<Integer, Integer> cameraTextures_ = null;
    private GLSurfaceView view_;
    private Renderer renderer_;
    private volatile TangoImageBuffer mCurrentImageBuffer;
    private int mDisplayRotation = 0;
    private Matrix rgbImageToDepthImage;
    protected SubDetectorEngine detect;

    public static final String DETECTION_SPEAK_BROADCAST_ACTION = "org.tensorflow.demo.SPEAK_DETECTION";
    public static final String KEY_DEPTH = "org.tensorflow.demo.SPEAK_DETECTION.CLOSEST_DEPTH";
    public static final String KEY_DIRECTION = "org.tensorflow.demo.SPEAK_DETECTION.KEY_DIRECTION";
    public static final String KEY_HEAD_COUNT = "org.tensorflow.demo.SPEAK_DETECTION.HEAD_COUNT";

    private class MeasuredPoint {
        public double mTimestamp;
        public float[] mDepthTPoint;

        public MeasuredPoint(double timestamp, float[] depthTPoint) {
            mTimestamp = timestamp;
            mDepthTPoint = depthTPoint;
        }
    }

    @SuppressLint("WrongConstant")
    DetectionEngine(Context context, Activity activity) {
        mContext = context;
        mActivity = activity;

        // GLSurfaceView for RGB color camera

        detect = new SubDetectorEngine(mContext);

        Display display = mActivity.getWindowManager().getDefaultDisplay();
        mDisplayRotation = display.getRotation();
        // detect.sensorOrientation = mDisplayRotation;

        rgbImageToDepthImage = ImageUtils.getTransformationMatrix(
                640, 480,
                1920, 1080,
                0, true);

        cameraTextures_ = new HashMap<>();
        mPointCloudManager = new TangoPointCloudManager();

        try {
            TimeUnit.SECONDS.sleep(1);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        cameraTextures_ = new HashMap<>();
        view_ = (GLSurfaceView) mActivity.findViewById(R.id.surfaceviewclass);
        view_.setEGLContextClientVersion(2);
        view_.setDebugFlags(GLSurfaceView.DEBUG_CHECK_GL_ERROR);
        view_.setRenderer(renderer_ = new Renderer(this));
        view_.setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);

        detect.setup();
        Log.i("onCreate", "detection setup completed");

    }

    public void resumeEngine() {
        Log.i(TAG, "resumeEngine called");
    }

    public void instantiateDetectorTangoConfig(TangoConfig mConfig) {
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_COLORCAMERA, true);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_AUTORECOVERY, true);
    }

    public void startDetectorTango(TangoConfig mConfig, Tango mTango) {
        tango_ = mTango;
        startTango();
    }

    public void startDetection() {
        new Thread(new RunDetection()).start();
    }


    public void pauseEngine() {
        Log.i(TAG, "pauseEngine called");
        tangoConnected_ = false;
    }

    public void destroyEngine() {
        Log.i(TAG, "destroyEngine called");
    }

    public synchronized void attachTexture(final int cameraId, final int textureName) {
        if (textureName > 0) {
            // Link the texture with Tango if the texture changes after
            // Tango is connected. This generally doesn't happen but
            // technically could because they happen in separate
            // threads. Otherwise the link will be made in startTango().
            if(cameraTextures_ != null && tango_ != null) {
                if (tangoConnected_ && cameraTextures_.get(cameraId) != textureName)
                    tango_.connectTextureId(cameraId, textureName);
                cameraTextures_.put(cameraId, textureName);
            }
        }
        else
            cameraTextures_.remove(cameraId);
    }

    public synchronized void updateTexture(int cameraId) {
        if (tangoConnected_) {
            try {
                tango_.updateTexture(cameraId);
            }
            catch (TangoInvalidException e) {
                e.printStackTrace();
            }
        }
    }

    public Point getCameraFrameSize(int cameraId) {
        // TangoCameraIntrinsics intrinsics = mTango.getCameraIntrinsics(cameraId);
        // return new Point(intrinsics.width, intrinsics.height);
        return new Point(640, 480);
        //   return new Point(1280, 720);
    }

    public void pointCloudUpdate(TangoPointCloudData pointCloud) {
        mPointCloudManager.updatePointCloud(pointCloud);
    }

    public void onTangoFrameAvailable(int i) {
        //Log.i("onFrameAvailabe", "Main onFrameAvailabe called");
        if (i == TangoCameraIntrinsics.TANGO_CAMERA_COLOR) {
            // mColorCameraPreview.onFrameAvailable();
            view_.requestRender();
            if(renderer_.argbInt != null){
                detect.argbInt = renderer_.argbInt;
                detect.processPerFrame();
            }
        }
    }


    private void startTango() {
        try {

            tangoConnected_ = true;
            Log.i("startTango", "Tango Connected");

            // Attach cameras to textures.
            synchronized(this) {
                for (Map.Entry<Integer, Integer> entry : cameraTextures_.entrySet())
                    tango_.connectTextureId(entry.getKey(), entry.getValue());
            }

            // Attach Tango experimental listener.
            tango_.experimentalConnectOnFrameListener(TangoCameraIntrinsics.TANGO_CAMERA_COLOR,
                    new Tango.OnFrameAvailableListener() {
                        @Override
                        public void onFrameAvailable(TangoImageBuffer tangoImageBuffer, int i) {
                            mCurrentImageBuffer = copyImageBuffer(tangoImageBuffer);
                           // Log.i("onFrame",String.format("Tango Image Size: %dx%d",
                               //     mCurrentImageBuffer.width,mCurrentImageBuffer.height));
                        }

                        TangoImageBuffer copyImageBuffer(TangoImageBuffer imageBuffer) {
                            ByteBuffer clone = ByteBuffer.allocateDirect(imageBuffer.data.capacity());
                            imageBuffer.data.rewind();
                            clone.put(imageBuffer.data);
                            imageBuffer.data.rewind();
                            clone.flip();
                            return new TangoImageBuffer(imageBuffer.width, imageBuffer.height,
                                    imageBuffer.stride, imageBuffer.frameNumber,
                                    imageBuffer.timestamp, imageBuffer.format, clone,
                                    imageBuffer.exposureDurationNs);
                        }
                    });
        }
        catch (TangoOutOfDateException e) {
            Log.e(TAG, "TangoCore update required");
        }
        catch (TangoErrorException e) {
            Log.e(TAG, "Tango error: " + e.getMessage());
        }
    }

    public class RunDetection implements Runnable{
        @Override
        public void run(){
            final int  sleepShort = 5;
            int count = 0;
            int head_count = 0;
            float closest_depth;
            PointF closest_obstacle = new PointF(0.0f,0.0f);
            while(true) {
                try {
                    if(tangoConnected_ == false){
                        Thread.sleep(sleepShort);
                        continue;
                    }
                    if (null != renderer_.argbInt) {
                        ++count;
                        detect.argbInt = renderer_.argbInt;
                        detect.process();
                            if(count == 5) {
                                count = 0;
                                detect.processRects();
                                head_count = 0;
                                closest_depth = 10.0f;
                                closest_obstacle.set(0.0f,0.0f);
                                for(PointF rect: detect.rectDepthxy) {
                                    ++head_count;
                                    MeasuredPoint m = getBboxDepth(rect.x,rect.y);
                                    if (m.mDepthTPoint.length == 3) {
                                        if(m.mDepthTPoint[2] < closest_depth) {
                                            closest_depth = m.mDepthTPoint[2];
                                            closest_obstacle = new PointF(rect.x,rect.y);

                                        }
                                    }
                                }
                                if(closest_obstacle.x != 0.0f) {
                                    boolean orientation = getOrientationDir(closest_obstacle);
                                    double orientationval = getOrientationVal(closest_obstacle);
                                    DetectionDirection curr;
                                    if (orientationval > 30.0f && orientation) {
                                        // tts1.speak(String.format("There are %d people. The closest is about %d meters away, to the right.", head_count, Math.round(closest_depth)), TextToSpeech.QUEUE_FLUSH, null, "Detected");
                                        curr = DetectionDirection.RIGHT;
                                    }
                                    else if (orientationval > 30.0f && !orientation){
                                        // tts1.speak(String.format("There are %d people. The closest is about %d meters away, to the left.", head_count, Math.round(closest_depth)), TextToSpeech.QUEUE_FLUSH, null, "Detected");
                                        curr = DetectionDirection.LEFT;
                                    }
                                    else {
                                        // tts1.speak(String.format("There are %d people. The closest is about %d meters away, ahead of you.", head_count, Math.round(closest_depth)), TextToSpeech.QUEUE_FLUSH, null, "Detected");
                                        curr = DetectionDirection.AHEAD;
                                    }

                                    Intent in = new Intent(DETECTION_SPEAK_BROADCAST_ACTION);
                                    in.putExtra(KEY_HEAD_COUNT, head_count);
                                    in.putExtra(KEY_DEPTH, Math.round(closest_depth));
                                    in.putExtra(KEY_DIRECTION, curr);
                                    mContext.sendBroadcast(in);
                                }
                            }
                        //detect.processRects();
                    } else {
                        Thread.sleep(sleepShort);
                    }
                }catch(Exception e){
                    System.out.println(e);
                }
            }
        }
    }

    enum DetectionDirection {
        LEFT,
        RIGHT,
        AHEAD
    }


    public MeasuredPoint getBboxDepth(float u, float v) {
        TangoPointCloudData pointCloud = mPointCloudManager.getLatestPointCloud();
        if (pointCloud == null) {
            return null;
        }

        double rgbTimestamp;
        TangoImageBuffer imageBuffer = mCurrentImageBuffer;
        rgbTimestamp = imageBuffer.timestamp;

        TangoPoseData depthlTcolorPose = TangoSupport.getPoseAtTime(
                rgbTimestamp,
                TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                TangoPoseData.COORDINATE_FRAME_CAMERA_COLOR,
                TangoSupport.ENGINE_TANGO,
                TangoSupport.ENGINE_TANGO,
                TangoSupport.ROTATION_IGNORED);
        if (depthlTcolorPose.statusCode != TangoPoseData.POSE_VALID) {
            Log.w("getdepthBbox", "Could not get color camera transform at time "
                    + rgbTimestamp);
            return null;
        }

        float[] depthPoint;


        depthPoint = TangoDepthInterpolation.getDepthAtPointBilateral(
                pointCloud,
                new double[] {0.0, 0.0, 0.0},
                new double[] {0.0, 0.0, 0.0, 1.0},
                imageBuffer,
                u, v,
                mDisplayRotation,
                depthlTcolorPose.translation,
                depthlTcolorPose.rotation);

        if (depthPoint == null) {
            Log.i("getBboxDepth()", "depth is null");
            return null;
        }
        //Log.i("getBboxDepth()", String.format("x:%f, y:%f, z:%f",depthPoint[0],depthPoint[1],depthPoint[2]));
        //tts1.speak("Depth detected",TextToSpeech.QUEUE_ADD,null,"Detected");
        return new MeasuredPoint(rgbTimestamp, depthPoint);
    }

    public boolean getOrientationDir(PointF bbox_in){
        boolean isClockwise = false;
        float adjacent = 320.0f - 640.0f*bbox_in.x;
        if(adjacent < 0.f){
            isClockwise = true;
        }
        else{
            isClockwise = false;
        }
        return isClockwise;
    }

    public double getOrientationVal(PointF bbox_in){
       double orientation = 0;
        double adjacent = (double)(480.0f - 480.0f*bbox_in.y);
        double opposite = (double)(Math.abs((320.0f - 640.0f*bbox_in.x)));
            orientation = Math.toDegrees(Math.atan(opposite/adjacent));
        return orientation;
    }
}



