package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class BlueTeamProp implements VisionProcessor {

    public int width;
    public int height;
    public Mat rgbResult;
    public Mat hsvResult;
    public Mat masked;
    public CameraCalibration calibration;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
        this.calibration = calibration;
        this.rgbResult = new Mat(width, height, CvType.CV_8UC4);
        this.hsvResult = new Mat(width, height, CvType.CV_8UC4);
        this.masked = new Mat();
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, rgbResult, Imgproc.COLOR_BGR2RGB);
        Imgproc.cvtColor(rgbResult, hsvResult, Imgproc.COLOR_RGB2HSV);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
