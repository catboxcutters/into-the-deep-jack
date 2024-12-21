package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;
import org.firstinspires.ftc.teamcode.drive.writtenCode.pipelines.edge_pipeline;

public class CameraController {

    private OpenCvWebcam webcam;
    private edge_pipeline pipeline;

    public CameraController(HardwareMap hardwareMap, String webcamName) {
        // Initialize the webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        // Set the pipeline
        pipeline = new edge_pipeline();
        webcam.setPipeline(pipeline);
    }

    public void startCamera() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                System.err.println("Error Opening Camera: Error code " + errorCode);
            }
        });
    }

    public void stopCamera() {
        if (webcam != null) {
            webcam.stopStreaming();
        }
    }

    public double getOrientationAngle() {
        return pipeline.getOrientationAngle();
    }

    public void setShowMaskedImage(boolean showMaskedImage) {
        pipeline.setShowMaskedImage(showMaskedImage);
    }
}
