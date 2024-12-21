package org.firstinspires.ftc.teamcode.drive.writtenCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class PipelineTEST extends LinearOpMode {

    OpenCvWebcam webcam;
    SamplePipeline pipeline;

    @Override
    public void runOpMode() {
        // Initialize the webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set the pipeline
        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);

        // Start streaming using an anonymous inner class
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming when the camera is opened
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle the error
                telemetry.addData("Error Opening Camera", "Error code: " + errorCode);
                telemetry.update();
            }
        });

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            // Print rectangle angle
            telemetry.addData("Rectangle Angle", pipeline.getRectangleAngle());
            telemetry.update();

            sleep(50);
        }

        // Stop the camera when done
        webcam.stopStreaming();
    }

    static class SamplePipeline extends OpenCvPipeline {
        private volatile double rectangleAngle = 0;  // Make it volatile for thread safety

        @Override
        public Mat processFrame(Mat input) {
            // Convert to HSV
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define blue color range
            Scalar lowerBlue = new Scalar(100, 150, 10);
            Scalar upperBlue = new Scalar(140, 255, 255);

            // Mask the blue color
            Mat mask = new Mat();
            Core.inRange(hsv, lowerBlue, upperBlue, mask);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Define a minimum area threshold
            double minArea = 500;

            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) < minArea) {
                    continue;
                }

                // Approximate the contour to a polygon
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

                // Check if the approximated polygon has 4 vertices
                if (approxCurve.total() == 4) {
                    // Get the minimum area rectangle
                    RotatedRect rect = Imgproc.minAreaRect(contour2f);

                    // Store the angle of the rectangle
                    rectangleAngle = rect.angle;
                    if (rectangleAngle < -45) {
                        rectangleAngle += 90;
                    }

                    // Draw the rectangle
                    Point[] boxPoints = new Point[4];
                    rect.points(boxPoints);
                    MatOfPoint box = new MatOfPoint(boxPoints);
                    List<MatOfPoint> boxList = new ArrayList<>();
                    boxList.add(box);
                    Imgproc.drawContours(input, boxList, -1, new Scalar(0, 255, 0), 2);

                    // Draw the center point
                    Imgproc.circle(input, rect.center, 5, new Scalar(255, 0, 0), -1);
                }
            }

            // Release Mats to free memory
            hsv.release();
            mask.release();
            hierarchy.release();

            return input;  // Return the processed frame
        }

        public double getRectangleAngle() {
            return rectangleAngle;
        }
    }
}
