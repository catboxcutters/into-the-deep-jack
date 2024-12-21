package org.firstinspires.ftc.teamcode.drive.writtenCode.pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class edge_pipeline extends OpenCvPipeline {

    private volatile double orientationAngle = 0;  // Angle of the detected object
    private boolean showMaskedImage = false;      // Variable to toggle between original and masked image

    @Override
    public Mat processFrame(Mat input) {
        // Convert the image to HSV color space
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Define blue color range
        Scalar lowerBlue = new Scalar(100, 150, 50);
        Scalar upperBlue = new Scalar(140, 255, 255);

        // Mask the blue color
        Mat mask = new Mat();
        Core.inRange(hsv, lowerBlue, upperBlue, mask);

        // Perform Canny edge detection on the masked image
        Mat edges = new Mat();
        Imgproc.Canny(mask, edges, 50, 150);

        // Use Hough Line Transform to detect lines
        Mat lines = new Mat();
        Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180, 100, 50, 10);

        LineInfo longestLine = null;
        double maxLength = 0;

        if (lines.rows() > 0) {
            for (int i = 0; i < lines.rows(); i++) {
                double[] line = lines.get(i, 0);
                double x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];

                // Calculate the length of the line
                double length = Math.hypot(x2 - x1, y2 - y1);

                if (length > maxLength) {
                    maxLength = length;
                    double angle = Math.atan2(y2 - y1, x2 - x1) * 180 / Math.PI;
                    if (angle < 0) {
                        angle += 180; // Normalize angle to [0, 180]
                    }
                    longestLine = new LineInfo(new Point(x1, y1), new Point(x2, y2), angle);
                }
            }

            if (longestLine != null) {
                orientationAngle = longestLine.angle;

                // Draw the longest line on the masked image (if enabled) or input image
                Imgproc.line(showMaskedImage ? mask : input, longestLine.start, longestLine.end, new Scalar(255, 0, 0), 2); // Blue line

                if (!showMaskedImage) {
                    Imgproc.putText(input, "Angle: " + String.format("%.2f", orientationAngle) + " degrees",
                            new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);
                }
            }
        }

        // Release resources
        hsv.release();
        edges.release();
        lines.release();

        return showMaskedImage ? mask : input;  // Return the appropriate frame
    }

    public double getOrientationAngle() {
        return orientationAngle;
    }

    public void setShowMaskedImage(boolean showMaskedImage) {
        this.showMaskedImage = showMaskedImage;
    }

    // Helper class to store line information
    private static class LineInfo {
        Point start;
        Point end;
        double angle;

        LineInfo(Point start, Point end, double angle) {
            this.start = start;
            this.end = end;
            this.angle = angle;
        }
    }
}
