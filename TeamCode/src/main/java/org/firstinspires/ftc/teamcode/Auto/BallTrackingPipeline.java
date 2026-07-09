package org.firstinspires.ftc.teamcode.Auto;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.*;


public class BallTrackingPipeline extends OpenCvPipeline {
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Scalar lower = new Scalar(40, 100, 100);
        Scalar upper = new Scalar(60, 255, 255);

        public double ballX = -1, ballY = -1, ballRadius = -1;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
            Core.inRange(hsv, lower, upper, mask);

            // Optional: erode/dilate to clean up noise
            Imgproc.erode(mask, mask, new Mat(), new Point(-1,-1), 2);
            Imgproc.dilate(mask, mask, new Mat(), new Point(-1,-1), 2);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(),
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint c : contours) {
                double area = Imgproc.contourArea(c);
                if (area < 500) continue;  // reject tiny blobs

                // Check circularity: 4π·area / perimeter²
                MatOfPoint2f c2f = new MatOfPoint2f(c.toArray());
                double perim = Imgproc.arcLength(c2f, true);
                double circularity = 4 * Math.PI * area / (perim * perim);
                if (circularity < 0.7) continue;

                float[] radius = new float[1];
                Point center = new Point();
                Imgproc.minEnclosingCircle(c2f, center, radius);

                ballX = center.x;
                ballY = center.y;
                ballRadius = radius[0];

                // Draw for driver station preview
                Imgproc.circle(input, center, (int)radius[0],
                        new Scalar(0, 255, 0), 2);
            }
            return input;
        }
    }

