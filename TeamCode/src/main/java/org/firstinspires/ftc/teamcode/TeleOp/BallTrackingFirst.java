package org.firstinspires.ftc.teamcode.TeleOp;


    // ============================================================
//  PickleballDetectionOpMode.java
//  Detects yellow pickleballs with a Logitech USB webcam
//  using EasyOpenCV (bundled in FTC SDK v8.2+).
//
//  HOW TO USE:
//  1. Copy this file into your TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ folder.
//  2. In the Driver Station app, add your webcam to the hardware config and name it "Webcam 1".
//  3. Run the OpMode in INIT-only mode first (don't press Start yet).
//     Open the Driver Station overflow menu → "Camera Stream" to see the live feed
//     and tune your HSV values if needed.
//  4. Adjust HSV_* constants below until only the yellow pickleball lights up white
//     in the mask preview.
// ============================================================
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

    @TeleOp(name = "Pickleball Detector", group = "Vision")
    public class BallTrackingFirst extends LinearOpMode {

        // -------------------------------------------------------
        //  TUNING CONSTANTS — adjust these for your lighting!
        //  HSV range for yellow pickleballs.
        //  Hue:        0–179 in OpenCV  (yellow ≈ 20–35)
        //  Saturation: 0–255            (100+ keeps vivid yellows)
        //  Value:      0–255            (100+ removes dark shadows)
        // -------------------------------------------------------
        static final double HSV_HUE_MIN   = 20;
        static final double HSV_HUE_MAX   = 45;
        static final double HSV_SAT_MIN   = 100;
        static final double HSV_SAT_MAX   = 255;
        static final double HSV_VAL_MIN   = 100;
        static final double HSV_VAL_MAX   = 255;

        static final OpenCvWebcam.StreamFormat STREAM_FORMAT =
                OpenCvWebcam.StreamFormat.MJPEG;

        // Minimum contour area in pixels — rejects tiny noise blobs.
        // Increase this if you're getting false positives from small reflections.
        static final double MIN_CONTOUR_AREA = 650;

        // Minimum circularity score (0–1).
        // A perfect circle = 1.0. A pickleball should be > 0.7.
        // Lower this if the ball is partially occluded or at an angle.
        static final double MIN_CIRCULARITY = 0.50;

        // Camera resolution — must match a resolution your webcam supports.
        // Common Logitech resolutions: 320x240, 640x480, 1280x720
        static final int CAMERA_WIDTH  = 1280;
        static final int CAMERA_HEIGHT = 720;

        static final int Camera_x = 7;
        static final int Camera_y = 6;
        static final int Camera_z = 8;

        // Drive motor names — match your hardware config exactly.
        // Remove or change these if your robot has a different drive setup.
        //static final String LEFT_MOTOR_NAME  = "leftDrive";
        //static final String RIGHT_MOTOR_NAME = "rightDrive";

        // How fast the robot rotates to center on the ball (0.0–1.0).
        //static final double TURN_GAIN  = 0.012;

        // How fast the robot drives toward the ball (0.0–1.0).
        //static final double DRIVE_GAIN = 0.008;

        // The robot will try to drive toward the ball until its
        // area (% of screen) reaches this size — roughly "close enough".
        static final double TARGET_AREA_PERCENT = 8.0;
        static final double PICKLEBALL_DIAMETER_in = 2.75;
        static final double FOCAL_LENGTH_PX = 1040;

        // -------------------------------------------------------
        //  Member variables
        // -------------------------------------------------------
        OpenCvWebcam        webcam;
        PickleballPipeline  pipeline;


        @Override
        public void runOpMode() {

            // ---- Hardware map ----
            // Comment out these lines if you don't want drive control yet
            // and just want to test the camera detection.
            /*leftDrive  = hardwareMap.get(DcMotor.class, LEFT_MOTOR_NAME);
            rightDrive = hardwareMap.get(DcMotor.class, RIGHT_MOTOR_NAME);
            leftDrive.setDirection(DcMotor.Direction.REVERSE); // flip if needed
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

            // ---- Camera setup ----
            int cameraMonitorViewId = hardwareMap.appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id",
                            hardwareMap.appContext.getPackageName());

            webcam = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, "Webcam 1"),
                    cameraMonitorViewId);

            pipeline = new PickleballPipeline();
            webcam.setPipeline(pipeline);

            // Give the USB driver 2.5 seconds to connect before timing out.
            webcam.setMillisecondsPermissionTimeout(2500);

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    // Start streaming once the camera opens.
                    webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT,
                            OpenCvCameraRotation.UPRIGHT, STREAM_FORMAT);
                }

                @Override
                public void onError(int errorCode) {
                    // Camera failed to open — show in telemetry and keep going.
                    telemetry.addData("Camera Error", errorCode);
                    telemetry.update();
                }
            });

            telemetry.addLine("Waiting for start — aim camera at a yellow pickleball.");
            telemetry.addLine("Open Driver Station overflow → 'Camera Stream' to preview.");
            telemetry.update();

            waitForStart();

            // ---- Main loop ----
            while (opModeIsActive()) {

                // Read latest detection results from the pipeline.
                // These are set by processFrame() on the camera thread.
                boolean ballFound  = pipeline.ballFound;
                double  ballX      = pipeline.ballCenterX;   // pixels from left edge
                double  ballY      = pipeline.ballCenterY;   // pixels from top edge
                double  ballRadius = pipeline.ballRadius;     // pixels
                double  ballArea   = pipeline.ballAreaPct;    // 0–100 % of frame

                double ballX_in = pipeline.ballX_in + Camera_x; // + = right of center
                double ballY_in = pipeline.ballY_in - Camera_y; // + = below center
                double ballZ_in = pipeline.ballZ_in - Camera_z;


                // ---- Telemetry ----
                telemetry.addLine("=== Pickleball Detector ===");
                if (ballFound) {
                    // How far left/right the ball is from the center of the frame.
                    double xError = ballX - (CAMERA_WIDTH  / 2.0); // positive = right
                    double yError = ballY - (CAMERA_HEIGHT / 2.0); // positive = down


                    telemetry.addData("Ball found",    "YES");
                    telemetry.addData("Center (px)",   "x=%.0f  y=%.0f", ballX, ballY);
                    telemetry.addData("Radius (px)",   "%.1f", ballRadius);
                    telemetry.addData("Area (%%)",     "%.2f", ballArea);
                    telemetry.addData("X error",       "%.1f px (+ = right)", xError);
                    telemetry.addData("Position (in)", "X=%.0f  Y=%.0f  Z=%.0f",
                            ballX_in, ballY_in, ballZ_in);

                    // ---- Drive logic ----
                    // If the ball area is already big enough we are close — stop.
                    if (ballArea >= TARGET_AREA_PERCENT) {
                        telemetry.addLine("Status: AT TARGET — stopping");
                    } else {
                        // Turn to center the ball horizontally, and drive forward.
                        //double turn  = xError * TURN_GAIN;   // + turns right, - turns left
                        //double drive = (TARGET_AREA_PERCENT - ballArea) * DRIVE_GAIN;

                        // Clamp drive and turn to safe power levels.
                       // drive = Math.min(drive, 0.5);
                        //turn  = Math.max(-0.4, Math.min(0.4, turn));

                        //leftDrive.setPower(drive + turn);
                        //rightDrive.setPower(drive - turn);
                        //telemetry.addData("Drive power", "L=%.2f  R=%.2f",
                        //        drive + turn, drive - turn);
                        telemetry.addLine("Status: TRACKING");
                    }

                } else {
                    // No ball visible — stop the robot.
                    telemetry.addData("Ball found", "NO");
                    telemetry.addLine("Status: SEARCHING — rotate to find ball");
                }

                telemetry.addLine();
                telemetry.addData("Frame count", webcam.getFrameCount());
                telemetry.addData("FPS",         "%.1f", webcam.getFps());
                telemetry.update();

                // A small sleep so we don't hammer telemetry unnecessarily.
                sleep(20);
            }

            // Stop streaming when the OpMode ends.
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }


        // ===========================================================
        //  PIPELINE CLASS
        //  processFrame() is called automatically by EasyOpenCV on a
        //  separate thread each time a new camera frame arrives.
        //  Read the volatile result fields from your OpMode loop above.
        // ===========================================================
        static class PickleballPipeline extends OpenCvPipeline {


            // Results — written by the camera thread, read by the OpMode thread.
            // 'volatile' ensures the OpMode thread always sees the latest value.
            volatile boolean ballFound   = false;
            volatile double  ballCenterX = 0;
            volatile double  ballCenterY = 0;
            volatile double  ballRadius  = 0;
            volatile double  ballAreaPct = 0;  // ball area as % of full frame

            // Pre-allocated Mats — reusing them avoids garbage-collection hitches.

            volatile double ballX_in = 0;
            volatile double ballY_in = 0;
            volatile double ballZ_in = 0;

            final Mat hsv     = new Mat();
            final Mat mask    = new Mat();
            final Mat eroded  = new Mat();
            final Mat dilated = new Mat();

            // HSV threshold bounds — read from the outer class constants.
            final Scalar lowerYellow = new Scalar(HSV_HUE_MIN, HSV_SAT_MIN, HSV_VAL_MIN);
            final Scalar upperYellow = new Scalar(HSV_HUE_MAX, HSV_SAT_MAX, HSV_VAL_MAX);

            // Drawing colors
            final Scalar GREEN  = new Scalar(0, 255, 0);
            final Scalar RED    = new Scalar(255, 0, 0);
            final Scalar WHITE  = new Scalar(255, 255, 255);

            // Which view to show in the Driver Station stream.
            // Tap the viewport on the Robot Controller phone to cycle between views.
            enum Stage { FINAL, MASK }
            volatile Stage activeStage = Stage.FINAL;

            @Override
            public void onViewportTapped() {
                // Cycle through pipeline stages when the user taps the RC screen.
                if (activeStage == Stage.FINAL) {
                    activeStage = Stage.MASK;
                } else {
                    activeStage = Stage.FINAL;
                }
            }

            @Override
            public Mat processFrame(Mat input) {
                // ---- Step 1: Convert from RGBA (EasyOpenCV) to HSV ----
                // EasyOpenCV gives RGBA frames (not BGR like desktop OpenCV!).
                Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

                // ---- Step 2: Threshold — keep only pixels in the yellow HSV range ----
                Core.inRange(hsv, lowerYellow, upperYellow, mask);

                // ---- Step 3: Morphological opening — removes small noise dots ----
                // Erode shrinks blobs (kills tiny noise), dilate grows them back.
                Imgproc.erode(mask, eroded, new Mat(), new Point(-1, -1), 2);
                Imgproc.dilate(eroded, dilated, new Mat(), new Point(-1, -1), 2);

                // ---- Step 4: Find contours in the cleaned mask ----
                List<MatOfPoint> contours  = new ArrayList<>();
                Imgproc.findContours(
                        dilated,
                        contours,
                        new Mat(),                        // hierarchy (not needed here)
                        Imgproc.RETR_EXTERNAL,            // only outer contours
                        Imgproc.CHAIN_APPROX_SIMPLE);     // compress redundant points

                // ---- Step 5: Pick the best contour (most circular + biggest) ----
                double  bestScore  = -1;
                Point   bestCenter = null;
                float[] bestRadius = {0};

                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);

                    // Skip blobs that are too small — they're likely noise.
                    if (area < MIN_CONTOUR_AREA) continue;

                    // Calculate circularity: 4π·area / perimeter²
                    // Perfect circle = 1.0, jagged shape → closer to 0.
                    MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                    double perimeter  = Imgproc.arcLength(contour2f, true);
                    double circularity = (4 * Math.PI * area) / (perimeter * perimeter);

                    if (circularity < MIN_CIRCULARITY) continue;

                    // Score = area × circularity² — rewards large AND round blobs.
                    double score = area * (circularity * circularity);
                    if (score > bestScore) {
                        bestScore = score;

                        float[] radius = new float[1];
                        Point   center = new Point();
                        Imgproc.minEnclosingCircle(contour2f, center, radius);

                        bestCenter = center;
                        bestRadius = radius;
                    }
                }

                // ---- Step 6: Update result fields for the OpMode thread ----
                double frameArea = input.rows() * input.cols();

                if (bestCenter != null) {
                    ballFound   = true;
                    ballCenterX = bestCenter.x;
                    ballCenterY = bestCenter.y;
                    ballRadius  = bestRadius[0];
                    // Area of circle as % of total frame area
                    ballAreaPct = (Math.PI * bestRadius[0] * bestRadius[0] / frameArea) * 100.0;

                    double apparentDiameterPx = bestRadius[0] * 2.0;
                    double imageCenterX = input.cols() / 2.0;
                    double imageCenterY = input.rows() / 2.0;

                    ballZ_in = (PICKLEBALL_DIAMETER_in * FOCAL_LENGTH_PX) / apparentDiameterPx;

                    // X/Y: once Z is known, back out how far left/right and up/down
                    // the ball is in real units, using how far its pixel position
                    // is from the image center.
                    ballX_in = (bestCenter.x - imageCenterX) * ballZ_in / FOCAL_LENGTH_PX;
                    ballY_in = (bestCenter.y - imageCenterY) * ballZ_in/ FOCAL_LENGTH_PX;
                    // Draw a green circle around the detected ball.
                    Imgproc.circle(input, bestCenter, (int) bestRadius[0], GREEN, 3);
                    // Draw crosshair dot at the center.
                    Imgproc.circle(input, bestCenter, 5, GREEN, -1);
                    // Label the ball with its area %.
                    Imgproc.putText(input,
                            String.format("%.1f%%", ballAreaPct),
                            new Point(bestCenter.x + bestRadius[0] + 5, bestCenter.y),
                            Imgproc.FONT_HERSHEY_SIMPLEX,
                            0.6, WHITE, 2);
                } else {
                    ballFound   = false;
                    ballAreaPct = 0;

                    // Draw a red "no ball" indicator in the top-left corner.
                    Imgproc.putText(input, "NO BALL",
                            new Point(10, 30),
                            Imgproc.FONT_HERSHEY_SIMPLEX,
                            0.8, RED, 2);
                }

                // ---- Step 7: Draw the center crosshair of the frame ----
                int cx = input.cols() / 2;
                int cy = input.rows() / 2;
                Imgproc.line(input, new Point(cx - 20, cy), new Point(cx + 20, cy), WHITE, 1);
                Imgproc.line(input, new Point(cx, cy - 20), new Point(cx, cy + 20), WHITE, 1);

                // ---- Step 8: Return the correct view for the DS stream ----
                // Tap the RC phone screen to switch between the annotated feed
                // and the raw binary mask (useful for tuning HSV values).
                if (activeStage == Stage.MASK) {
                    // Convert grayscale mask back to RGBA so it can be displayed.
                    Mat maskRgba = new Mat();
                    Imgproc.cvtColor(dilated, maskRgba, Imgproc.COLOR_GRAY2RGBA);
                    return maskRgba;
                }

                return input;  // return the annotated color feed
            }
        }
    }

