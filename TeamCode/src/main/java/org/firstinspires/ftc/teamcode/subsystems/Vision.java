// ============================================================
//  Vision.java
//  Wraps a Limelight3A running the "Color/Retroreflective"
//
//  and exposes simple, ready-to-use ball-tracking methods.
//
//  All distances are in INCHES.
//
//  USAGE:
//    Vision vision = new Vision(hardwareMap, "limelight");
//    vision.init();
//    ...
//    while (opModeIsActive()) {
//        vision.update();                       // call ONCE per loop
//        Vision.Ball ball = vision.getClosestBall();
//        if (ball != null) {
//            telemetry.addData("Bearing", ball.bearingDeg);
//            telemetry.addData("Distance", ball.distanceIn);
//        }
//    }
// ============================================================

package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

@Configurable
public class Vision {

    // -------------------------------------------------------
    //  TUNING CONSTANTS — calibrate these for YOUR robot
    // -------------------------------------------------------

    // Which of the 10 pipeline slots holds your tuned Color/Retroreflective
    // pickleball pipeline (from the configuration guide).
    private static final int PIPELINE_INDEX = 0;

    // --- Camera mount geometry (for floor-distance calculation) ---
    // How high the Limelight's LENS sits above the floor, in inches.
    public static double CAMERA_MOUNT_HEIGHT_IN = 8.5; // <-- measure and replace //8.5

    // How far downward the camera is tilted from level/horizontal, in degrees.
    // Use a protractor or phone level app to get a starting value, then
    // CALIBRATE this by placing a ball at a taped, known distance and
    // adjusting this constant until getClosestBall().distanceIn matches
    // your tape measurement. Small angle errors cause large distance errors,
    // so this calibration step matters more than a precise protractor reading.
    public static double CAMERA_MOUNT_ANGLE_DEG = 9.0; // <-- calibrate

    // Height of a pickleball's CENTER off the floor, in inches
    // (roughly its own radius, since it's resting on the ground —
    // a regulation pickleball is about 2.9" in diameter).
    public static double BALL_CENTER_HEIGHT_IN = 1.45;

    // If your camera is not mounted pointing exactly straight ahead on the
    // chassis (angled to clear an arm, etc.), put that yaw offset here.
    // Positive = camera points right of the robot's true forward direction.
    public static double CAMERA_MOUNT_YAW_OFFSET_DEG = 0.0;

    // --- Single-ball vs. clump classification ---
    // Reference measurement: place ONE ball at a known distance, read its
    // area (ta) from telemetry, and record both numbers here. The class
    // uses the inverse-square law (apparent area shrinks with distance
    // squared) to predict what a single ball SHOULD look like at any other
    // distance, then compares real area against that prediction — this is
    // more accurate than a single fixed area% cutoff, since a lone ball
    // close-up and a clump far away can otherwise look similar in size.
    public static double SINGLE_BALL_REF_DISTANCE_IN = 17.0; // <-- calibrate
    public static  double SINGLE_BALL_REF_AREA_PERCENT = 0.8; // <-- calibrate

    // How much bigger than "expected single ball" something has to be
    // before it's classified as a clump rather than measurement noise.
    public static double CLUMP_AREA_RATIO_THRESHOLD = 1.6;

    // Minimum area% for a detection to be treated as a real ball at all.
    // Without this, a tiny noise speck (a stray pixel, reflection, shadow
    // edge) can still pass the Limelight's own contour filters, and — since
    // getClosestBall() ranks by computed floor distance, not size — a speck
    // sitting low in the frame can compute out to a deceptively SHORT
    // distance and incorrectly win over your real, much larger ball. This
    // threshold rejects anything too small to plausibly be a real ball
    // before it's ever added to the candidate list.
    public static double MIN_VALID_AREA_PERCENT = 0.15; // <-- tune if needed

    // -------------------------------------------------------
    //  Internal state
    // -------------------------------------------------------
    private final Limelight3A limelight;
    private List<Ball> cachedBalls = new ArrayList<>();

    public Vision(HardwareMap hardwareMap, String deviceName) {
        limelight = hardwareMap.get(Limelight3A.class, deviceName);
    }

    /** Call once, typically in your OpMode's init(). */
    public void init() {
        limelight.setPollRateHz(100);       // ask for fresh data 100x/sec
        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.start();                  // REQUIRED — results are invalid until this is called
    }

    /** Call once per loop iteration, before using any getters below. */
    public void update() {
        LLResult result = limelight.getLatestResult();
        cachedBalls = new ArrayList<>();

        if (result == null || !result.isValid()) {
            return; // cachedBalls stays empty
        }

        List<ColorResult> colorTargets = result.getColorResults();
        for (ColorResult target : colorTargets) {
            // Skip anything too small to plausibly be a real ball — this is
            // what stops noise specks from winning getClosestBall() via a
            // deceptively short trig-computed distance. See MIN_VALID_AREA_PERCENT.
            if (target.getTargetArea()*100 < MIN_VALID_AREA_PERCENT) {continue;}
            cachedBalls.add(parseBall(target));
        }
    }

    /** Stop the camera pipeline. Call when your OpMode ends, if desired. */
    public void stop() {
        limelight.stop();
    }

    // -------------------------------------------------------
    //  Public getters — all operate on data from the last update()
    // -------------------------------------------------------

    /** True if at least one ball or clump is currently visible. */
    public boolean hasTargets() {
        return !cachedBalls.isEmpty();
    }

    /** Every valid ball/clump seen in the last frame, unsorted. */
    public List<Ball> getAllBalls() {
        return cachedBalls;
    }

    /** Only targets classified as a single ball (not a clump). */
    public List<Ball> getSingleBalls() {
        List<Ball> out = new ArrayList<>();
        for (Ball b : cachedBalls) {
            if (!b.isClump) out.add(b);
        }
        return out;
    }

    /** Only targets classified as a clump of touching balls. */
    public List<Ball> getClumps() {
        List<Ball> out = new ArrayList<>();
        for (Ball b : cachedBalls) {
            if (b.isClump) out.add(b);
        }
        return out;
    }

    /** The nearest target of ANY type (ball or clump), or null if none visible. */
    public Ball getClosestBall() {
        return closestIn(cachedBalls);
    }

    /** The nearest single ball specifically, or null if none visible. */
    public Ball getClosestSingleBall() {
        return closestIn(getSingleBalls());
    }

    /** The nearest clump specifically, or null if none visible. */
    public Ball getClosestClump() {
        return closestIn(getClumps());
    }

    /** The largest target by screen area — usually the biggest/closest clump. */
    public Ball getLargestBall() {
        Ball best = null;
        for (Ball b : cachedBalls) {
            if (best == null || b.areaPercent > best.areaPercent) {
                best = b;
            }
        }
        return best;
    }

    /**
     * Convenience: robot-relative bearing in degrees to the closest target,
     * or Double.NaN if nothing is visible. Positive = target is to the right.
     */
    public double getBearingToClosest() {
        Ball b = getClosestBall();
        return (b == null) ? Double.NaN : b.bearingDeg;
    }

    // -------------------------------------------------------
    //  Internal helpers
    // -------------------------------------------------------

    private Ball closestIn(List<Ball> list) {
        Ball best = null;
        for (Ball b : list) {
            if (best == null || b.distanceIn < best.distanceIn) {
                best = b;
            }
        }
        return best;
    }

    private Ball parseBall(ColorResult target) {
        double bearingDeg = target.getTargetXDegrees() + CAMERA_MOUNT_YAW_OFFSET_DEG;
        double elevationDeg = target.getTargetYDegrees();
        double areaPercent = target.getTargetArea() * 100; //software glitch dowsnt return it as a percent, so multiplying by 100

        double distanceIn = computeFloorDistance(elevationDeg);
        double expectedSingleBallArea = expectedAreaAtDistance(distanceIn);

        boolean isClump = areaPercent > expectedSingleBallArea * CLUMP_AREA_RATIO_THRESHOLD;
        int estimatedCount = 1;
        if (isClump && expectedSingleBallArea > 0) {
            estimatedCount = Math.max(2, (int) Math.round(areaPercent / expectedSingleBallArea));
        }

        return new Ball(bearingDeg, elevationDeg, areaPercent, distanceIn, isClump, estimatedCount);
    }

    /**
     * Standard Limelight ground-target distance formula: uses the vertical
     * angle (ty) plus known camera mount height/angle to find floor distance
     * via trigonometry. Works for a target of ANY size sitting on the floor
     * — a lone ball or a clump — since it doesn't assume anything about how
     * big the target looks, only where it sits vertically in the frame.
     */
    private double computeFloorDistance(double elevationDeg) {
        double angleToTargetDeg = -CAMERA_MOUNT_ANGLE_DEG + elevationDeg;
        double angleToTargetRad = Math.toRadians(angleToTargetDeg);
        double tan = Math.tan(angleToTargetRad);

        if (Math.abs(tan) < 1e-6) {
            return Double.MAX_VALUE; // avoid divide-by-near-zero on the horizon
        }
        return (CAMERA_MOUNT_HEIGHT_IN - BALL_CENTER_HEIGHT_IN) / tan;
    }

    /**
     * Predicts what a SINGLE ball's area% should be at a given distance,
     * scaling from the calibrated reference measurement using the
     * inverse-square law (apparent size shrinks with distance squared).
     */
    private double expectedAreaAtDistance(double distanceIn) {
        if (distanceIn <= 0) return SINGLE_BALL_REF_AREA_PERCENT;
        double ratio = SINGLE_BALL_REF_DISTANCE_IN / distanceIn;
        return SINGLE_BALL_REF_AREA_PERCENT * ratio * ratio;
    }

    // -------------------------------------------------------
    //  Data class — one detected ball or clump
    // -------------------------------------------------------
    public static class Ball {
        /** Left/right angle from the robot's forward direction, in degrees. Positive = right. */
        public final double bearingDeg;
        /** Raw vertical angle from the Limelight (ty), in degrees. */
        public final double elevationDeg;
        /** Target size as a percentage of the frame (0-100). */
        public final double areaPercent;
        /** Estimated floor distance from the camera lens, in INCHES. */
        public final double distanceIn;
        /** True if this target was classified as a clump of touching balls. */
        public final boolean isClump;
        /** Rough estimated ball count if this is a clump (always 1 for a single ball). */
        public final int estimatedBallCount;

        Ball(double bearingDeg, double elevationDeg, double areaPercent,
             double distanceIn, boolean isClump, int estimatedBallCount) {
            this.bearingDeg = bearingDeg;
            this.elevationDeg = elevationDeg;
            this.areaPercent = areaPercent;
            this.distanceIn = distanceIn;
            this.isClump = isClump;
            this.estimatedBallCount = estimatedBallCount;
        }
    }
}