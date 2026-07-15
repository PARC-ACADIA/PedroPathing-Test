package org.firstinspires.ftc.teamcode.subsystems; // adjust to match your project's package

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

/**
 * Wraps a Limelight 3A running an AprilTag ("Fiducial Markers") pipeline and
 * exposes simple getters for distance, angle, and tag ID.
 *
 * Usage:
 *   LimelightAprilTagTracker tracker = new LimelightAprilTagTracker(hardwareMap, "limelight", 0);
 *   ...
 *   tracker.update();                          // call once per loop
 *   if (tracker.hasTarget(5)) {
 *       double dist = tracker.getDistanceFromRobot(5); // inches
 *       double angle = tracker.getAngleX(5);            // degrees
 *   }
 *
 * Distances are returned in inches, angles in degrees.
 * getDistanceFromRobot() requires the camera's offset from robot-center to be
 * set in the Limelight web UI's 3D tab. "Full 3D" must also be enabled on the
 * pipeline, or all pose-based methods will return -1 / null.
 */
public class AprilTagLimelight {
    private static final double METERS_TO_INCHES = 39.3701;

    private final Limelight3A limelight;
    private LLResult latestResult;

    /**
     * @param hardwareMap   the OpMode's hardwareMap
     * @param deviceName    name given to the Limelight in your hardware configuration
     * @param pipelineIndex index of the AprilTag pipeline to run (0-9)
     */
    public AprilTagLimelight(HardwareMap hardwareMap, String deviceName, int pipelineIndex) {
        limelight = hardwareMap.get(Limelight3A.class, deviceName);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(pipelineIndex);
        limelight.start();
    }

    /** Refreshes the latest frame's data. Call this once per loop before using any getter below. */
    public void update() {
        latestResult = limelight.getLatestResult();
    }

    /** @return true if at least one AprilTag was visible in the last update(). */
    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid() && !latestResult.getFiducialResults().isEmpty();
    }

    /** @return true if the given tag ID was visible in the last update(). */
    public boolean hasTarget(int id) {
        return getFiducial(id) != null;
    }

    /** @return the IDs of every tag visible in the last update() (empty list if none). */
    public List<Integer> getVisibleTagIds() {
        List<Integer> ids = new ArrayList<>();
        if (latestResult != null && latestResult.isValid()) {
            for (LLResultTypes.FiducialResult f : latestResult.getFiducialResults()) {
                ids.add(f.getFiducialId());
            }
        }
        return ids;
    }

    /** @return the ID of the closest currently-visible tag, or -1 if none are visible. */
    public int getClosestTagId() {
        int closestId = -1;
        double closestDistance = Double.MAX_VALUE;

        if (latestResult != null && latestResult.isValid()) {
            for (LLResultTypes.FiducialResult f : latestResult.getFiducialResults()) {
                double distance = distanceInches(f.getTargetPoseCameraSpace().getPosition());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestId = f.getFiducialId();
                }
            }
        }
        return closestId;
    }

    /**
     * Straight-line distance from the camera lens to the given tag, in inches.
     * @return distance in inches, or -1 if the tag isn't currently visible.
     */
    public double getDistanceFromCamera(int id) {
        LLResultTypes.FiducialResult f = getFiducial(id);
        if (f == null) return -1;
        return distanceInches(f.getTargetPoseCameraSpace().getPosition());
    }

    /**
     * Straight-line distance from the robot's center to the given tag, in inches.
     * Requires the camera's offset from robot-center to be configured in the
     * Limelight web UI's 3D tab.
     * @return distance in inches, or -1 if the tag isn't currently visible.
     */
    public double getDistanceFromRobot(int id) {
        LLResultTypes.FiducialResult f = getFiducial(id);
        if (f == null) return -1;
        return distanceInches(f.getRobotPoseTargetSpace().getPosition());
    }

    /**
     * Horizontal angle to the given tag (tx), in degrees. Positive = tag is to the right.
     * @return angle in degrees, or 0 if the tag isn't currently visible.
     */
    public double getAngleX(int id) {
        LLResultTypes.FiducialResult f = getFiducial(id);
        return f == null ? 0 : f.getTargetXDegrees();
    }

    /**
     * Vertical angle to the given tag (ty), in degrees. Positive = tag is above center.
     * @return angle in degrees, or 0 if the tag isn't currently visible.
     */
    public double getAngleY(int id) {
        LLResultTypes.FiducialResult f = getFiducial(id);
        return f == null ? 0 : f.getTargetYDegrees();
    }

    /**
     * The robot's full field-space pose, computed from all currently-visible tags at once.
     * Requires a field map to be uploaded to the Limelight (see the AprilTag pipeline's
     * map upload section in the web UI) and "Full 3D" to be enabled.
     * @return the robot's pose in field space, or null if no valid pose is available this frame.
     */
    public Pose3D getRobotFieldPose() {
        if (latestResult == null || !latestResult.isValid()) return null;
        return latestResult.getBotpose();
    }

    /** Switches to a different pipeline index (e.g. to swap between AprilTag and another pipeline). */
    public void switchPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    /** Stops the Limelight. Call from your OpMode's stop() if you want to free it explicitly. */
    public void stop() {
        limelight.stop();
    }

    // ---- internal helpers ----

    private LLResultTypes.FiducialResult getFiducial(int id) {
        if (latestResult == null || !latestResult.isValid()) return null;
        for (LLResultTypes.FiducialResult f : latestResult.getFiducialResults()) {
            if (f.getFiducialId() == id) return f;
        }
        return null;
    }

    private double distanceInches(Position pos) {
        double meters = Math.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
        return meters * METERS_TO_INCHES;
    }
}
