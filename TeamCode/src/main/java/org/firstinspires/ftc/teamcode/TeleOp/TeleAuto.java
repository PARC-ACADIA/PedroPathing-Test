package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.abs;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "TeleAuto", group = "TeleOp")
public class TeleAuto extends LinearOpMode {

    private Follower follower;
    public static final Pose startPose = new Pose(9, 31, 0);
    private GamepadEx gp1;
    Pose targetPose = new Pose (70, 62, Math.toRadians(90));
    Intake intake;
    private PathChain GoalPath;
    private boolean isAutomated = false; // Tracks if the robot is currently pathing

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        intake = new Intake(this);
        gp1 = new GamepadEx(gamepad1);

        // Prepares local tracking for TeleOp controls
        follower.startTeleopDrive();

        waitForStart();

        while (opModeIsActive()) {
            // CRITICAL: Must be called every single loop cycle
            follower.update();
            gp1.readButtons();

            // 1. Check for manual override or path cancellation
            if (Math.abs(gp1.getLeftX()) > 0.2 || Math.abs(gp1.getLeftY()) > 0.2) {
                if (isAutomated) {
                    isAutomated = false;
                    follower.breakFollowing();// Break out of auto path if driver moves sticks
                    follower.startTeleopDrive();
                }
            }

            // 2. Trigger the autonomous path (e.g., Driving to Score Position)
            if (gp1.wasJustPressed(GamepadKeys.Button.A) && !isAutomated) {
                isAutomated = true;
                Pose currentPose = follower.getPose();

                // Generate a path starting from wherever the robot currently is
                GoalPath = follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, targetPose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                        .build();

                // Command the follower engine to execute the path
                follower.followPath(GoalPath);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B) && !isAutomated) {
               targetPose = follower.getPose();
            }
            // 3. Drive Mode Selection Logical Split
            if (isAutomated) {
                // Check if the path completed its run
                if (!follower.isBusy()) {
                    isAutomated = false;
                    follower.startTeleopDrive(); // Hand control back to driver handles
                }
                // Do NOT call setTeleOpDrive here; let followPath do its job
            } else {

                if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1){
                    follower.setTeleOpDrive(
                            LinearSpeed(gp1.getLeftY()),  // Forward / Backward
                            LinearSpeed(-gp1.getLeftX()),  // Strafe
                            LinearSpeed(-gp1.getRightX()), // Turn
                            true              // Robot Centric (false for Field Centric)
                    );
                }
                else{
                // Regular manual driving behavior
                follower.setTeleOpDrive(
                        gp1.getLeftY(),  // Forward / Backward
                        -gp1.getLeftX(),  // Strafe
                        -gp1.getRightX(), // Turn
                        true              // Robot Centric (false for Field Centric)
                );}
            }
            if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1){
                intake.intake();
            }

            else if (gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
                intake.reverse();
            }

            else{
                intake.stop();
            }

            // Diagnostic reporting
            telemetry.addData("Mode", isAutomated ? "AUTONOMOUS MACRO" : "MANUAL DRIVE");
            telemetry.addData("X Position", follower.getPose().getX());
            telemetry.addData("Y Position", follower.getPose().getY());
            telemetry.update();
        }

    }
    public static double LinearSpeed(double input){
        double sign = Math.signum(input);
        double x = Math.min(abs(input), 1.0);

        if (x<0.05) return 0.0;//deadzone

        double y = (0.4-0.1)*x + 0.1;
        return sign * y;
    }
}



