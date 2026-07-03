package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

@Configurable
@Autonomous(name = "FirstAuto", group = "Autos")
public class FirstAuto extends LinearOpMode {
    private Follower follower;
    private PathChain Scorepath1, Scorepath2, Scorepath3, Scorepath4, returnPath;
    public static final Pose startPose = new Pose(9, 31, 0);
    private static double goalposeX = 70.0;
    private static double goalposeY = 62.0;
    private static double ControlX = 30.0;
    private static double ControlY = 50.0;
    private static double firstT = 0.3;
    private static double firstAngle = 30.0;
    private static double secondT = 0.5;
    private static double spitT = 0.9;
    private static MotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = new MotorEx(this.hardwareMap, "IntakeMotor", Motor.GoBILDA.RPM_1150);
        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        motor.setRunMode(Motor.RunMode.RawPower);

        Scheduler.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        waitForStart();

        if (isStopRequested()) return;
        //ok

        Scorepath1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(ControlX, ControlY),
                                new Pose(goalposeX, goalposeY)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0, firstT,
                                        HeadingInterpolator.linear(startPose.getHeading(),Math.toRadians(firstAngle))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        firstT, secondT,
                                        HeadingInterpolator.constant(Math.toRadians(firstAngle))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        secondT, 0.9,
                                        HeadingInterpolator.linear(Math.toRadians(firstAngle), Math.toRadians(90))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.9, 1,
                                        HeadingInterpolator.constant(Math.toRadians(90))
        )
                        )
                )
                .addParametricCallback(spitT, this::reverse)
                .build();
        Scorepath2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(70.000, 62.000),
                                new Pose(75.000, 13.000),
                                new Pose(48.000, 13.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))

                .addPath(
                        new BezierCurve(
                                new Pose(48.000, 13.000),
                                new Pose(40.000, 30.000),
                                new Pose(70.000, 62.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addParametricCallback(0.9, this::reverse)
                .build();

        Scorepath3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(70.000, 62.000),
                                new Pose(64.000, 10.000),
                                new Pose(93.000, 13.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        new BezierCurve(
                                new Pose(93.000, 13.000),
                                new Pose(100.000, 30.000),
                                new Pose(70.000, 62.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addParametricCallback(0.9, this::reverse)
                .build();

        Scorepath4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(70.000, 62.000),
                                new Pose(105.000, 48.000)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0, 0.5,
                                        HeadingInterpolator.linear(Math.toRadians(90),Math.toRadians(-22))
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.5, 1,
                                        HeadingInterpolator.constant(Math.toRadians(-22))
                                )))
                .addPath(
                        new BezierLine(
                                new Pose(105.000, 45.000),
                                new Pose(70.000, 62.000)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0, 0.5,
                                        HeadingInterpolator.linear(Math.toRadians(-22),Math.toRadians(90))
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.5, 1,
                                        HeadingInterpolator.constant(Math.toRadians(90))
                                )))
                .addParametricCallback(0.9, this::reverse)
                .build();

        returnPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(70.000, 62.000),
                                startPose
                        )
                )
                .setLinearHeadingInterpolation(follower.getHeading(), startPose.getHeading())
                .build();

        Command auto = sequential(
                // race() finishes as soon as the FIRST command finishes.
                // follow() finishes when the path is complete, so intakeSpin
                // gets cancelled (and its setEnd stops the motor) at that
               parallel(follow(follower, Scorepath1), instant(this::intake)),
                parallel(follow(follower, Scorepath2), instant(this::intake)),
                parallel(follow(follower, Scorepath3), instant(this::intake)),
                parallel(follow(follower, Scorepath4), instant(this::intake)),
                parallel(follow(follower, returnPath),instant(this::stopI))


        );

        schedule(auto);

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            Scheduler.execute();
            telemetry.addData("Auto Running", Scheduler.isScheduled(auto));
            telemetry.addData("Pose", follower.getPose());
            telemetry.update();
        }
    }
    public void intake() {
        motor.set(1);
    }
    public void reverse() {
        motor.set(-1);
    }
    public void stopI() {
        motor.set(0);
    }
}
