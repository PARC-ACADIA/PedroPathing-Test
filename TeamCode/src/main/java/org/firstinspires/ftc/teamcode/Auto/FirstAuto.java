package org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
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
    private PathChain Scorepath1;

    Intake intake = new Intake(this);

    public static final Pose startPose = new Pose(9, 31, 0);
    public static double goalposeX = 71.0;
    public static double goalposeY = 62.0;
    public static double ControlX = 30.0;
    public static double ControlY = 50.0;
    public static double firstT = 0.3;
    public static double firstAngle = 30.0;
    public static double secondT = 0.7;
    public static double spitT = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        Scheduler.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        waitForStart();

        if (isStopRequested()) return;

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
                                        secondT, 1,
                                        HeadingInterpolator.linear(Math.toRadians(firstAngle), Math.toRadians(90))
                                )
                        )
                )
                .addParametricCallback(spitT, () -> intake.reverse())
                .build();

        Command auto = sequential(
                // race() finishes as soon as the FIRST command finishes.
                // follow() finishes when the path is complete, so intakeSpin
                // gets cancelled (and its setEnd stops the motor) at that
               parallel( // exact moment automatically.
                follow(follower, Scorepath1),
                instant(() -> intake.intake())) 
                
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
}
