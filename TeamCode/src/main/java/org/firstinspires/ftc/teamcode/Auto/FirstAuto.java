package org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;


@Autonomous(name = "FirstAuto", group = "Autos")
public class FirstAuto extends LinearOpMode {

    private Follower follower;

    Intake intake = new Intake(this);

    public static final Pose startPose = new Pose(9, 31, 0);
    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        Scheduler.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

        }
    }
}
