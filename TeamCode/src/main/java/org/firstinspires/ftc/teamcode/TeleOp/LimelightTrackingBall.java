package org.firstinspires.ftc.teamcode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Configurable
@TeleOp(name = "Limelight Tracking", group = "Vision")
public class LimelightTrackingBall extends LinearOpMode {

    public static double Pos = 0.08;
    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap, "limelight");
        vision.init();

        Servo servo = hardwareMap.get(Servo.class, "servo");

        servo.setPosition(Pos);

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(Pos);
            vision.update();                       // call ONCE per loop
            Vision.Ball ball = vision.getLargestBall();
            if (ball != null) {
                telemetry.addLine("Ball:");
                telemetry.addData("Bearing", ball.bearingDeg);
                telemetry.addData("Distance", ball.distanceIn);
                telemetry.addData("Area", ball.areaPercent);
                telemetry.update();
            }
            if(isStopRequested()){
                vision.stop();
                stop();
            }

        }
    }
}
