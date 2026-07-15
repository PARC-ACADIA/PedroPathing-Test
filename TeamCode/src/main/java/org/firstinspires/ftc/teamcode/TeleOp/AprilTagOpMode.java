package org.firstinspires.ftc.teamcode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagLimelight;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
@TeleOp(name = "AprilTag Limelight", group = "Vision")
public class AprilTagOpMode extends LinearOpMode{
    public static int index = 2;
    public static double Pos = 0.08;

    @Override
    public void runOpMode() {
        AprilTagLimelight a = new AprilTagLimelight(hardwareMap, "limelight", index);
        Servo servo = hardwareMap.get(Servo.class, "servo");

        servo.setPosition(Pos);

        waitForStart();
        while (opModeIsActive()) {
            a.update();
            servo.setPosition(Pos);
                                  // call ONCE per loop
            if (a.hasTarget()) {
                telemetry.addLine("Tag:");
                telemetry.addData("Yaw", a.getAngleX(21));
                telemetry.addData("Distance", a.getDistanceFromRobot(21));
            }

            else{
                telemetry.clear();
                telemetry.addLine("No Tag Found");

            }
            telemetry.update();

        }


    }
}
