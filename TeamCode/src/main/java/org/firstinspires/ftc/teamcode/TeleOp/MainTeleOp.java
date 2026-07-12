package org.firstinspires.ftc.teamcode.TeleOp;
import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.ArrayList;
import java.util.List;
@Configurable
@TeleOp(name = "Main TeleOp", group = "Teleop")
public class MainTeleOp extends LinearOpMode {

    public static double Mode = 1;

    public static double start = 0.2;

    public static double end = 0.4;
    //public static double driveMult = 1;
    private Intake intake;
    private GamepadEx gp1;
    public MotorEx bl, br, fl, fr;

    public void runOpMode() throws InterruptedException{
        intake = new Intake(this);
        gp1 = new GamepadEx(gamepad1);
        waitForStart();
        fl = new MotorEx(this.hardwareMap, "LFD",Motor.GoBILDA.RPM_312);
        fr = new MotorEx(this.hardwareMap, "RFD", Motor.GoBILDA.RPM_312);
        bl = new MotorEx(this.hardwareMap, "LBD", Motor.GoBILDA.RPM_312);
        br = new MotorEx(this.hardwareMap, "RBD", Motor.GoBILDA.RPM_312);
        fr.setInverted(true);
        bl.setInverted(true);
        fl.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        fl.setRunMode(Motor.RunMode.RawPower);
        fr.setRunMode(Motor.RunMode.RawPower);
        bl.setRunMode(Motor.RunMode.RawPower);
        br.setRunMode(Motor.RunMode.RawPower);


        while (opModeIsActive()) {
            gp1.readButtons();
            if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1){
                intake.intake();
            }

            else if (gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
                intake.reverse();
            }

            else{
                intake.stop();
            }

            drive();
            telemetry.addData("LeftX:", gp1.getLeftX());
            telemetry.addData("LeftY:", gp1.getLeftY());
            telemetry.addData("RightX:", gp1.getRightX());
            telemetry.addData("Trigger", gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

            telemetry.update();
        }

    }
    public void driveRobotCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
        double[] speeds = {
                (forwardBackSpeed + strafeSpeed - turnSpeed),
                (forwardBackSpeed - strafeSpeed + turnSpeed),
                (forwardBackSpeed - strafeSpeed - turnSpeed),
                (forwardBackSpeed + strafeSpeed + turnSpeed)
        };
        double maxSpeed = 0;
        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }
        if (maxSpeed > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }
        fl.set(speeds[0]);
        fr.set(speeds[1]);
        bl.set(speeds[2]);
        br.set(speeds[3]);
    }
    public static double variableSpeedV1(double input) {
        double sign = Math.signum(input);
        double x = Math.min(abs(input), 1.0); // magnitude, clamped to [0,1]

        double y;
        if (x < 0.1) {
            y = 0.0;                          // deadzone
        } else if (x > 0.9) {
            y = 1.0;                          // saturation
        } else if (x < 0.5) {
            y = 0.2 * x + 0.1;                // linear ramp (fine control)
        } else {
            // exponential ramp — value & slope matched to the linear
            // piece at x=0.5, and lands exactly on y=1 at x=0.9
            double A = 0.00024126;
            double k = 9.0375;
            double B = 0.17787;
            y = A * Math.exp(k * x) + B;
        }

        return sign * y; // preserves direction of the input
    }
    public static double variableSpeed(double input) {
        double sign = Math.signum(input);
        double x = Math.min(abs(input), 1.0); // magnitude, clamped to [0,1]

        double y;
        if (x < 0.1) {
            y = 0.0;                          // deadzone
        } else if (x > 0.9) {
            y = 1.0;                          // saturation
        } else if (x < 0.5) {
            y = 0.4 * x + 0.2;                // linear ramp (fine control)
        } else {
            // exponential ramp — value & slope matched to the linear
            // piece at x=0.5, and lands exactly on y=1 at x=0.9
            double A = 0.004333;
            double k = 5.6035;
            double B = 0.328616;
            y = A * Math.exp(k * x) + B;
        }

        return sign * y; // preserves direction of the input
    }

    public static double LinearSpeed(double input){
        double sign = Math.signum(input);
        double x = Math.min(abs(input), 1.0);

        double y = (end-start)*x + start;
        return sign * y;
    }
    public void drive() {
        ///uncomment all lines to get the exponential scaling
        gp1.readButtons();
        //double trigger = Math.min(driveMult -gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), 1);

        Vector2d driveVector = new Vector2d(gp1.getLeftX(), gp1.getLeftY()),
                turnVector = new Vector2d(-gp1.getRightX(), 0);
        if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.1 /*&& Mode == 1*/){
            driveRobotCentric(LinearSpeed(driveVector.getX()) , LinearSpeed(driveVector.getY()), LinearSpeed(turnVector.getX()) );
        }
        else /*if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)<0.1 && Mode == 1)*/{
            driveRobotCentric(driveVector.getX() , driveVector.getY(), turnVector.getX());
        }
/*
        else{
            driveRobotCentric(variableSpeed(gp1.getLeftX())*trigger, variableSpeed(gp1.getLeftY())*trigger, variableSpeed(-gp1.getRightX())*trigger);
        }
*/
    }
}