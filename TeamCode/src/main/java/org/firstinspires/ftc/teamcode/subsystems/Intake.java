package org.firstinspires.ftc.teamcode.subsystems;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierLine;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
public class Intake {
    public static double intakePower = 1;
    public static double reversePower = 1;
    private static MotorEx motor;

    public Intake(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "IntakeMotor", Motor.GoBILDA.RPM_1150);
        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        motor.setRunMode(Motor.RunMode.RawPower);
    }

    public void intake() {
         motor.set(intakePower);
    }
    public void reverse() {
        motor.set(reversePower);
    }

    public static void stop() {
        motor.set(0);
    }
}
