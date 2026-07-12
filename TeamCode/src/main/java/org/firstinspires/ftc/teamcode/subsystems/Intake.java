package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
public class Intake {
    public static double intakePower = 1;
    public static double reversePower = -1;
    public static MotorEx motor;

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
