package org.firstinspires.ftc.teamcode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Servo tester (use Panels)", group = "Tests")
public class ServoTester extends LinearOpMode {
    public static double Pos = 0.0;
    //0, is lowest,
    //0.11 is perfect 90 degrees


    private GamepadEx gp1;
    private Servo servo;
    @Override
    public void runOpMode() {
        servo  = hardwareMap.get(Servo.class, "servo");
        gp1 = new GamepadEx(gamepad1);
        waitForStart();
        while (opModeIsActive()){
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.X)){
                servo.setPosition(Pos);
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.B)){
                servo.setDirection(Servo.Direction.REVERSE);
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.A)){
                servo.setDirection(Servo.Direction.FORWARD);
            }


            telemetry.addData("Servo Position: ", servo.getPosition());
            telemetry.addData("Direction: ", servo.getDirection());
            telemetry.update();
        }


    }
}
