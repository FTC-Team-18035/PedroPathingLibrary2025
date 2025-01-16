package org.firstinspires.ftc.teamcode.opModes.working.extras;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Run Intake V4Bar Test Built In Function")
public class BuiltInFunctionRunIntake4Bar extends LinearOpMode {

    Servo IntakeV4B;


    @Override
    public void runOpMode() throws InterruptedException {

        IntakeV4B = hardwareMap.servo.get("Intake V4B");

        IntakeV4B.setPosition(.8);

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a) {
                RunV4B(.5);
            }
            else if(gamepad1.b) {
                RunV4B(.8);
            }
        }
    }

    public void RunV4B(double target) {
        if(target > 1) {
            target = 1;
        }
        else if (target < 0) {
            target = 0;
        }
        IntakeV4B.setPosition(target);
    }
}
