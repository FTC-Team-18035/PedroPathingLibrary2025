package org.firstinspires.ftc.teamcode.opModes.working.extras;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "Run Outtake V4Bar Test Built In Function")
public class BuiltInFunctionRunOuttake4Bar extends LinearOpMode {

    Servo OuttakeV4B;


    @Override
    public void runOpMode() throws InterruptedException {

        OuttakeV4B = hardwareMap.servo.get("Outtake V4B");

        OuttakeV4B.setPosition(1);

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a) {
                RunOuttakeV4B(0);
            }
            else if(gamepad1.b) {
                RunOuttakeV4B(1);
            }
        }
    }

    public void RunOuttakeV4B(double target) {
        if(target > 1) {
            target = 1;
        }
        else if (target < 0) {
            target = 0;
        }
        OuttakeV4B.setPosition(target);
    }
}
