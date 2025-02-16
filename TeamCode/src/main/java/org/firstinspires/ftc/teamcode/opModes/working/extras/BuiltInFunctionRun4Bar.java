package org.firstinspires.ftc.teamcode.opModes.working.extras;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "Run V4Bars Test Built In Function || WORKING")
public class BuiltInFunctionRun4Bar extends LinearOpMode {

    Servo IntakeV4B;
    Servo OuttakeV4B;


    @Override
    public void runOpMode() throws InterruptedException {

        IntakeV4B = hardwareMap.servo.get("Intake V4B");
        OuttakeV4B = hardwareMap.servo.get("Outtake V4B");

        IntakeV4B.setPosition(.8);
        OuttakeV4B.setPosition(1);

        waitForStart();
        while (opModeIsActive()) {

            if(gamepad1.a) {RunV4B("Intake", .5);}
            else if(gamepad1.b) {RunV4B("Intake", .8);}

            if(gamepad1.x) {RunV4B("Outtake", 0);}
            else if (gamepad1.y) {RunV4B("Outtake", 1);}

            telemetry.update();
        }
    }

    public void RunV4B(String V4B, double target) {
        switch (V4B) {
            case "Intake":
                IntakeV4B.setPosition(target);
                break;

            case "Outtake":
                OuttakeV4B.setPosition(target);
                break;

            default:
                telemetry.addData("Invalid V4B", V4B);
                break;
        }
    }
}
