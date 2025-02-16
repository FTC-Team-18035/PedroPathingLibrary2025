package org.firstinspires.ftc.teamcode.opModes.working.extras;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "Run Claws Test Built In Function")
public class BuiltInFunctionRunClaw extends LinearOpMode {

    Servo IntakeClaw, OuttakeClaw;

    private boolean IntakeClawClosed = false;                   // claw holder variable
    private boolean OuttakeClawClosed = false;

    private ElapsedTime ClawTime = new ElapsedTime();

    public void runOpMode() {

        IntakeClaw = hardwareMap.servo.get("Intake Claw");
        OuttakeClaw = hardwareMap.servo.get("Outtake Claw");

        IntakeClaw.setPosition(0);    // Closes Intake Claw

        OuttakeClaw.setPosition(0);   // Closes Outtake Claw

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.right_bumper && !IntakeClawClosed && ClawTime.seconds() >= .3){
                RunClaws("Intake", .5); // .5 Closed
            }
            else if (gamepad2.right_bumper && IntakeClawClosed && ClawTime.seconds() >= 1){
                RunClaws("Intake", 0); // 0 Open
            }
            if (gamepad2.left_bumper && !OuttakeClawClosed && ClawTime.seconds() >= .3){
                RunClaws("Outtake", .5); // .5 Closed
            }
            else if (gamepad2.left_bumper && OuttakeClawClosed && ClawTime.seconds() >= .3) {
                RunClaws("Outtake", 0); // 0 Open
            }

            telemetry.update();
        }
    }

    public void RunClaws(String claw, double target) {
        switch(claw) {
            case "Intake":
                ClawTime.reset();
                IntakeClaw.setPosition(target);
                IntakeClawClosed = !IntakeClawClosed;
                break;
            case "Outtake":
                ClawTime.reset();
                OuttakeClaw.setPosition(target);
                OuttakeClawClosed = !OuttakeClawClosed;
                break;

            default:
                telemetry.addData("Invalid Claw", claw);
                break;
        }
        telemetry.addData("The Claw", claw);
        telemetry.addData("Target Pos", target);
    }

}
