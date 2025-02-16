package org.firstinspires.ftc.teamcode.opModes.working.extras;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Disabled
@TeleOp
public class CurrentSensingTest extends LinearOpMode {
    private DcMotorEx LeftLift;
    private DcMotorEx RightLift;
    private DcMotorEx IntakeLeft;
    private DcMotorEx IntakeRight;
    private static double HorizontalCurrentThreshold = 2;
    private static double VerticalCurrentThreshold = 1;
    private double HorizontalCurrent;
    private double VerticalCurrent;
    private double LiftPower;
    private double ExtendPower;
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        LeftLift = hardwareMap.get(DcMotorEx.class, "Left Lift");
        RightLift = hardwareMap.get(DcMotorEx.class, "Right Lift");
        IntakeLeft = hardwareMap.get(DcMotorEx.class, "Intake Left");
        IntakeRight = hardwareMap.get(DcMotorEx.class, "Intake Right");
        LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);   // Reverses the direction the motor turns
        LeftLift.setDirection(DcMotorSimple.Direction.REVERSE);     // Reverses the direction the motor turns
        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);     // Sets the motor to be locked when stopped
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);    // Sets the motor to be locked when stopped
        IntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // Sets the motor to be locked when stopped
        IntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // Sets the motor to be locked when stopped
        waitForStart();
        while (opModeIsActive()) {
            if (opModeIsActive()) {

                HorizontalCurrent = IntakeLeft.getCurrent(CurrentUnit.AMPS);
                VerticalCurrent = LeftLift.getCurrent(CurrentUnit.AMPS);
                ExtendPower = gamepad1.left_stick_x;
                LiftPower = gamepad1.right_stick_x;
                if (gamepad1.x) {
                    ExtendPower = -.25;
                    IntakeLeft.setPower(ExtendPower);
                    IntakeRight.setPower(ExtendPower);
                    while (IntakeLeft.getCurrent(CurrentUnit.AMPS) < HorizontalCurrentThreshold) {
                    }
                    IntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    IntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    LiftPower = -.25;
                    LeftLift.setPower(LiftPower);
                    RightLift.setPower(LiftPower);
                    while (RightLift.getCurrent(CurrentUnit.AMPS) < VerticalCurrentThreshold) {
                    }
                    LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    break;
                }
                LeftLift.setPower(LiftPower);
                RightLift.setPower(LiftPower);
                IntakeLeft.setPower(ExtendPower);
                IntakeRight.setPower(ExtendPower);
                telemetry.addData("Horizontal Current Draw", HorizontalCurrent);
                telemetry.addData("Horizontal Position", IntakeLeft.getCurrentPosition());
                telemetry.addData("Vertical Current Draw", VerticalCurrent);
                telemetry.addData("Vertical Position", RightLift.getCurrentPosition());
                telemetry.addData("Left X", gamepad1.left_stick_x);
                telemetry.addData("Right X", gamepad1.right_stick_x);
                telemetry.update();
            }
        }
    }
}