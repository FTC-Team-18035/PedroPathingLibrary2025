package org.firstinspires.ftc.teamcode.opModes.broken;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Run Lift Test Built In Function BROKEN")
public class BuiltInFunctionRunLift extends LinearOpMode {

    private DcMotorEx LeftLift;
    private DcMotorEx RightLift;

    private static PIDController LiftController;
    public static double Lp = 0.015, Li = 0, Ld = 0.0002;
    public static double Lf = 0.04;
    public static int TargetLift = 0;
    private static final double lift_ticks_in_degrees = 1.068055;

    public static double LiftPos;

    private static final int MAX_TARGET_LIFT = 2825;
    private double LiftPower = .9;



    @Override
    public void runOpMode() {
        LeftLift = hardwareMap.get(DcMotorEx .class, "Left Lift");
        RightLift = hardwareMap.get(DcMotorEx.class, "Right Lift");

        LiftController = new PIDController(Lp, Li, Ld);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Lift Pos", LiftPos);
            telemetry.addData("Lift Target", TargetLift);
            if (gamepad1.a) {
                RunLift(1500, MAX_TARGET_LIFT);
            } else if (gamepad1.b) {
                RunLift(1, MAX_TARGET_LIFT);
            }
            updateLift();
            telemetry.update();
        }
    }

    public void RunLift(int target, int MaxTargetLift) {
        if (target <= MaxTargetLift - 5) {
            TargetLift = target;
        }
    }

    public void updateLift() {
        LiftPos = LeftLift.getCurrentPosition();
        LiftController.setPID(Lp, Li, Ld);
        double Lpid = LiftController.calculate(LiftPos, TargetLift);
        double LiftFF = Math.cos(Math.toRadians(TargetLift / lift_ticks_in_degrees)) * Lf;
        double LiftPower = Lpid + LiftFF;
        LeftLift.setPower(LiftPower);
        RightLift.setPower(LiftPower);
    }
}
