package org.firstinspires.ftc.teamcode.helperFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MasterClass extends OpMode {

    private static PIDController LiftController;
    public static double Lp = 0.015, Li = 0, Ld = 0.0002;
    public static double Lf = 0.04;
    public static int TargetLift = 0;
    private static final double lift_ticks_in_degrees = 1.068055;

    public static double LiftPos;

    private static final int MAX_TARGET_LIFT = 2825;
    private double LiftPower = .9;

    public void Init(DcMotor motor1, DcMotor motor2) {
        LiftController = new PIDController(Lp, Li, Ld);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void ExtendLift(int target, DcMotor motor1, DcMotor motor2) {
        if(target <= MAX_TARGET_LIFT) {
            TargetLift = target;
        }
        LiftPos = motor1.getCurrentPosition();
        LiftController.setPID(Lp, Li, Ld);
        double Lpid = LiftController.calculate(LiftPos, TargetLift);
        double LiftFF = Math.cos(Math.toRadians(TargetLift / lift_ticks_in_degrees)) * Lf;
        double LiftPower = Lpid + LiftFF;
        motor1.setPower(LiftPower);
        motor2.setPower(LiftPower);
    }

    @Override
    public void init(){}


    @Override
    public void loop() {}
}
