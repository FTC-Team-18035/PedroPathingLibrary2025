package org.firstinspires.ftc.teamcode.opModes.broken;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Run Extension Test Built In Function BROKEN")
public class BuiltInFunctionRunExtension extends LinearOpMode {

    private DcMotorEx IntakeLeft;
    private DcMotorEx IntakeRight;

    private static PIDController ExtendController;
    public static double Ep = .01, Ei = 0, Ed = .0004;
    public static double Ef = 0;
    public static int TargetExtend = 0;
    private final double extend_ticks_in_degrees = .403;

    public static double ExtendPos;

    private final int MAX_EXTENSION_LENGTH = 415;





    @Override
    public void runOpMode() {
        IntakeLeft = hardwareMap.get(DcMotorEx.class, "Intake Left");
        IntakeRight = hardwareMap.get(DcMotorEx.class, "Intake Right");

        ExtendController = new PIDController(Ep, Ei, Ed);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        IntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        IntakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        IntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        IntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Intake Pos", ExtendPos);
            telemetry.addData("Intake Target", TargetExtend);
            if (gamepad1.a) {
                RunExtension(415, MAX_EXTENSION_LENGTH);
            } else if (gamepad1.b) {
                RunExtension(1, MAX_EXTENSION_LENGTH);
            }
            telemetry.update();
        }
    }

    public void RunExtension(int target, int MaxExtendIntake) {
        if (target <= MaxExtendIntake) {
            TargetExtend = target;
        }
        ExtendPos = IntakeLeft.getCurrentPosition();
        ExtendController.setPID(Ep, Ei, Ed);
        double Epid = ExtendController.calculate(ExtendPos, TargetExtend);

        double ExtendPower = Epid;
        IntakeLeft.setPower(ExtendPower);
        IntakeRight.setPower(ExtendPower);
    }
}
