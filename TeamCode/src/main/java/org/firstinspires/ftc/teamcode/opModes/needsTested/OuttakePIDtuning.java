package org.firstinspires.ftc.teamcode.opModes.needsTested;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class OuttakePIDtuning extends OpMode {

    private PIDController OuttakeController;
    public static double Op = 0, Oi = 0, Od = 0;
    public static double Of = 0;

    public static int OuttakeTarget = 0;

    private final double outtake_ticks_in_degrees = 1;

    private CRServo OuttakeServo;
    private AnalogInput OuttakeInput;

    boolean looped = false;
    double position;
    double additive;

    public static double OldVoltage, CurrentVoltage;

    @Override
    public void init() {
        OuttakeController = new PIDController(Op, Oi, Od);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        OuttakeServo = hardwareMap.get(CRServo.class, "Outtake Wrist");
        OuttakeInput = hardwareMap.get(AnalogInput.class, "Outtake V4B");

    }

    @Override
    public void loop(){
        OuttakeController.setPID(Op, Oi, Od);

        CurrentVoltage = OuttakeInput.getVoltage();   //read  voltage

        if (Math.abs(OldVoltage - CurrentVoltage) > 3) {          // compare last
            looped = !looped;
        }
                                    // if over rate set additive to formula
        if(looped) {
            additive = 360;
        }
        else { additive = 0; }
        position = (CurrentVoltage / 3.3 * 360) + additive;
        //double OuttakeMultiplier = 5.5;
        double OuttakePos = position;

        double Opid = OuttakeController.calculate(OuttakePos, OuttakeTarget);

        double OuttakeFF = Math.cos(Math.toRadians(OuttakeTarget / outtake_ticks_in_degrees)) * Of;

        double OuttakePower = Opid + OuttakeFF;

        OuttakeServo.setPower(OuttakePower);

        OldVoltage = CurrentVoltage;

        telemetry.addData("Outtake pos ", OuttakePos);
        telemetry.addData("Outtake target ", OuttakeTarget);
        telemetry.update();

    }

}

