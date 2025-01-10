package org.firstinspires.ftc.teamcode.opModes.needsTested;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helperFiles.MasterClass;

@TeleOp(name = "MasterClass Extend Lift Test")
public class MasterClassTest_ExtendLift extends OpMode {

    private DcMotorEx LeftLift;
    private DcMotorEx RightLift;

    MasterClass robot = new MasterClass();



    @Override
    public void init() {
        LeftLift = hardwareMap.get(DcMotorEx .class, "Left Lift");
        RightLift = hardwareMap.get(DcMotorEx.class, "Right Lift");

        robot.Init(LeftLift, RightLift);
    }

    @Override
    public void loop() {
        telemetry.addData("Lift Pos", MasterClass.LiftPos);
        telemetry.addData("Lift Target", MasterClass.TargetLift);
        if(gamepad1.a) {
            robot.ExtendLift(1500, LeftLift, RightLift);
        }
        else if(gamepad1.b) {
            robot.ExtendLift(1, LeftLift, RightLift);
        }
        telemetry.update();
    }
}
