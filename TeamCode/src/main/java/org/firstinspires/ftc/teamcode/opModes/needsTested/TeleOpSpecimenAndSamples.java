package org.firstinspires.ftc.teamcode.opModes.needsTested;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Main TeleOp Swappable Target States")
public class TeleOpSpecimenAndSamples extends LinearOpMode {

    private PIDController LiftController;
    private PIDController ExtendController;

    public static double Lp = 0.015, Li = 0, Ld = 0.0002;
    public static double Ep = .01, Ei = 0, Ed = .0004;
    public static double Lf = 0.04;
    public static double Ef = 0;

    public static int TargetLift = 750;
    public static int TargetExtend = 0;

    private final double lift_ticks_in_degrees = 1.068055;
    private final double extend_ticks_in_degrees = .403;

    private DcMotorEx LeftLift, RightLift, IntakeLeft, IntakeRight;
    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight;
    private Servo RightIntakeWrist, LeftIntakeWrist, OuttakeV4B, OuttakeWrist, IntakeV4B, IntakeClaw, OuttakeClaw;

    private double frontLeftPower = 0;     // declare motor power variable
    private double backLeftPower = 0;      // declare motor power variable
    private double frontRightPower = 0;    // declare motor power variable
    private double backRightPower = 0;     // declare motor power variable
    private double denominator = 1;        // declare motor power calculation variable

    private final int MAX_TARGET_LIFT = 2825;       // The max Lift Height
    private final int MAX_EXTENSION_LENGTH = 415;   // The max Extension Length

    private int precision = 2;                                  // chassis motor power reduction factor 1
    private boolean IntakeClawClosed = false;                   // claw holder variable
    private boolean OuttakeClawClosed = false;                  // claw holder variable

    private ElapsedTime Transfer_Time = new ElapsedTime();      // Timer to keep track of the transfer time
    private ElapsedTime Transfer_Delay = new ElapsedTime();
    private ElapsedTime ClawTime = new ElapsedTime();           // Timer to keep track since the claw was used last

    private ElapsedTime ChangeTargetStateTime = new ElapsedTime();

    public double LeftServo;
    public double RightServo;
    private double V4Bpos = 1;
    private double Flex = 0;
    private double Yaw = 0;
    private boolean Transfered = false;

    private enum SampleState {
        SWAP,
        INTAKE,
        TRANSFER,
        IDLE,
        OUTTAKE,
        CLIMB
    }

    private enum SpecimenState {
        SWAP,
        INTAKE,
        DELIVER
    }

    private enum CurrentTarget {
        SAMPLES,
        SPECIMENS
    }

    CurrentTarget currentTarget = CurrentTarget.SAMPLES;
    SampleState sampleState = SampleState.INTAKE;
    SpecimenState specimenState = SpecimenState.INTAKE;

    @Override
    public void runOpMode(){
        LiftController = new PIDController(Lp, Li, Ld);
        ExtendController = new PIDController(Ep, Ei, Ed);
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


        //*********************************** MOTORS ************************************************
        FrontRight = hardwareMap.dcMotor.get("Front Right");   // Chub Port 0 // Gamepad 1
        BackRight = hardwareMap.dcMotor.get("Back Right");     // Chub Port 1 // Left Stick For Moving
        FrontLeft = hardwareMap.dcMotor.get("Front Left");     // Chub Port 2 // Right Stick For Turning
        BackLeft = hardwareMap.dcMotor.get("Back Left");       // Chub Port 3

        //*********************************** SERVOS ************************************************
        IntakeClaw = hardwareMap.servo.get("Intake Claw");              // Chub Port 0 // O Button
        RightIntakeWrist = hardwareMap.servo.get("Right Intake Wrist"); // Chub Port 1 // Increments Using Dpad Side Buttons?
        LeftIntakeWrist = hardwareMap.servo.get("Left Intake Wrist");   // Chub Port 2 // Ideally Stick Controlled
        IntakeV4B = hardwareMap.servo.get("Intake V4B");     // Chub Port 3 // Preset To Swing Out With X
        // Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------
        //  Servo RightHook = hardwareMap.servo.get("Right Hook");                // Chub Port 5 // Linked To LeftHook Activated At The Same Time

        OuttakeClaw = hardwareMap.servo.get("Outtake Claw");            // Ehub Port 0 // If Slides Up O Activates This Claw
        OuttakeWrist = hardwareMap.servo.get("Outtake Wrist");          // Ehub Port 1 // Preset To Go To Delivery Position With Triangle
        OuttakeV4B = hardwareMap.servo.get("Outtake V4B");   // Ehub Port 2 // Preset With Triangle
        // Servo LeftHook = hardwareMap.servo.get("Left Hook");                  // Ehub Port 4 // Both Players Press A Button TBD Which

        LeftServo = Flex - (.5 * Yaw);
        RightServo = Flex + (.5 * Yaw);

        //****************************** REVERSE MOTORS *****************************************************

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //****************************** RESET ENCODERS *******************************************************

        //****************************** SET DEFAULT TARGET POSITION ***********************************************


        //****************************** SET MODE TO RUN_TO_POSITION ***************************************************

        //****************************** SET MOTORS TO BRAKE MODE *****************************************************
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);    // Sets the motor to be locked when stopped
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // Sets the motor to be locked when stopped
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);     // Sets the motor to be locked when stopped
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);    // Sets the motor to be locked when stopped

        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);     // Sets the motor to be locked when stopped
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);    // Sets the motor to be locked when stopped

        IntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // Sets the motor to be locked when stopped
        IntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // Sets the motor to be locked when stopped

        //***************************** RESET SERVOS ***********************************************************
        IntakeClaw.setPosition(0);    // Closes Intake Claw

        OuttakeClaw.setPosition(0);   // Closes Outtake Claw

        LeftServo = Math.max(0, Math.min(1, Flex - (.5 * Yaw)));
        RightServo = Math.max(0, Math.min(1, Flex + (.5 * Yaw)));

        LeftIntakeWrist.setPosition(LeftServo);    // Sets the intake wrist to the starting position // Left is 0 Right is 1
        RightIntakeWrist.setPosition(RightServo);   // Sets the intake wrist to the starting position // Left is 0 Right is 1
        IntakeV4B.setPosition(.8);   // Sets the intake virtual four bar to the starting position
        //RightIntakeV4B.setPosition(1);  // Sets the intake virtual four bar to the starting position

        OuttakeWrist.setPosition(0);    // Sets the outtake wrist to the starting position

        OuttakeV4B.setPosition(1);  // Sets the outtake virtual four bar to the starting position
        //RightOuttakeV4B.setPosition(0); // Sets the outtake virtual four bar to the starting position

        //   LeftHook.setPosition(0);    // Sets the left hook to the starting position
        //   RightHook.setPosition(0);   // Sets the right hook to the starting position


        waitForStart();
        while (opModeIsActive()){

            double IntakePos = IntakeLeft.getCurrentPosition();
            double LiftPos = LeftLift.getCurrentPosition();

            DriveControls();

            switch(currentTarget) {
                case SAMPLES:
                    Samples(IntakePos, LiftPos);
                    IntakeV4BCalc();
                    if(Transfered){
                        OuttakeV4B.setPosition(0);
                        OuttakeWrist.setPosition(.7);
                    }
                    else {
                        OuttakeV4B.setPosition(1);
                        OuttakeWrist.setPosition(.03);
                    }

                    if(IntakePos <= 50 && Transfer_Delay.seconds() >= 2){
                        Flex = 0;
                        V4Bpos = 1;

                    }
                    else if(Transfer_Delay.seconds() >= 2){
                        if (gamepad1.left_trigger > 0){
                            V4Bpos = 0.5*(1-(gamepad1.left_trigger)); //Control for variable virtual four bar height when in INTAKE state
                        }
                        else {
                            V4Bpos = .5;}
                        Flex = .63;
                    }
                    break;

                case SPECIMENS:
                    Specimens();
                    break;
            }

            if(gamepad1.dpad_left && gamepad1.dpad_right && ChangeTargetStateTime.seconds() > 5) {
                ChangeTargetStateTime.reset();
                ChangeTargets();
            }

            if(gamepad1.dpad_up && gamepad2.dpad_up){
                TargetLift = MAX_TARGET_LIFT - 10;
                currentTarget = CurrentTarget.SAMPLES;
                sampleState = SampleState.CLIMB;
            }


            if(sampleState != SampleState.TRANSFER) {
                if (gamepad2.right_trigger >= .75 && TargetLift < MAX_TARGET_LIFT - 10){
                    TargetLift = TargetLift + 10;
                }
                else if (gamepad2.left_trigger >= .75 && TargetLift > 10){
                    TargetLift = TargetLift - 10;
                }
            }

            PIDcalc(IntakePos, LiftPos);

            RunTelemetry(IntakePos, LiftPos);
        }
    }

    public void Samples(double intakePos, double liftPos) {
        switch(sampleState) {
            case SWAP:
                IntakeClaw.setPosition(0);

                OuttakeClaw.setPosition(0);

                LeftIntakeWrist.setPosition(LeftServo);
                IntakeV4B.setPosition(.8);
                OuttakeWrist.setPosition(0);
                OuttakeV4B.setPosition(1);

                TargetExtend = 0;
                TargetLift = 750;
                sampleState = SampleState.INTAKE;

            case INTAKE:
                if (gamepad2.x){
                    TargetExtend = MAX_EXTENSION_LENGTH;
                }
                else if(gamepad2.b){
                    TargetExtend = 1;
                }
                if (intakePos <= 7 && IntakeClawClosed){
                    Transfer_Time.reset();
                    Transfer_Delay.reset();
                    Transfered = false;
                    sampleState = SampleState.TRANSFER;
                }
                if (gamepad2.right_bumper || gamepad1.b && !IntakeClawClosed && ClawTime.seconds() >= .3){
                    ClawTime.reset();
                    IntakeClaw.setPosition(.5);
                    IntakeClawClosed = true;
                }
                else if (gamepad2.right_bumper || gamepad1.b && IntakeClawClosed && ClawTime.seconds() >= 1){
                    ClawTime.reset();
                    IntakeClaw.setPosition(0);
                    IntakeClawClosed = false;
                }
                if (gamepad2.dpad_left && TargetExtend < MAX_EXTENSION_LENGTH){
                    TargetExtend = TargetExtend + 10;
                }
                else if (gamepad2.dpad_right && TargetExtend > 10){
                    TargetExtend = TargetExtend - 10;
                }

                if (gamepad1.touchpad_finger_1) {
                    Yaw = gamepad1.touchpad_finger_1_x;
                }
                else {
                    Yaw = 0;
                }
                break;
            case TRANSFER:
                if (Transfer_Time.seconds() >= .25) {
                    TargetLift = 480;
                }
                else {
                    TargetLift = 800;
                }
                if(liftPos < 484 && Transfer_Delay.seconds() >= .6){
                    OuttakeClawClosed = true;
                    OuttakeClaw.setPosition(1);
                    if(OuttakeClawClosed && Transfer_Delay.seconds() >= 1.1){
                        IntakeClawClosed = false;
                        IntakeClaw.setPosition(0);
                    }
                }
                if(!IntakeClawClosed){
                    V4Bpos = .75;
                    if(Transfer_Delay.seconds() >= 1.5) {
                        sampleState = SampleState.IDLE;
                        Transfered = true;
                    }
                }
                break;

            case IDLE:
                if (gamepad2.y) {
                    TargetLift = 2520;
                    sampleState = SampleState.OUTTAKE;
                }
                if (gamepad2.left_bumper && !OuttakeClawClosed && ClawTime.seconds() >= .3){
                    ClawTime.reset();
                    OuttakeClaw.setPosition(.5);
                    OuttakeClawClosed = true;
                }
                else if (gamepad2.left_bumper && OuttakeClawClosed && ClawTime.seconds() >= .3) {
                    ClawTime.reset();
                    OuttakeClaw.setPosition(0);
                    OuttakeClawClosed = false;
                }
                if (gamepad2.right_bumper  && !IntakeClawClosed && ClawTime.seconds() >= .3){
                    ClawTime.reset();
                    IntakeClaw.setPosition(.5);
                    IntakeClawClosed = true;
                }
                else if (gamepad2.right_bumper && IntakeClawClosed && ClawTime.seconds() >= .3){
                    ClawTime.reset();
                    IntakeClaw.setPosition(0);
                    IntakeClawClosed = false;
                }
                if (gamepad2.a){
                    TargetLift = 750;
                    Transfered = false;
                    sampleState = SampleState.INTAKE;
                }
                break;
            case OUTTAKE:
                if (gamepad2.a){
                    TargetLift = 750;
                    Transfered = false;
                    sampleState = SampleState.INTAKE;
                }
                if (gamepad2.left_bumper && !OuttakeClawClosed && ClawTime.seconds() >= .3){
                    ClawTime.reset();
                    OuttakeClaw.setPosition(.5);
                    OuttakeClawClosed = true;
                }
                else if (gamepad2.left_bumper && OuttakeClawClosed && ClawTime.seconds() >= .3) {
                    ClawTime.reset();
                    OuttakeClaw.setPosition(0);
                    OuttakeClawClosed = false;
                }
                break;

            case CLIMB:
                if(gamepad2.dpad_down){
                    TargetLift = 600;
                    sampleState = SampleState.INTAKE;
                }
                break;
        }
    }

    public void Specimens() {
        switch(specimenState) {
            case SWAP:
                /********** RESET TO INIT POSITIONS FOR THIS TARGET **************
                IntakeClaw.setPosition(0);

                OuttakeClaw.setPosition(0);

                LeftIntakeWrist.setPosition(LeftServo);
                IntakeV4B.setPosition(.8);
                OuttakeWrist.setPosition(0);
                OuttakeV4B.setPosition(1);

                TargetExtend = 0;
                TargetLift = 750;
                *******************************/
                specimenState = SpecimenState.INTAKE;
                break;

            case INTAKE:
                OuttakeV4B.setPosition(1); // Grab Position
                OuttakeWrist.setPosition(.5); // Grab Position
                if(gamepad2.right_bumper && !OuttakeClawClosed && ClawTime.seconds() >= .5) {
                    OuttakeClawClosed = true;
                    OuttakeClaw.setPosition(0);
                }
                else if(gamepad2.right_bumper && OuttakeClawClosed && ClawTime.seconds() >= .5) {
                    OuttakeClawClosed = false;
                    OuttakeClaw.setPosition(1);
                }
                if(gamepad2.b) {
                    specimenState = SpecimenState.DELIVER;
                }
                break;

            case DELIVER:
                OuttakeV4B.setPosition(1); // Deliver Position
                OuttakeWrist.setPosition(.5); // Deliver Position
                if(gamepad2.a) {
                    TargetLift = 500; // Deliver Height
                }
                if(gamepad2.x) {
                    specimenState = SpecimenState.INTAKE;
                }
        }
    }

    public void ChangeTargets() {
        switch (currentTarget) {
            case SAMPLES:
                specimenState = SpecimenState.SWAP;
                currentTarget = CurrentTarget.SPECIMENS;
                break;

            case SPECIMENS:
                sampleState = SampleState.SWAP;
                currentTarget = CurrentTarget.SAMPLES;
                break;
        }
    }

    public void DriveControls() {
        //*************************** DRIVE CONTROLS **************************************************
        // check for driving input
        double y = -gamepad1.left_stick_y;         // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;       // Measures turning
        if (gamepad1.right_trigger >= 0.75) {      // Checks if the Right Trigger was pressed and if so it continues the stuff in the brackets
            y = gamepad1.left_stick_y;           // Remember, this is reversed!
            x = -gamepad1.left_stick_x * 1.1;    // Counteract imperfect strafing
            rx = gamepad1.right_stick_x;          // Measures turning

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);    // calculate motor movement math and adjust according to lift height or manual precision mode selection

            // check for Turbo or Precision Mode
            if (gamepad1.left_bumper) {         // Left bumper is being pressed
                precision = 1;                  // set speed to full power - TURBO MODE
            } else if (gamepad1.right_bumper) { // Right bumper is being pressed
                precision = 4;                  // set speed to 1/4 power - PRECISION MODE
            } else {
                precision = 2;                  // reset default speed to half power
            }

            // calculate motor power
            denominator = denominator * precision;          // this adjusts motor speed to the desired precision level
            frontLeftPower = (y + x + rx) / denominator;    // Does math to convert stick movement to motor powers
            backLeftPower = (y - x + rx) / denominator;     // Does math to convert stick movement to motor powers
            frontRightPower = (y - x - rx) / denominator;   // Does math to convert stick movement to motor powers
            backRightPower = (y + x - rx) / denominator;    // Does math to convert stick movement to motor powers

            // issue Drive Wheels motor power
            FrontLeft.setPower(frontLeftPower);    // Sets the front left wheel's power
            BackLeft.setPower(backLeftPower);     // Sets the back left wheel's power
            FrontRight.setPower(frontRightPower);  // Sets the front right wheel's power
            BackRight.setPower(backRightPower);   // Sets the back right wheel's power

        }
    }

    public void PIDcalc(double intakePos, double liftPos) {
        LiftController.setPID(Lp, Li, Ld);
        ExtendController.setPID(Ep, Ei, Ed);

        double Lpid = LiftController.calculate(liftPos, TargetLift);
        double Epid = ExtendController.calculate(intakePos, TargetExtend);
        double LiftFF = Math.cos(Math.toRadians(TargetLift / lift_ticks_in_degrees)) * Lf;

        double LiftPower = Lpid + LiftFF;
        double ExtendPower = Epid;

        LeftLift.setPower(LiftPower);
        RightLift.setPower(LiftPower);
        IntakeLeft.setPower(ExtendPower);
        IntakeRight.setPower(ExtendPower);

    }

    public void IntakeV4BCalc() {
        IntakeV4B.setPosition(V4Bpos);
        //RightIntakeV4B.setPosition(V4Bpos);
        LeftServo = Math.max(0, Math.min(1, Flex - (.5 * Yaw)));
        RightServo = Math.max(0, Math.min(1, Flex + (.5 * Yaw)));
        LeftIntakeWrist.setPosition(LeftServo); //Sets servos to calculated positions
        RightIntakeWrist.setPosition(RightServo); //^
    }

    public void RunTelemetry(double intakePos, double liftPos) {
        telemetry.addData("Lift pos ", liftPos);
        telemetry.addData("Extend pos ", intakePos);
        telemetry.addData("lift target ", TargetLift);
        telemetry.addData("extend target ", TargetExtend);
        telemetry.addData("sample state", sampleState);
        telemetry.update();
    }
}
