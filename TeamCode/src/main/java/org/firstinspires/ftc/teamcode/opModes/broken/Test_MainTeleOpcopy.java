package org.firstinspires.ftc.teamcode.opModes.broken;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

// ************************ HARDWARE !!!!!  Configuration change ****************
// IntakeLeft/Right changed to Left/RightIntake

/*
@TeleOp(name = "Test copy_Main TeleOp")
@Disabled
public class Test_MainTeleOPcopy extends LinearOpMode {

    private double frontLeftPower = 0;     // declare motor power variable
    private double backLeftPower = 0;      // declare motor power variable
    private double frontRightPower = 0;    // declare motor power variable
    private double backRightPower = 0;     // declare motor power variable
    private double denominator = 1;        // declare motor power calculation variable

    private final int MAX_TARGET_LIFT = 2825;       // The max Lift Height
    private final int MAX_EXTENSION_LENGTH = 500;   // The max Extension Length

    private int TargetLift = 700;                   // The Lift target position
    private int ExtensionTarget = 0;                // The Extension target position

    private double LiftPower = .9;                  // The Power set to the lift
    private double ExtensionPower = .5;            // The Power set to the extension

    private int precision = 2;                                  // chassis motor power reduction factor 1
    private boolean IntakeClawClosed = false;                   // claw holder variable
    private boolean OuttakeClawClosed = false;                  // claw holder variable
    private boolean LiftDown = true;                            // Is the Lift all the way down

    private ElapsedTime Transfer_Time = new ElapsedTime();      // Timer to keep track of the transfer time
    private ElapsedTime ClawTime = new ElapsedTime();           // Timer to keep track since the claw was used last

    private ElapsedTime ClawDelay = new ElapsedTime();

    public double LeftServo;
    public double RightServo;
    private double V4Bpos = 1;
    private double Flex = 0;
    private double Yaw = 0;

    private enum State {
        INTAKE,
        TRANSFER,
        IDLE,
        OUTTAKE,
        CLIMB
    }

    State state = State.INTAKE;

    @Override
    public void runOpMode() {

        //*********************************** MOTORS ************************************************
        DcMotor FrontRight = hardwareMap.dcMotor.get("Front Right");   // Chub Port 0 // Gamepad 1
        DcMotor BackRight = hardwareMap.dcMotor.get("Back Right");     // Chub Port 1 // Left Stick For Moving
        DcMotor FrontLeft = hardwareMap.dcMotor.get("Front Left");     // Chub Port 2 // Right Stick For Turning
        DcMotor BackLeft = hardwareMap.dcMotor.get("Back Left");       // Chub Port 3

        RightIntake = hardwareMap.get(DcMotorEx.class, "Right Intake");  // Ehub Port 0 // X Button To Position Automatically? // Joystick Up And Down?
        LeftIntake = hardwareMap.get(DcMotorEx.class, "Left Intake");   // Ehub Port 1 // ----------------------------------
        RightLift = hardwareMap.get(DcMotorEx.class, "Right Lift");     // Ehub Port 2 // Triangle Button To Delivery Position
        LeftLift = hardwareMap.get(DcMotorEx.class, "Left Lift");       // Ehub Port 3 // ------------------------------------

        //*********************************** SERVOS ************************************************
        Servo IntakeClaw = hardwareMap.servo.get("Intake Claw");              // Chub Port 0 // O Button
        Servo RightIntakeWrist = hardwareMap.servo.get("Right Intake Wrist"); // Chub Port 1 // Increments Using Dpad Side Buttons?
        Servo LeftIntakeWrist = hardwareMap.servo.get("Left Intake Wrist");   // Chub Port 2 // Ideally Stick Controlled
        Servo IntakeV4B = hardwareMap.servo.get("Intake V4B");     // Chub Port 3 // Preset To Swing Out With X
        // Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------
        //  Servo RightHook = hardwareMap.servo.get("Right Hook");                // Chub Port 5 // Linked To LeftHook Activated At The Same Time

        Servo OuttakeClaw = hardwareMap.servo.get("Outtake Claw");            // Ehub Port 0 // If Slides Up O Activates This Claw
        Servo OuttakeWrist = hardwareMap.servo.get("Outtake Wrist");          // Ehub Port 1 // Preset To Go To Delivery Position With Triangle
        Servo OuttakeV4B = hardwareMap.servo.get("Outtake V4B");   // Ehub Port 2 // Preset With Triangle
        // Servo LeftHook = hardwareMap.servo.get("Left Hook");                  // Ehub Port 4 // Both Players Press A Button TBD Which

        //******************************** Create Gamepad state holding variables **************************
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        // ******************************* Get list of all Hubs in use *************************
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //****************************** REVERSE MOTORS *****************************************************

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        RightIntake.setDirection(DcMotorSimple.Direction.REVERSE);   // Reverses the direction the motor turns

        LeftLift.setDirection(DcMotorSimple.Direction.REVERSE);     // Reverses the direction the motor turns

        //****************************** RESET ENCODERS *******************************************************
        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // Resets the position so it sets it's current position to 0
        LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    // Resets the position so it sets it's current position to 0

        RightIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Resets the position so it sets it's current position to 0
        LeftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Resets the position so it sets it's current position to 0

        //****************************** SET DEFAULT TARGET POSITION ***********************************************
        LeftLift.setTargetPosition(0);     // Makes sure it starts at the set 0
        RightLift.setTargetPosition(0);    // Makes sure it starts at the set 0

        RightIntake.setTargetPosition(0);  // Makes sure it starts at the set 0
        LeftIntake.setTargetPosition(0);   // Makes sure it starts at the set 0

        //****************************** SET MODE TO RUN_TO_POSITION ***************************************************
        LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // Sets the mode so we can say to move the motor a certain amount of ticks
        RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Sets the mode so we can say to move the motor a certain amount of ticks

        RightIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Sets the mode so we can say to move the motor a certain amount of ticks
        LeftIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);    // Sets the mode so we can say to move the motor a certain amount of ticks

        //****************************** SET MOTORS TO BRAKE MODE *****************************************************
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    // Sets the motor to be locked when stopped
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   // Sets the motor to be locked when stopped
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);     // Sets the motor to be locked when stopped
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    // Sets the motor to be locked when stopped

        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);     // Sets the motor to be locked when stopped
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    // Sets the motor to be locked when stopped

        LeftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   // Sets the motor to be locked when stopped
        RightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // Sets the motor to be locked when stopped

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

        //   LeftHook.setPosition(0);    // Sets the left hook to the starting position
        //   RightHook.setPosition(0);   // Sets the right hook to the starting position


        // ************** Set Bulk Caching to MANUAL MODE ********************
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();
        while (opModeIsActive()) {

            // If you are using MANUAL mode, you must clear the BulkCache once per control cycle
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            // Read motor positions
            rIntake = RightIntake.getCurrentPosition();   // Uses 1 bulk-read to obtain ALL the motor data
            lIntake = LeftIntake.getCurrentPosition();
            rLift = RightLift.getCurrentPosition();
            lLift = LeftLift.getCurrentPosition();
            oClaw = OuttakeClaw.getPosition();
            iClaw = IntakeClaw.getPosition();


            //*************************** GAMEPAD COPYING ***************************
            previousGamepad1.copy(currentGamepad1); // Copy previous values to variables
            previousGamepad2.copy(currentGamepad2); // These are used for comparision between last loop & this loop's readings

            currentGamepad1.copy(gamepad1);         // Copy current values to variables
            currentGamepad2.copy(gamepad2);         // These are used instead of the button read
            //*************************** DRIVE CONTROLS **************************************************
            // check for driving input
            double y = -gamepad1.left_stick_y;         // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;       // Measures turning
            if (gamepad1.right_trigger >= 0.75) {      // Checks if the Right Trigger was pressed and if so it continues the stuff in the brackets
                y = gamepad1.left_stick_y;           // Remember, this is reversed!
                x = -gamepad1.left_stick_x * 1.1;    // Counteract imperfect strafing
                rx = gamepad1.right_stick_x;          // Measures turning
            }
            switch(state) {
                case INTAKE:
                    if (gamepad2.x){
                        ExtensionTarget = 490;
                    }
                    else if(gamepad2.b){
                        ExtensionTarget = 1;
                    }
                    if (lIntake <= 5 && iClaw == .5){
                        Transfer_Time.reset();
                        state = State.TRANSFER;
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
                    if (gamepad2.dpad_left && ExtensionTarget < MAX_EXTENSION_LENGTH){
                        ExtensionTarget = ExtensionTarget + 10;
                    }
                    else if (gamepad2.dpad_right && ExtensionTarget > 10){
                        ExtensionTarget = ExtensionTarget - 10;
                    }
                    if (gamepad1.touchpad_finger_1) {
                        Yaw = gamepad1.touchpad_finger_1_x;
                    }
                    else {
                        Yaw = 0;
                    }
                    break;

                case TRANSFER:
                    if(iClaw) == 0){
                    state = State.IDLE;
                    }
                    if (Transfer_Time.seconds() >= .5) {
                        TargetLift = 480;
                    }
                    else {
                        TargetLift = 800;
                    }
                    if(lLift < 484){
                        OuttakeClawClosed = true;
                        OuttakeClaw.setPosition(1);
                        if(oClaw == 1){
                            IntakeClawClosed = false;
                            IntakeClaw.setPosition(0);
                        }
                    }
                    break;

                case IDLE:
                    if (gamepad2.y) {
                        TargetLift = 2520;
                        state = State.OUTTAKE;
                    }
                    // Todo A conditional statement to break if time is < .3 will simplify all the others
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
                    break;

                case OUTTAKE:
                    if (gamepad2.a){
                        TargetLift = 750;
                        state = State.INTAKE;
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
                    if (gamepad2.right_trigger >= .75 && TargetLift < MAX_TARGET_LIFT - 10){
                        TargetLift = TargetLift + 10;
                    }
                    else if (gamepad2.left_trigger >= .75 && TargetLift > 10){
                        TargetLift = TargetLift - 10;
                    }
                    break;

                case CLIMB:

                    if (gamepad2.right_trigger >= .75 && TargetLift < MAX_TARGET_LIFT - 10){
                        TargetLift = TargetLift + 10;
                    }
                    else if (gamepad2.left_trigger >= .75 && TargetLift > 50){
                        TargetLift = TargetLift - 50;
                    }
                    if(gamepad2.dpad_down){
                        TargetLift = 600;
                        state = State.INTAKE;
                    }
                    break;
            }
            if(gamepad1.dpad_up && gamepad2.dpad_up){
                TargetLift = MAX_TARGET_LIFT;
                state = State.CLIMB;
            }
            if(lLift >= 800 && lLift <= MAX_TARGET_LIFT - 5){  // (800 < lLift <= MAX_TARGET_LIFT - 5) ???
                OuttakeV4B.setPosition(0);
                OuttakeWrist.setPosition(.7);
            }
            else {
                OuttakeV4B.setPosition(1);
                OuttakeWrist.setPosition(0);
            }
// ReWrite of code below - reversed logic and simplified trigger input factoring
            if(lIntake > 50){       // Positional override when intake is retracted
                V4Bpos = 0.5*(1-(gamepad1.left_trigger));   // Control for variable virtual four bar height when in INTAKE state
                Flex = 0.63;
            }
            else{
                 V4Bpos = 1;}
                 Flex = 0;
                 Yaw = 0;
            }
/*          if(lIntake <= 50){
                Flex = 0;
                V4Bpos = 1;
            }
            else{
                if (gamepad1.left_trigger > 0){
                    V4Bpos = 0.5*(1-(gamepad1.left_trigger)); //Control for variable virtual four bar height when in INTAKE state
                }
                else {
                    V4Bpos = .5;}
                Flex = .63;
            }
        */
/*

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

            IntakeV4B.setPosition(V4Bpos);
            //RightIntakeV4B.setPosition(V4Bpos);
            LeftServo = Math.max(0, Math.min(1, Flex - (.5 * Yaw)));
            RightServo = Math.max(0, Math.min(1, Flex + (.5 * Yaw)));
            LeftIntakeWrist.setPosition(LeftServo); //Sets servos to calculated positions
            RightIntakeWrist.setPosition(RightServo); //^

            // Lift Power and movement
            if (!(TargetLift > MAX_TARGET_LIFT)) {          // If the lift target height is < the max height
                RightLift.setTargetPosition(TargetLift);    // Sets the right lift motor to turn until it's ticks = lift target
                LeftLift.setTargetPosition(TargetLift);     // Sets the left lift motor to turn until it's ticks = lift target
                RightLift.setPower(LiftPower);              // Sets the power to the Right lift motor
                LeftLift.setPower(LiftPower);               // Sets the power to the left lift motor
            }
            // Intake power and movement
            if (!(ExtensionTarget > MAX_EXTENSION_LENGTH)){          // Extension target < the max height
                LeftIntake.setTargetPosition(ExtensionTarget);  // Sets the Intake Motors to a synced position
                RightIntake.setTargetPosition(ExtensionTarget); // Sets the Intake Motors to a synced position
                LeftIntake.setPower(ExtensionPower);            // Sets the Motor Power to ExtensionPower Declared Above
                RightIntake.setPower(ExtensionPower);           // Sets the Motor Power to ExtensionPower Declared Above
            }

            telemetry.addData("Yaw", Yaw);
            telemetry.addData("Flex", Flex);
            telemetry.addData("Left Wrist Position", LeftServo);
            telemetry.addData("Right Wrist Position", RightServo);
            telemetry.addData("state", state);
            telemetry.update();

        }

    }

 */
