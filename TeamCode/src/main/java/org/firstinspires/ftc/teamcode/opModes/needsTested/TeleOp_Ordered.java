package org.firstinspires.ftc.teamcode.opModes.needsTested;                                                                     // opModes.working.main

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp                                                                                                                                                            // @TeleOp (group = �Main�)
public class TeleOp_Ordered extends LinearOpMode {

 // ********************* create PIDS for Lift and Extension ************
 private PIDController LiftController;
 private PIDController ExtendController;

 // ******************** PID tuning coefficients **************************
 public static double Lp = 0.015, Li = 0, Ld = 0.0002;
 public static double Ep = .01, Ei = 0, Ed = .0004;
 public static double Lf = 0.04;
 public static double Ef = 0;

 // ******************** Lift and Extension target variables *************
 private static int TargetLift = 750;
 private static int TargetExtend = 0;

 // *********** Calculated Lift and Extension tick measurements ******
 private final double lift_ticks_in_degrees = 1.068055;
 private final double extend_ticks_in_degrees = .403;

 // ****************** Chassis motors *************************************
 private DcMotor FrontRight;
 private DcMotor BackRight;
 private DcMotor FrontLeft;
 private DcMotor BackLeft;

 // ******************  Lift and Extension motors **************************
 private DcMotorEx LeftLift;
 private DcMotorEx RightLift;
 private DcMotorEx IntakeLeft;
 private DcMotorEx IntakeRight;

 // **************** declare Chassis motor control variables *************
 private double frontLeftPower = 0;
 private double backLeftPower = 0;
 private double frontRightPower = 0;
 private double backRightPower = 0;
 private double denominator = 1;            	 // declare motor power calculation variable 
 private int precision = 2;                      // chassis motor power reduction factor 1 Turbo/ 2 Normal/ 4 Precision

 // *********** declare Lift and Extension motor control variables  *********
 private double LiftPower;		// declare lift power variable
 private double ExtendPower;	// declare extension power variable

 // ******** constraint variables for Lift and Extension safety ****************
 private final int MAX_TARGET_LIFT = 2655;               // The max Lift Height
 private final int MAX_EXTENSION_LENGTH = 415;   // The max Extension Length

 // ********** self calibration sequence variables for Lift and Extension **************
 private static double HorizontalCurrentThreshold = 2;		// Variables to enable self reset of Extension and Lift
 private static double VerticalCurrentThreshold = 1;		// CurrentSensingTest has this value
 private double HorizontalCurrent;
 private double VerticalCurrent;

 // ************* switching place holders for state logic and control **************
 private boolean IntakeClawClosed = false;                   // claw holder variable
 private boolean OuttakeClawClosed = false;                  // claw holder variable
 private boolean LiftDown = true;                            // Is the Lift all the way down
 private boolean Transfered = false;
 private boolean Started = false;

 // ************ timers for state logic and control *******************
 private ElapsedTime Transfer_Time = new ElapsedTime();      // Timer to keep track of the transfer time
 private ElapsedTime Transfer_Delay = new ElapsedTime();
 private ElapsedTime ClawTime = new ElapsedTime();           // Timer to keep track since the claw was used last
 private ElapsedTime ClawDelay = new ElapsedTime();
 private ElapsedTime PegLegTime = new ElapsedTime();         // Timer to keep track of how long the peg leg is out � no usage ????

 //***********************************      SERVOS ************************************************
 Servo IntakeClaw = hardwareMap.servo.get("Intake Claw");              // Chub Port 0 // O Button
 Servo RightIntakeWrist = hardwareMap.servo.get("Right Intake Wrist"); // Chub Port 1 // Increments Using Dpad Side Buttons?
 Servo LeftIntakeWrist = hardwareMap.servo.get("Left Intake Wrist");   // Chub Port 2 // Ideally Stick Controlled
 Servo IntakeV4B = hardwareMap.servo.get("Intake V4B");     // Chub Port 3 // Preset To Swing Out With X
 Servo OuttakeClaw = hardwareMap.servo.get("Outtake Claw");            // Ehub Port 0 // If Slides Up O Activates This Claw
 Servo OuttakeWrist = hardwareMap.servo.get("Outtake Wrist");          // Ehub Port 1 // Preset To Go To Delivery Position With Triangle
 Servo OuttakeV4B = hardwareMap.servo.get("Outtake V4B");   // Ehub Port 2 // Preset With Triangle
 Servo PegLeg = hardwareMap.servo.get("Peg Leg");            // Ehub Port 5 // Gamepad 2 Dpad down

 // ****************** Servo position variables ********************
 public double LeftServo;		// servo position variables
 public double RightServo;
 private double V4Bpos = 1;
 private double Flex = 0;
 private double Yaw = 0;

 // ************** State machine variables ************************
 private enum State {
     INTAKE,
     TRANSFER,
     IDLE,
     OUTTAKE,
     CLIMB
 }

 // ************** set initial state of machine ************************
 State state = State.INTAKE;

 // ******************** call the class for looping opmode ***********************
 @Override
 public void runOpMode() {
      
    // ****************** initialization stage of opmode *********************************

    // ******************** set up FTC Dashboard *****************************************
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); 

    // ************************ PID CONTROLLERS *****************************************
    LiftController = new PIDController(Lp, Li, Ld);
    ExtendController = new PIDController(Ep, Ei, Ed);

    // *********************************** MOTORS *****************************************
    FrontRight = hardwareMap.dcMotor.get("Front Right");   // Chub Port 0 // Gamepad 1
    BackRight = hardwareMap.dcMotor.get("Back Right");     // Chub Port 1 // Left Stick For Moving
    FrontLeft = hardwareMap.dcMotor.get("Front Left");     // Chub Port 2 // Right Stick For Turning
    BackLeft = hardwareMap.dcMotor.get("Back Left");       // Chub Port 3

    LeftLift = hardwareMap.get(DcMotorEx.class, "Left Lift");
    RightLift = hardwareMap.get(DcMotorEx.class, "Right Lift");
    IntakeLeft = hardwareMap.get(DcMotorEx.class, "Intake Left");
    IntakeRight = hardwareMap.get(DcMotorEx.class, "Intake Right");

    // *********************************** SERVOS ************************************************
    Servo IntakeClaw = hardwareMap.servo.get("Intake Claw");              // Chub Port 0 // O Button
    Servo RightIntakeWrist = hardwareMap.servo.get("Right Intake Wrist"); // Chub Port 1 // Increments Using Dpad Side Buttons?
    Servo LeftIntakeWrist = hardwareMap.servo.get("Left Intake Wrist");   // Chub Port 2 // Ideally Stick Controlled
    Servo IntakeV4B = hardwareMap.servo.get("Intake V4B");                // Chub Port 3 // Preset To Swing Out With X
    Servo OuttakeClaw = hardwareMap.servo.get("Outtake Claw");            // Ehub Port 0 // If Slides Up O Activates This Claw
    Servo OuttakeWrist = hardwareMap.servo.get("Outtake Wrist");          // Ehub Port 1 // Preset To Go To Delivery Position With Triangle

    Servo PegLeg = hardwareMap.servo.get("Peg Leg");                      // Ehub Port 5 // Gamepad 2 Dpad down

    // ****************************** REVERSE MOTORS *****************************************************
    FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    IntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);   // Reverses the direction the motor turns
    LeftLift.setDirection(DcMotorSimple.Direction.REVERSE);     // Reverses the direction the motor turns

    // ****************************** RESET ENCODERS *******************************************************
    LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    IntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    IntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // ****************************** SET MODE TO RUN_WITHOUT_ENCODER ****************************************
    LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    IntakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    IntakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    // ****************************** SET MOTORS TO FLOAT MODE *****************************************************
    FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);    // Sets the motor to float when stopped
    FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // Sets the motor to float when stopped
    BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);     // Sets the motor to float when stopped
    BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);    // Sets the motor to float when stopped

    LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);     // Sets the motor to float when stopped
    RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);    // Sets the motor to float when stopped

    IntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // Sets the motor to float when stopped
    IntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // Sets the motor to float when stopped

    // *********************************** wrist calculation formula **********************************************
    LeftServo = Math.max(0, Math.min(1, Flex - (.5 * Yaw)));	// intake wrist calculation formula
    RightServo = Math.max(0, Math.min(1, Flex + (.5 * Yaw)));

    waitForStart();

     // *************************** set claw startup positions ******************************
     IntakeClaw.setPosition(0);     // Closes Intake Claw
     OuttakeClaw.setPosition(.45);  // Closes Outtake Claw

     // ******************** set initial Servo positions *****************************
     LeftIntakeWrist.setPosition(LeftServo);    // Sets the intake wrist to the starting position // Left is 0 Right is 1
     RightIntakeWrist.setPosition(RightServo);  // Sets the intake wrist to the starting position // Left is 0 Right is 1
     OuttakeV4B.setPosition(.5);
     IntakeV4B.setPosition(.8);
     OuttakeWrist.setPosition(.25);
     PegLeg.setPosition(0);

     // *********************** Extension and Lift reset sequence ***********************************
     HorizontalCurrent = IntakeLeft.getCurrent(CurrentUnit.AMPS);       	 // read Extension motor amperage for comparison
     VerticalCurrent = LeftLift.getCurrent(CurrentUnit.AMPS);                // read Lift  motor amperage for comparison
     ExtendPower = -.25;                                                     // begin low power Extension reset sequence
     IntakeLeft.setPower(ExtendPower);
     IntakeRight.setPower(ExtendPower);
     while (IntakeLeft.getCurrent(CurrentUnit.AMPS) < HorizontalCurrentThreshold) { }  // wait for complete Extension retraction
     IntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                       // reset position
     IntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     LiftPower = -.25;                                                                                                  // begin low power Lift reset sequence
     LeftLift.setPower(LiftPower);
     RightLift.setPower(LiftPower);
     while (RightLift.getCurrent(CurrentUnit.AMPS) < VerticalCurrentThreshold) { }     // wait for complete Lift retraction
     LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);              // reset position
     RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

     // ****************************** set Extension and Lift to run with PID control ***************************************
     IntakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     IntakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

     // *************************begin LOOP ACTIVITY**************************
     while (opModeIsActive()) {	

         //*************************** DRIVE CONTROLS **************************************************
         // check for driving input
         double y = -gamepad1.left_stick_y;         // Remember, this is reversed!
         double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
         double rx = gamepad1.right_stick_x;       // Measures turning
         if (gamepad1.right_trigger >= 0.75) {      // Checks if the Right Trigger was pressed and if so translates movement in reverse
           y = gamepad1.left_stick_y;             // Remember, this is reversed!
           x = -gamepad1.left_stick_x * 1.1;    // Counteract imperfect strafing
           rx = gamepad1.right_stick_x;          // Measures turning
         }
        
         // **************************** start of STATE MACHINE ****************************
         switch (state) {

            case INTAKE:                    			// Intake state activities
                if (gamepad2.square) {					// intake extension and retraction
                    TargetExtend = MAX_EXTENSION_LENGTH;
                } else if (gamepad2.circle) {
                    TargetExtend = 1;
                }
                if (IntakeLeft.getCurrentPosition() <= 7 && IntakeClaw.getPosition() == .5) {     // switch to Transfer position and state upon acquiring
                    Transfer_Time.reset();
                    Transfer_Delay.reset();
                    Transfered = false;
                    state = State.TRANSFER;
                }
                if (gamepad2.right_bumper && !IntakeClawClosed && ClawTime.seconds() >= .3) { // driver 2 claw open and close toggle
                    ClawTime.reset();
                    IntakeClaw.setPosition(.5);  // close claw
                    IntakeClawClosed = true;
                } else if (gamepad2.right_bumper && IntakeClawClosed && ClawTime.seconds() >= 1) {
                    ClawTime.reset();
                    IntakeClaw.setPosition(0);  // open claw
                    IntakeClawClosed = false;
                }
                if (gamepad2.dpad_left && TargetExtend < MAX_EXTENSION_LENGTH) {    // driver 2 manual intake extension and retraction
                    TargetExtend = TargetExtend + 10;
                } else if (gamepad2.dpad_right && TargetExtend > 10) {
                    TargetExtend = TargetExtend - 10;
                }
                Yaw = gamepad1.touchpad_finger_1_x;                  			    // driver 1 yaw fine tuning of wrist � default is 0.0f
                break;

            case TRANSFER:			// synchronized Transfer state activities
                if (Transfer_Time.seconds() >= .25) {    //
                    TargetLift = 480;
                } else {
                    TargetLift = 800;
                }
                if (LeftLift.getCurrentPosition() < 484 && Transfer_Delay.seconds() >= .6) {   // verified and delayed Outtake claw closing
                    OuttakeClawClosed = true;
                    OuttakeClaw.setPosition(0);
                    if (OuttakeClaw.getPosition() == 0 && Transfer_Delay.seconds() >= .85) {   // delayed Intake claw opening
                        IntakeClawClosed = false;
                        IntakeClaw.setPosition(0);
                    }
                }
                if (IntakeClaw.getPosition() == 0) {            //  
                    V4Bpos = .75;
                    if (Transfer_Delay.seconds() >= .75) {     //
                        state = State.IDLE;
                        Transfered = true;
                    }
                }
                break;

            case IDLE:			// Idle state activities
                if (gamepad2.triangle) {	// driver 2 raise lift to scoring height and enter Outtake state
                    TargetLift = 2520;
                    state = State.OUTTAKE;
                }
                if (gamepad2.left_bumper && !OuttakeClawClosed && ClawTime.seconds() >= .3) {    // driver 2 outtake claw toggle
                    ClawTime.reset();
                    OuttakeClaw.setPosition(0);
                    OuttakeClawClosed = true;
                } else if (gamepad2.left_bumper && OuttakeClawClosed && ClawTime.seconds() >= .3) {
                    ClawTime.reset();
                    OuttakeClaw.setPosition(.45);
                    OuttakeClawClosed = false;
                }
                if (gamepad2.right_bumper && !IntakeClawClosed && ClawTime.seconds() >= .3) {  // driver 2 intake claw toggle
                    ClawTime.reset();
                    IntakeClaw.setPosition(.5);
                    IntakeClawClosed = true;
                } else if (gamepad2.right_bumper && IntakeClawClosed && ClawTime.seconds() >= .3) {
                    ClawTime.reset();
                    IntakeClaw.setPosition(0);
                    IntakeClawClosed = false;
                }
                if (gamepad2.right_trigger >= .75 && TargetLift < MAX_TARGET_LIFT - 10) {          // driver 2 manual Lift adjustment
                    TargetLift = TargetLift + 10;
                } else if (gamepad2.left_trigger >= .75 && TargetLift > 10) {
                    TargetLift = TargetLift - 10;
                }
                if (gamepad2.cross) {			// driver 2 selection of INTAKE state
                    TargetLift = 750;
                    Transfered = false;
                    state = State.INTAKE;
                }
                break;

            case OUTTAKE:	        		// Outtake state activities
                if (gamepad2.cross) {		// driver 2 selection of INTAKE state
                    TargetLift = 750;
                    Transfered = false;
                    state = State.INTAKE;
                }
                if (gamepad2.left_bumper && !OuttakeClawClosed && ClawTime.seconds() >= .3) {  // driver 2 outtake claw toggle
                    ClawTime.reset();
                    OuttakeClaw.setPosition(0);
                    OuttakeClawClosed = true;
                } else if (gamepad2.left_bumper && OuttakeClawClosed && ClawTime.seconds() >= .3) {
                    ClawTime.reset();
                    OuttakeClaw.setPosition(.45);
                    OuttakeClawClosed = false;
                }
                if (gamepad2.right_trigger >= .75 && TargetLift < MAX_TARGET_LIFT - 10) {    // driver 2 manual lift adjustment
                    TargetLift = TargetLift + 10;
                } else if (gamepad2.left_trigger >= .75 && TargetLift > 10) {
                    TargetLift = TargetLift - 10;
                }
                break;

            case CLIMB:	            		// Climb state activities
                if (gamepad1.cross) {		// driver 1 execution of pegleg climb assistance
                    PegLeg.setPosition(1);
                } else {                    // retract pegleg upon release of button
                    PegLeg.setPosition(0);
                }
                if (gamepad2.right_trigger >= .75 && TargetLift < MAX_TARGET_LIFT - 10) {   // driver 2 manual lift adjustment
                    TargetLift = TargetLift + 10;
                } else if (gamepad2.left_trigger >= .75 && TargetLift > 50) {
                    TargetLift = TargetLift - 50;
                }
                if (gamepad2.dpad_down) {   // driver 2 manual lift down to climb
                    TargetLift = 600;
                    state = State.INTAKE;
                }
                break;
            } // **************************************End of State Machine

            if ( state != State.TRANSFER) {   // exclude CLIMB and manual lift controls during TRANSFER

                // ************************** current sensing reset selection during opmode if realignment is needed **************
                if (gamepad1.square && gamepad1.triangle) {    // enter sensing mode
                    OuttakeV4B.setPosition(.5);
                    IntakeV4B.setPosition(.8);
                    OuttakeWrist.setPosition(.25);
                    HorizontalCurrent = IntakeLeft.getCurrent(CurrentUnit.AMPS);
                    VerticalCurrent = LeftLift.getCurrent(CurrentUnit.AMPS);
                    ExtendPower = -.25;
                    IntakeLeft.setPower(ExtendPower);
                    IntakeRight.setPower(ExtendPower);
                    while (IntakeLeft.getCurrent(CurrentUnit.AMPS) < HorizontalCurrentThreshold) { }
                    IntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    IntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    LiftPower = -.25;
                    LeftLift.setPower(LiftPower);
                    RightLift.setPower(LiftPower);
                    while (RightLift.getCurrent(CurrentUnit.AMPS) < VerticalCurrentThreshold) { }
                    LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    IntakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    IntakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                // ***************************Manual control of Outtake Claw **********************************
                if (gamepad2.left_bumper && !OuttakeClawClosed && ClawTime.seconds() >= .3) {    // manual conrol of Outtake claw in all states
                    ClawTime.reset();
                    OuttakeClaw.setPosition(0);
                    OuttakeClawClosed = true;
                } else if (gamepad2.left_bumper && OuttakeClawClosed && ClawTime.seconds() >= .3) {
                    ClawTime.reset();
                    OuttakeClaw.setPosition(.45);
                    OuttakeClawClosed = false;
                }
                
                // ************************* driver 2 selection of CLIMB state ************
                if (gamepad2.dpad_up) {
                    TargetLift = MAX_TARGET_LIFT - 10;
                    PegLegTime.reset();
                    state = State.CLIMB;
                }
            }
           
            // **************************** switch Outtake wrist and claw based on Transfered boolean value ********************
            if (Transfered) {			// automatic positioning of outtake wrist/claw based on conditional transfer variable
                OuttakeV4B.setPosition(0);
                OuttakeWrist.setPosition(.7);
            } else {
                OuttakeV4B.setPosition(1);
                OuttakeWrist.setPosition(.03);
            }
            
            // *********************************
            if (IntakeLeft.getCurrentPosition() <= 50 && Transfer_Delay.seconds() >= 2) {   // automatic positioning of intake wrist/claw
                Flex = 0;
                V4Bpos = 1;
            } else if (Transfer_Delay.seconds() >= 2) {
                if (gamepad1.left_trigger > 0) {
                    V4Bpos = 0.5 * (1 - (gamepad1.left_trigger));     // Manual control for variable V4B height when in INTAKE state
                } else {
                    V4Bpos = .5;
                }
                Flex = .63;
            }

            // *********	calculate chassis motor movement math
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // *********** check for Turbo or Precision Mode
            if (gamepad1.left_bumper) {            // Left bumper is being pressed
                precision = 1;                                // set speed to full power - TURBO MODE
            } else if (gamepad1.right_bumper) { // Right bumper is being pressed
                 precision = 4;                               // set speed to 1/4 power - PRECISION MODE
            } else {
                precision = 2;                                // reset default speed to half - NORMAL MODE
            }

            // *********** calculate motor power
            denominator = denominator * precision;      // this adjusts motor speed to the desired precision level 
            frontLeftPower = (y + x + rx) / denominator;    // Does math to convert stick movement to motor powers
            backLeftPower = (y - x + rx) / denominator;     // Does math to convert stick movement to motor powers
            frontRightPower = (y - x - rx) / denominator;   // Does math to convert stick movement to motor powers
            backRightPower = (y + x - rx) / denominator;    // Does math to convert stick movement to motor powers

            // ************ issue Drive Wheels motor power
            FrontLeft.setPower(frontLeftPower);    // Sets the front left wheel's power
            BackLeft.setPower(backLeftPower);     // Sets the back left wheel's power
            FrontRight.setPower(frontRightPower);  // Sets the front right wheel's power
            BackRight.setPower(backRightPower);   // Sets the back right wheel's power

            // *********** set postitions for Intake wrist
            IntakeV4B.setPosition(V4Bpos);
            LeftServo = Math.max(0, Math.min(1, Flex - (.5 * Yaw)));
            RightServo = Math.max(0, Math.min(1, Flex + (.5 * Yaw)));
            LeftIntakeWrist.setPosition(LeftServo); //Sets servos to calculated positions
            RightIntakeWrist.setPosition(RightServo); //^

            // *********** PID calculations for Lift and Extension
            LiftController.setPID(Lp, Li, Ld);
            ExtendController.setPID(Ep, Ei, Ed);
            int LiftPos = LeftLift.getCurrentPosition();
            int ExtendPos = IntakeLeft.getCurrentPosition();
            double Lpid = LiftController.calculate(LiftPos, TargetLift);
            double Epid = ExtendController.calculate(ExtendPos, TargetExtend);
            double LiftFF = Math.cos(Math.toRadians(TargetLift / lift_ticks_in_degrees)) * Lf;

            // *********** get PID values 
            double LiftPower = Lpid + LiftFF;
            double ExtendPower = Epid;

            // ************ set Lift and Extension power
           LeftLift.setPower(LiftPower);
           RightLift.setPower(LiftPower);
           IntakeLeft.setPower(ExtendPower);
           IntakeRight.setPower(ExtendPower);

            // ********** send all telemetry
            telemetry.addData("Lift pos ", LiftPos);
            telemetry.addData("Extend pos ", ExtendPos);
            telemetry.addData("lift target ", TargetLift);
            telemetry.addData("extend target ", TargetExtend);
            telemetry.addData("state", state);
            telemetry.update();

        }
    }
}


