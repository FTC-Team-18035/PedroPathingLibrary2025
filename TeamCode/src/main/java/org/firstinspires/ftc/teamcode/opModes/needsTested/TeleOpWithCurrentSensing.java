package org.firstinspires.ftc.teamcode.opModes.needsTested;

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

@TeleOp
public class TeleOpWithCurrentSensing extends LinearOpMode {

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

    private DcMotorEx LeftLift;
    private DcMotorEx RightLift;
    private DcMotorEx IntakeLeft;
    private DcMotorEx IntakeRight;

    private double frontLeftPower = 0;     // declare motor power variable
    private double backLeftPower = 0;      // declare motor power variable
    private double frontRightPower = 0;    // declare motor power variable
    private double backRightPower = 0;     // declare motor power variable
    private double denominator = 1;        // declare motor power calculation variable

    private final int MAX_TARGET_LIFT = 2655;       // The max Lift Height
    private final int MAX_EXTENSION_LENGTH = 415;   // The max Extension Length

    private static double HorizontalCurrentThreshold = 2;
    private static double VerticalCurrentThreshold = 2;
    private double HorizontalCurrent;
    private double VerticalCurrent;

    private double LiftPower;
    private double ExtendPower;

    private int precision = 2;                                  // chassis motor power reduction factor 1
    private boolean IntakeClawClosed = false;                   // claw holder variable
    private boolean OuttakeClawClosed = false;                  // claw holder variable
    private boolean LiftDown = true;                            // Is the Lift all the way down

    private ElapsedTime Transfer_Time = new ElapsedTime();      // Timer to keep track of the transfer time
    private ElapsedTime Transfer_Delay = new ElapsedTime();
    private ElapsedTime ClawTime = new ElapsedTime();           // Timer to keep track since the claw was used last

    private ElapsedTime PegLegTime = new ElapsedTime();         // Timer to keep track of how long the peg leg is out
    private ElapsedTime ClawDelay = new ElapsedTime();

    public double LeftServo;
    public double RightServo;
    private double V4Bpos = 1;
    private double Flex = 0;
    private double Yaw = 0;
    private boolean Transfered = false;
    private boolean Started = false;

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
        DcMotor FrontRight = hardwareMap.dcMotor.get("Front Right");   // Chub Port 0 // Gamepad 1
        DcMotor BackRight = hardwareMap.dcMotor.get("Back Right");     // Chub Port 1 // Left Stick For Moving
        DcMotor FrontLeft = hardwareMap.dcMotor.get("Front Left");     // Chub Port 2 // Right Stick For Turning
        DcMotor BackLeft = hardwareMap.dcMotor.get("Back Left");       // Chub Port 3

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

        Servo PegLeg = hardwareMap.servo.get("Peg Leg");            // Ehub Port 5 // Gamepad 2 Dpad down

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


        waitForStart();

        IntakeClaw.setPosition(0);    // Closes Intake Claw

        OuttakeClaw.setPosition(.45);   // Closes Outtake Claw

        LeftServo = Math.max(0, Math.min(1, Flex - (.5 * Yaw)));
        RightServo = Math.max(0, Math.min(1, Flex + (.5 * Yaw)));

        LeftIntakeWrist.setPosition(LeftServo);    // Sets the intake wrist to the starting position // Left is 0 Right is 1
        RightIntakeWrist.setPosition(RightServo);   // Sets the intake wrist to the starting position // Left is 0 Right is 1

        OuttakeV4B.setPosition(.5);
        IntakeV4B.setPosition(.8);
        OuttakeWrist.setPosition(.25);
        PegLeg.setPosition(0);

        HorizontalCurrent = IntakeLeft.getCurrent(CurrentUnit.AMPS);
        VerticalCurrent = LeftLift.getCurrent(CurrentUnit.AMPS);
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

        LeftLift.setPower(LiftPower);
        RightLift.setPower(LiftPower);
        IntakeLeft.setPower(ExtendPower);
        IntakeRight.setPower(ExtendPower);

        IntakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        while (opModeIsActive()) {
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
            switch (state) {
                case INTAKE:
                    if (gamepad2.x) {
                        TargetExtend = MAX_EXTENSION_LENGTH;
                    } else if (gamepad2.b) {
                        TargetExtend = 1;
                    }
                    if (IntakeLeft.getCurrentPosition() <= 7 && IntakeClaw.getPosition() == .5) {
                        Transfer_Time.reset();
                        Transfer_Delay.reset();
                        Transfered = false;
                        state = State.TRANSFER;
                    }
                    if (gamepad2.right_bumper && !IntakeClawClosed && ClawTime.seconds() >= .3) {
                        ClawTime.reset();
                        IntakeClaw.setPosition(.5);
                        IntakeClawClosed = true;
                    } else if (gamepad2.right_bumper && IntakeClawClosed && ClawTime.seconds() >= 1) {
                        ClawTime.reset();
                        IntakeClaw.setPosition(0);
                        IntakeClawClosed = false;
                    }
                    if (gamepad2.dpad_left && TargetExtend < MAX_EXTENSION_LENGTH) {
                        TargetExtend = TargetExtend + 10;
                    } else if (gamepad2.dpad_right && TargetExtend > 10) {
                        TargetExtend = TargetExtend - 10;
                    }

                    if (gamepad1.touchpad_finger_1) {
                        Yaw = gamepad1.touchpad_finger_1_x;
                    } else {
                        Yaw = 0;
                    }
                    break;
                case TRANSFER:
                    if (Transfer_Time.seconds() >= .25) {
                        TargetLift = 480;
                    } else {
                        TargetLift = 800;
                    }
                    if (LeftLift.getCurrentPosition() < 484 && Transfer_Delay.seconds() >= .6) {
                        OuttakeClawClosed = true;
                        OuttakeClaw.setPosition(0);
                        if (OuttakeClaw.getPosition() == 0 && Transfer_Delay.seconds() >= .85) {
                            IntakeClawClosed = false;
                            IntakeClaw.setPosition(0);
                        }
                    }
                    if (IntakeClaw.getPosition() == 0) {
                        V4Bpos = .75;
                        if (Transfer_Delay.seconds() >= .75) {
                            state = State.IDLE;
                            Transfered = true;
                        }
                    }
                    break;

                case IDLE:
                    if (gamepad2.y) {
                        TargetLift = 2520;
                        state = State.OUTTAKE;
                    }
                    if (gamepad2.left_bumper && !OuttakeClawClosed && ClawTime.seconds() >= .3) {
                        ClawTime.reset();
                        OuttakeClaw.setPosition(0);
                        OuttakeClawClosed = true;
                    } else if (gamepad2.left_bumper && OuttakeClawClosed && ClawTime.seconds() >= .3) {
                        ClawTime.reset();
                        OuttakeClaw.setPosition(.45);
                        OuttakeClawClosed = false;
                    }
                    if (gamepad2.right_bumper && !IntakeClawClosed && ClawTime.seconds() >= .3) {
                        ClawTime.reset();
                        IntakeClaw.setPosition(.5);
                        IntakeClawClosed = true;
                    } else if (gamepad2.right_bumper && IntakeClawClosed && ClawTime.seconds() >= .3) {
                        ClawTime.reset();
                        IntakeClaw.setPosition(0);
                        IntakeClawClosed = false;
                    }
                    if (gamepad2.right_trigger >= .75 && TargetLift < MAX_TARGET_LIFT - 10) {
                        TargetLift = TargetLift + 10;
                    } else if (gamepad2.left_trigger >= .75 && TargetLift > 10) {
                        TargetLift = TargetLift - 10;
                    }
                    if (gamepad2.a) {
                        TargetLift = 750;
                        Transfered = false;
                        state = State.INTAKE;
                    }
                    break;
                case OUTTAKE:
                    if (gamepad2.a) {
                        TargetLift = 750;
                        Transfered = false;
                        state = State.INTAKE;
                    }
                    if (gamepad2.left_bumper && !OuttakeClawClosed && ClawTime.seconds() >= .3) {
                        ClawTime.reset();
                        OuttakeClaw.setPosition(0);
                        OuttakeClawClosed = true;
                    } else if (gamepad2.left_bumper && OuttakeClawClosed && ClawTime.seconds() >= .3) {
                        ClawTime.reset();
                        OuttakeClaw.setPosition(.45);
                        OuttakeClawClosed = false;
                    }
                    if (gamepad2.right_trigger >= .75 && TargetLift < MAX_TARGET_LIFT - 10) {
                        TargetLift = TargetLift + 10;
                    } else if (gamepad2.left_trigger >= .75 && TargetLift > 10) {
                        TargetLift = TargetLift - 10;
                    }
                    break;

                case CLIMB:

                    if (gamepad1.a) {
                        PegLeg.setPosition(1);
                    } else {
                        PegLeg.setPosition(0);
                    }

                    if (gamepad2.right_trigger >= .75 && TargetLift < MAX_TARGET_LIFT - 10) {
                        TargetLift = TargetLift + 10;
                    } else if (gamepad2.left_trigger >= .75 && TargetLift > 50) {
                        TargetLift = TargetLift - 50;
                    }
                    if (gamepad2.dpad_down) {
                        TargetLift = 600;
                        state = State.INTAKE;
                    }
                    break;
            }
            if (gamepad1.dpad_up && gamepad2.dpad_up) {
                TargetLift = MAX_TARGET_LIFT - 10;
                PegLegTime.reset();
                state = State.CLIMB;
            }
            if (Transfered) {
                OuttakeV4B.setPosition(0);
                OuttakeWrist.setPosition(.7);
            } else {
                OuttakeV4B.setPosition(1);
                OuttakeWrist.setPosition(.03);
            }

            if (IntakeLeft.getCurrentPosition() <= 50 && Transfer_Delay.seconds() >= 2) {
                Flex = 0;
                V4Bpos = 1;

            } else if (Transfer_Delay.seconds() >= 2) {
                if (gamepad1.left_trigger > 0) {
                    V4Bpos = 0.5 * (1 - (gamepad1.left_trigger)); //Control for variable virtual four bar height when in INTAKE state
                } else {
                    V4Bpos = .5;
                }
                Flex = .63;
            }


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


            LiftController.setPID(Lp, Li, Ld);
            ExtendController.setPID(Ep, Ei, Ed);
            int LiftPos = LeftLift.getCurrentPosition();
            int ExtendPos = IntakeLeft.getCurrentPosition();
            double Lpid = LiftController.calculate(LiftPos, TargetLift);
            double Epid = ExtendController.calculate(ExtendPos, TargetExtend);
            double LiftFF = Math.cos(Math.toRadians(TargetLift / lift_ticks_in_degrees)) * Lf;

            double LiftPower = Lpid + LiftFF;
            double ExtendPower = Epid;

            LeftLift.setPower(LiftPower);
            RightLift.setPower(LiftPower);
            IntakeLeft.setPower(ExtendPower);
            IntakeRight.setPower(ExtendPower);

            telemetry.addData("Lift pos ", LiftPos);
            telemetry.addData("Extend pos ", ExtendPos);
            telemetry.addData("lift target ", TargetLift);
            telemetry.addData("extend target ", TargetExtend);
            telemetry.addData("state", state);
            telemetry.update();

        }
    }
}
