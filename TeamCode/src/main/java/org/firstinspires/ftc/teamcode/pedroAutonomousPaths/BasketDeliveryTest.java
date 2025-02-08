package org.firstinspires.ftc.teamcode.pedroAutonomousPaths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Basket Auto")
public class BasketDeliveryTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer, actionTimer;
    private int pathState;

    Servo RightIntakeWrist; // Chub Port 1 // Increments Using Dpad Side Buttons?
    Servo LeftIntakeWrist;   // Chub Port 2 // Ideally Stick Controlled
    Servo IntakeV4B;     // Chub Port 3 // Preset To Swing Out With X
    // Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------
    //  Servo RightHook = hardwareMap.servo.get("Right Hook");                // Chub Port 5 // Linked To LeftHook Activated At The Same Time
    Servo IntakeClaw, OuttakeV4B;
    Servo OuttakeClaw;            // Ehub Port 0 // If Slides Up O Activates This Claw
    Servo OuttakeWrist;          // Ehub Port 1 // Preset To Go To Delivery Position With Triangl
    // LIFT

    private DcMotorEx LeftLift, RightLift;

    private static PIDController LiftController;
    public static double Lp = 0.015, Li = 0, Ld = 0.0002;
    public static double Lf = 0.04;
    public static int TargetLift = 0;
    private static final double lift_ticks_in_degrees = 1.068055;

    public static double LiftPos;

    private static final int MAX_TARGET_LIFT = 2825;
    private double LiftPower = .9;


    // INTAKE

    private DcMotorEx IntakeLeft;
    private DcMotorEx IntakeRight;

    private static PIDController ExtendController;
    public static double Ep = .01, Ei = 0, Ed = .0004;
    public static double Ef = 0;
    public static int TargetExtend = 0;
    private final double extend_ticks_in_degrees = .403;

    public static double ExtendPos;

    public static double IntakeClawPos = 0;
    public static double OuttakeV4BPos = 0;
    public static double IntakeV4BPos = 0;
    public static double OuttakeClawPos = 0;
    private final int MAX_EXTENSION_LENGTH = 415;

    private boolean IntakeClawClosed = false;                   // claw holder variable
    private boolean OuttakeClawClosed = false;

    private ElapsedTime ClawTime = new ElapsedTime();


    private final Pose startPos = new Pose(8.6, 85, Math.toRadians(0));
    private final Pose deliverPos = new Pose(15, 119, Math.toRadians(-43));
    private final Pose sample1Pos = new Pose(32, 122, Math.toRadians(0));
    private final Pose sample2Pos = new Pose(32, 132, Math.toRadians(0));
    private final Pose sample3Pos = new Pose(32, 130, Math.toRadians(30));
    private final Pose parkPos = new Pose(67, 95, Math.toRadians(0));
    private final Pose parkControlPoint = new Pose(67, 120, Math.toRadians(0));

    private Path scorePreLoad, park;
    private PathChain grabSample1, grabSample2, grabSample3,  deliverSample1, deliverSample2, deliverSample3;


    public void buildPaths() {

        scorePreLoad = new Path(new BezierLine(new Point(startPos), (new Point(deliverPos))));
        scorePreLoad.setLinearHeadingInterpolation(startPos.getHeading(), deliverPos.getHeading());

        grabSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(deliverPos), new Point(sample1Pos)))
                .setLinearHeadingInterpolation(deliverPos.getHeading(), sample1Pos.getHeading())
                .build();

        deliverSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1Pos), new Point(deliverPos)))
                .setLinearHeadingInterpolation(sample1Pos.getHeading(), deliverPos.getHeading())
                .build();

        grabSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(deliverPos), new Point(sample2Pos)))
                .setLinearHeadingInterpolation(deliverPos.getHeading(), sample2Pos.getHeading())
                .build();

        deliverSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2Pos), new Point(deliverPos)))
                .setLinearHeadingInterpolation(sample2Pos.getHeading(), deliverPos.getHeading())
                .build();

        grabSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(deliverPos), new Point(sample3Pos)))
                .setLinearHeadingInterpolation(deliverPos.getHeading(), sample3Pos.getHeading())
                .build();

        deliverSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3Pos), new Point(deliverPos)))
                .setLinearHeadingInterpolation(sample3Pos.getHeading(), deliverPos.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(deliverPos), new Point(parkControlPoint), new Point(parkPos)));
        park.setLinearHeadingInterpolation(deliverPos.getHeading(), parkPos.getHeading());
    }

    public void autonomousUpdatePaths() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreLoad);
                actionTimer.resetTimer();
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()) {
                    // TargetLift = 2520;
                    // if(actionTimer.seconds() >= 3) {
                    //      OuttakeV4BPos = 0;
                    //      OuttakeWrist.setPosition(.7);
                    //}
                    // if(.getElapsedTimeSeconds() >= 4) {
                    //      OuttakeClawPos = 1;
                    //}
                    // TargetLift = 1;
                    // OuttakeV4BPos = 1;
                    // OuttakeWrist.setPosition(.03);
                    // Score Preload
                    actionTimer.resetTimer();
                    follower.followPath(grabSample1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    // TargetExtend = 415;
                    // IntakeV4BPos = .3;
                    // IntakeWrist.setPosition(.5);
                    // LeftIntakeWrist.setPosition(-.5);
                    //if(actionTimer.getElapsedTimeSeconds() >= 3) {
                    //      IntakeClawPos = 1;
                    //}
                    // TargetExtend = 1;
                    // IntakeV4BPos = 1;
                    // RightIntakeWrist.setPosition(0);
                    // LeftIntakeWrist.setPosition(0);
                    // Grab the 1st Sample
                    actionTimer.resetTimer();
                    follower.followPath(deliverSample1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    // TargetLift = 2520;
                    // if(actionTimer.getElapsedTimeSeconds() >= 3) {
                    //      OuttakeV4BPos = 0;
                    //      OuttakeWrist.setPosition(.7);
                    //}
                    // if(actionTimer.getElapsedTimeSeconds() >= 4) {
                    //      OuttakeClawPos = 1;
                    //}
                    // TargetLift = 1;
                    // OuttakeV4BPos = 1;
                    // OuttakeWrist.setPosition(.03);
                    // Deliver the 1st Sample

                    actionTimer.resetTimer();
                    follower.followPath(grabSample2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {
                    // TargetExtend = 415;
                    // IntakeV4BPos = .3;
                    // IntakeWrist.setPosition(.5);
                    // LeftIntakeWrist.setPosition(-.5);
                    // if(actionTimer.getElapsedTimeSeconds() >= 3) {
                    //      IntakeClawPos = 1;
                    //}
                    // TargetExtend = 1;
                    // IntakeV4BPos = 1;
                    // RightIntakeWrist.setPosition(0);
                    // LeftIntakeWrist.setPosition(0);
                    // Grab the 2nd Sample

                    actionTimer.resetTimer();
                    follower.followPath(deliverSample2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()) {
                    // TargetLift = 2520;
                    // if(actionTimer.getElapsedTimeSeconds() >= 3) {
                    //      OuttakeV4BPos = 0;
                    //      OuttakeWrist.setPosition(.7);
                    //}
                    // if(actionTimer.getElapsedTimeSeconds() >= 4) {
                    //      OuttakeClawPos = 1;
                    //}
                    // TargetLift = 1;
                    // OuttakeV4BPos = 1;
                    // OuttakeWrist.setPosition(.03);
                    // Deliver the 2nd Sample
                    actionTimer.resetTimer();
                    follower.followPath(grabSample3, true);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    // TargetExtend = 315;
                    // IntakeV4BPos = .3;
                    // IntakeWrist.setPosition(.7);
                    // LeftIntakeWrist.setPosition(-.7);
                    // if(actionTimer.getElapsedTimeSeconds() >= 3) {
                    //      IntakeClawPos = 1;
                    //}
                    // TargetExtend = 1;
                    // IntakeV4BPos = 1;
                    // RightIntakeWrist.setPosition(0);
                    // LeftIntakeWrist.setPosition(0);
                    // Grab the 3rd Sample

                    follower.followPath(deliverSample3, true);
                    setPathState(7);
                }
                break;

            case 7:
                if(!follower.isBusy()) {
                    // TargetLift = 2520;
                    // if(actionTimer.getElapsedTimeSeconds() >= 3) {
                    //      OuttakeV4BPos = 0;
                    //      OuttakeWrist.setPosition(.7);
                    //}
                    // if(actionTimer.getElapsedTimeSeconds() >= 4) {
                    //      OuttakeClawPos = 1;
                    //}
                    // TargetLift = 1;
                    // OuttakeV4BPos = 1;
                    // OuttakeWrist.setPosition(.03);
                    // Deliver the 3rd Sample

                    follower.followPath(park, true);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()) {

                    // Level 1 Ascent

                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousUpdatePaths();

        RunClaws("Intake", IntakeClawPos);
        RunClaws("Outtake", OuttakeClawPos);
        RunV4B("Intake", IntakeV4BPos);
        RunV4B("Outtake", OuttakeV4BPos);

        RunLift(TargetLift, MAX_TARGET_LIFT);
        RunExtension(TargetExtend, MAX_EXTENSION_LENGTH);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.addData("Intake Claw Pos", IntakeClawPos);
        telemetry.addData("Intake Claw", IntakeClaw.getPosition());

        telemetry.addData("Outtake Claw Pos", OuttakeClawPos);
        telemetry.addData("Outtake Claw", OuttakeClaw.getPosition());

        telemetry.addData("Intake V4B Pos", IntakeV4BPos);
        telemetry.addData("Intake V4B", IntakeV4B.getPosition());

        telemetry.addData("Outtake V4B Pos", OuttakeV4BPos);
        telemetry.addData("Outtake V4B", OuttakeV4B.getPosition());

        telemetry.addData("Target Lift", TargetLift);
        telemetry.addData("Lift Pos", LeftLift.getCurrentPosition());

        telemetry.addData("Target Extend", TargetExtend);
        telemetry.addData("Extend Pos", IntakeLeft.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPos);
        buildPaths();

        // LIFT INIT

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

        // INTAKE INIT

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

        IntakeV4B = hardwareMap.servo.get("Intake V4B");
        OuttakeV4B = hardwareMap.servo.get("Outtake V4B");

        IntakeV4B.setPosition(.8);
        OuttakeV4B.setPosition(1);

        IntakeClaw = hardwareMap.servo.get("Intake Claw");
        OuttakeClaw = hardwareMap.servo.get("Outtake Claw");

        IntakeClaw.setPosition(0);    // Closes Intake Claw

        OuttakeClaw.setPosition(0);   // Closes Outtake Claw
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}

    public void RunLift(int target, int MaxTargetLift) {
        if (target <= MaxTargetLift - 5) {
            TargetLift = target;
        }
        LiftPos = LeftLift.getCurrentPosition();
        LiftController.setPID(Lp, Li, Ld);
        double Lpid = LiftController.calculate(LiftPos, TargetLift);
        double LiftFF = Math.cos(Math.toRadians(TargetLift / lift_ticks_in_degrees)) * Lf;
        double LiftPower = Lpid + LiftFF;
        LeftLift.setPower(LiftPower);
        RightLift.setPower(LiftPower);
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

    public void RunV4B(String V4B, double target) {
        switch (V4B) {
            case "Intake":
                IntakeV4B.setPosition(target);
                break;

            case "Outtake":
                OuttakeV4B.setPosition(target);
                break;

            default:
                telemetry.addData("Invalid V4B", V4B);
                break;
        }
    }

    public void RunClaws(String claw, double target) {
        switch(claw) {
            case "Intake":
                ClawTime.reset();
                IntakeClaw.setPosition(target);
                IntakeClawClosed = !IntakeClawClosed;
                break;
            case "Outtake":
                ClawTime.reset();
                OuttakeClaw.setPosition(target);
                OuttakeClawClosed = !OuttakeClawClosed;
                break;

            default:
                telemetry.addData("Invalid Claw", claw);
                break;
        }
        telemetry.addData("The Claw", claw);
        telemetry.addData("Target Pos", target);
    }
}
