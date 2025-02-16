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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Disabled
@Autonomous(name = "Specimen Auto")
public class SpecimenDeliveryTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer, actionTimer;
    private int pathState;

    Servo RightIntakeWrist; // Chub Port 1 // Increments Using Dpad Side Buttons?
    Servo LeftIntakeWrist;   // Chub Port 2 // Ideally Stick Controlled
    Servo IntakeV4B;     // Chub Port 3 // Preset To Swing Out With X
    // Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------
    //  Servo RightHook = hardwareMap.servo.get("Right Hook");                // Chub Port 5 // Linked To LeftHook Activated At The Same Time

    Servo IntakeClaw;

    Servo OuttakeClaw;            // Ehub Port 0 // If Slides Up O Activates This Claw
    Servo OuttakeWrist;          // Ehub Port 1 // Preset To Go To Delivery Position With Triangle
    Servo OuttakeV4B;

    // LIFT

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


    // INTAKE

    private DcMotorEx IntakeLeft;
    private DcMotorEx IntakeRight;

    private static PIDController ExtendController;
    public static double Ep = .01, Ei = 0, Ed = .0004;
    public static double Ef = 0;
    public static int TargetExtend = 0;
    private final double extend_ticks_in_degrees = .403;

    public static double ExtendPos;

    private final int MAX_EXTENSION_LENGTH = 415;

    private boolean IntakeClawClosed = false;                   // claw holder variable
    private boolean OuttakeClawClosed = false;

    private ElapsedTime ClawTime = new ElapsedTime();

    // Change the starting orientation to 0 from 180
    private final Pose startPos = new Pose(8.6, 59, Math.toRadians(0));

    private final Pose hookPos = new Pose(42, 66,Math.toRadians(0));
    private final Pose hookControlPoint = new Pose(0, 0, Math.toRadians(0));
    private final Pose lineupForPush1Pos = new Pose(59, 17, Math.toRadians(90));
    private final Pose lineupForPush1ControlPoint = new Pose(2, 4, Math.toRadians(90));
    private final Pose lineupForPush1ControlPoint2 = new Pose(92, 42, Math.toRadians(90));
    private final Pose sample1Pos = new Pose(11, 17, Math.toRadians(90));
    private final Pose lineupForPush2Pos = new Pose(58, 7, Math.toRadians(90));
    private final Pose lineupForPush2ControlPoint = new Pose(87, 20, Math.toRadians(90));
    private final Pose sample2Pos = new Pose(11, 7, Math.toRadians(90));
    private final Pose lineupForPush3Pos = new Pose(58, .5, Math.toRadians(90));
    private final Pose lineupForPush3ControlPoint = new Pose(87, 9, Math.toRadians(90));
    private final Pose sample3Pos = new Pose(11, .5,Math.toRadians(90));
    private final Pose pickupPos = new Pose(11, 40, Math.toRadians(270));
    private final Pose pickupControlPoint = new Pose(42, 21, Math.toRadians(270));
    private final Pose pickupControlPoint2 = new Pose(12, 73, Math.toRadians(270));
    private final Pose parkPos = new Pose(11, 12, Math.toRadians(0));
    private final Pose parkControlPoint = new Pose(19, 73, Math.toRadians(0));

    private Path hookPreload, park;
    private PathChain lineup1, lineup2, lineup3, pushSample1, pushSample2, pushSample3, grabSpecimen1, grabSpecimen2, grabSpecimen3, hookSpecimen1, hookSpecimen2, hookSpecimen3;
    public void buildPaths() {

        hookPreload = new Path(new BezierLine(new Point(startPos), (new Point(hookPos))));
        hookPreload.setLinearHeadingInterpolation(startPos.getHeading(), hookPos.getHeading());

        lineup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hookPos) , new Point(lineupForPush1ControlPoint), new Point(lineupForPush1ControlPoint2), new Point(lineupForPush1Pos)))
                .setConstantHeadingInterpolation(lineupForPush1Pos.getHeading())
                .build();

        pushSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(lineupForPush1Pos), new Point(sample1Pos)))
                .setConstantHeadingInterpolation(sample1Pos.getHeading())
                .build();

        lineup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sample1Pos), new Point(lineupForPush2ControlPoint), new Point(lineupForPush2Pos)))
                .setConstantHeadingInterpolation(sample2Pos.getHeading())
                .build();

        pushSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(lineupForPush2Pos), new Point(sample2Pos)))
                .setConstantHeadingInterpolation(sample2Pos.getHeading())
                .build();

        lineup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sample2Pos), new Point(lineupForPush3ControlPoint), new Point(lineupForPush3Pos)))
                .setConstantHeadingInterpolation(sample3Pos.getHeading())
                .build();

        pushSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(lineupForPush3Pos), new Point(sample3Pos)))
                .setConstantHeadingInterpolation(sample3Pos.getHeading())
                .build();

        grabSpecimen1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sample3Pos), new Point(pickupControlPoint2), new Point(pickupPos)))
                .setLinearHeadingInterpolation(sample3Pos.getHeading(), pickupPos.getHeading())
                .build();

        hookSpecimen1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupPos), new Point(hookControlPoint), new Point(hookPos)))
                .setLinearHeadingInterpolation(pickupPos.getHeading(), hookPos.getHeading())
                .build();

        grabSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hookPos), new Point(pickupControlPoint), new Point(pickupPos)))
                .setLinearHeadingInterpolation(hookPos.getHeading(), pickupPos.getHeading())
                .build();

        hookSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupPos), new Point(hookControlPoint), new Point(hookPos)))
                .setLinearHeadingInterpolation(pickupPos.getHeading(), hookPos.getHeading())
                .build();

        grabSpecimen3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hookPos), new Point(pickupControlPoint), new Point(pickupPos)))
                .setLinearHeadingInterpolation(hookPos.getHeading(), pickupPos.getHeading())
                .build();

        hookSpecimen3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupPos), new Point(hookControlPoint), new Point(hookPos)))
                .setLinearHeadingInterpolation(pickupPos.getHeading(), hookPos.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(hookPos), new Point(parkControlPoint), new Point(parkPos)));
        park.setLinearHeadingInterpolation(hookPos.getHeading(), parkPos.getHeading());

    }

    public void autonomousUpdatePaths() {
        switch (pathState) {
            case 0:
                follower.followPath(hookPreload);
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()) {

                    // Score Preload

                    follower.followPath(lineup1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {

                    follower.followPath(pushSample1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {

                    follower.followPath(lineup2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {

                    follower.followPath(pushSample2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()) {

                    follower.followPath(lineup3, true);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()) {

                    follower.followPath(pushSample3, true);
                    setPathState(7);
                }
                break;

            case 7:
                if(!follower.isBusy()) {

                    follower.followPath(grabSpecimen1, true);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()) {

                    // Grab the 1st Specimen

                    follower.followPath(hookSpecimen1, true);
                    setPathState(9);
                }
                break;

            case 9:
                if(!follower.isBusy()) {

                    // Hook the 1st Specimen

                    follower.followPath(grabSpecimen2, true);
                    setPathState(10);
                }
                break;

            case 10:
                if(!follower.isBusy()) {

                    // Grab the 2nd Specimen

                    follower.followPath(hookSpecimen2, true);
                    setPathState(11);
                }
                break;

            case 11:
                if(!follower.isBusy()) {

                    // Hook the 2nd Specimen

                    follower.followPath(grabSpecimen3, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {

                    // Grab the 3rd Specimen

                    follower.followPath(hookSpecimen3, true);
                    setPathState(13);
                }
                break;

            case 13:
                if(!follower.isBusy()) {

                    // Hook the 3rd Specimen

                    follower.followPath(park);
                    setPathState(14);
                }
                break;

            case 14:
                if(!follower.isBusy()) {
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

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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

        IntakeClaw.setPosition(0);

        OuttakeClaw.setPosition(0);
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

