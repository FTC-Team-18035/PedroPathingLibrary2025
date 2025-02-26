package org.firstinspires.ftc.teamcode.opModes.needsTested;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


@Autonomous(name = "4 Sample Basket", group = "Main")
public class FourSampleBasketDelivery extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;

    private int pathState;

    private DcMotorEx LeftLift, RightLift, IntakeLeft, IntakeRight;

    private static PIDController LiftController;
    private static PIDController ExtendController;

    public static double Lp = 0.015, Li = 0, Ld = 0.0002;
    public static double Ep = .01, Ei = 0, Ed = .0004;

    public static double Lf = 0.04;
    public static double Ef = 0;

    public static int TargetLift = 0;
    public static int TargetExtend = 0;
    private static final double lift_ticks_in_degrees = 1.068055;
    private final double extend_ticks_in_degrees = .403;

    public static double LiftPos;
    public static double ExtendPos;

    private static final int MAX_TARGET_LIFT = 2825;
    private final int MAX_EXTENSION_LENGTH = 415;

    private double LiftPower;
    private double ExtendPower;

    Servo IntakeV4B;     // Chub Port 3 // Preset To Swing Out With X
    // Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------
    //  Servo RightHook = hardwareMap.servo.get("Right Hook");                // Chub Port 5 // Linked To LeftHook Activated At The Same Time
    Servo IntakeClaw, OuttakeV4B;
    Servo OuttakeClaw;
    Servo OuttakeWrist;

    Servo RightIntakeWrist;
    Servo LeftIntakeWrist;

    private final Pose startPose = new Pose(8.6, 88, Math.toRadians(0));
    private final Pose lineupPos = new Pose(19, 128, Math.toRadians(-45));
    private final Pose scorePose = new Pose(16, 130, Math.toRadians(-45));
    private final Pose sample1Pos = new Pose(32, 122, Math.toRadians(0));
    private final Pose sample2Pose = new Pose(32, 132, Math.toRadians(0));
    private final Pose sample3Pose = new Pose(32, 130, Math.toRadians(30));
    private final Pose parkPose = new Pose(61, 95, Math.toRadians(90));
    private final Pose parkControlPose = new Pose(61, 120, Math.toRadians(90));


    private Path score, lineupPreload, pickupSample1, lineupSample1, pickupSample2, lineupSample2, pickupSample3, lineupSample3, park;

    public void buildPaths() {
        lineupPreload = new Path(new BezierLine(new Point(startPose), new Point(lineupPos)));
        lineupPreload.setLinearHeadingInterpolation(startPose.getHeading(), lineupPos.getHeading());

        score = new Path(new BezierLine(new Point(lineupPos), new Point(scorePose)));
        score.setLinearHeadingInterpolation(lineupPos.getHeading(), scorePose.getHeading());

        pickupSample1 = new Path(new BezierLine(new Point(scorePose), new Point(sample1Pos)));
        pickupSample1.setLinearHeadingInterpolation(scorePose.getHeading(), sample1Pos.getHeading());

        lineupSample1 = new Path(new BezierLine(new Point(sample1Pos), new Point(lineupPos)));
        lineupSample1.setLinearHeadingInterpolation(sample1Pos.getHeading(), lineupPos.getHeading());

        pickupSample2 = new Path(new BezierLine(new Point(scorePose), new Point(sample2Pose)));
        pickupSample2.setLinearHeadingInterpolation(scorePose.getHeading(), sample2Pose.getHeading());

        lineupSample2 = new Path(new BezierLine(new Point(sample2Pose), new Point(lineupPos)));
        lineupSample2.setLinearHeadingInterpolation(sample2Pose.getHeading(), lineupPos.getHeading());

        pickupSample3 = new Path(new BezierLine(new Point(scorePose), new Point(sample3Pose)));
        pickupSample3.setLinearHeadingInterpolation(scorePose.getHeading(), sample3Pose.getHeading());

        lineupSample3 = new Path(new BezierLine(new Point(sample3Pose), new Point(lineupPos)));
        lineupSample3.setLinearHeadingInterpolation(sample3Pose.getHeading(), lineupPos.getHeading());

        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(lineupPreload);
                actionTimer.resetTimer();
                opModeTimer.resetTimer();
                setPathValue(1);
                break;
            case 1:
                if (actionTimer.getElapsedTimeSeconds() > 4) {
                    TargetLift = 2520;

                    OuttakeV4B.setPosition(0);
                    OuttakeWrist.setPosition(.7);
                    setPathValue(2);
                }
                break;

            case 2:
                    if (actionTimer.getElapsedTimeSeconds() > 3) {
                        follower.followPath(score);
                        setPathValue(3);
                    }
                break;
            case 3:
                if(actionTimer.getElapsedTimeSeconds() > 4.5) {
                    OuttakeClaw.setPosition(.45);
                    actionTimer.resetTimer();
                    setPathValue(4);
                }
                break;
            case 4:
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(pickupSample1);
                    actionTimer.resetTimer();
                    setPathValue(5);
                }
                break;
            case 5:
                if(actionTimer.getElapsedTimeSeconds() > 3) {
                    TargetLift = 800;
                    OuttakeV4B.setPosition(1);
                    OuttakeWrist.setPosition(.03);
                    TargetExtend = 415;
                    IntakeV4B.setPosition(.3);
                    if(actionTimer.getElapsedTimeSeconds() > 6) {
                        IntakeClaw.setPosition(1);
                    }
                    if(actionTimer.getElapsedTimeSeconds() > 8) {
                        IntakeV4B.setPosition(1);
                        TargetExtend = 1;
                    }
                    if(actionTimer.getElapsedTimeSeconds() > 10) {
                        TargetLift = 480;
                    }
                    if(actionTimer.getElapsedTimeSeconds() > 12) {
                        OuttakeClaw.setPosition(0);
                    }
                    if(actionTimer.getElapsedTimeSeconds() > 12.5) {
                        IntakeClaw.setPosition(0);
                        actionTimer.resetTimer();
                        setPathValue(6);
                    }
                }
                break;
            case 6:
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(lineupSample1);
                    actionTimer.resetTimer();
                    setPathValue(7);
                }
                break;
            case 7:
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    TargetLift = 2520;

                    OuttakeV4B.setPosition(0);
                    OuttakeWrist.setPosition(.7);
                }
                if(actionTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(score);
                    actionTimer.resetTimer();
                    setPathValue(8);
                }
                break;

            case 8:
                if(actionTimer.getElapsedTimeSeconds() > 4.5) {
                    OuttakeClaw.setPosition(.45);
                    actionTimer.resetTimer();
                    setPathValue(9);
                }
                break;

            case 9:
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(pickupSample2);
                    actionTimer.resetTimer();
                    setPathValue(10);
                }
                break;
            case 10:
                if(actionTimer.getElapsedTimeSeconds() > 3) {
                    TargetLift = 800;
                    OuttakeV4B.setPosition(1);
                    OuttakeWrist.setPosition(.03);
                    TargetExtend = 415;
                    IntakeV4B.setPosition(.3);
                    if(actionTimer.getElapsedTimeSeconds() > 6) {
                        IntakeClaw.setPosition(1);
                    }
                    if(actionTimer.getElapsedTimeSeconds() > 8) {
                        IntakeV4B.setPosition(1);
                        TargetExtend = 1;
                    }
                    if(actionTimer.getElapsedTimeSeconds() > 10) {
                        TargetLift = 480;
                    }
                    if(actionTimer.getElapsedTimeSeconds() > 12) {
                        OuttakeClaw.setPosition(0);
                    }
                    if(actionTimer.getElapsedTimeSeconds() > 12.5) {
                        IntakeClaw.setPosition(0);
                        actionTimer.resetTimer();
                        setPathValue(11);
                    }
                }
                break;
            case 11:
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(lineupSample2);
                    actionTimer.resetTimer();
                    setPathValue(12);
                }
                break;
            case 12:
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    TargetLift = 2520;

                    OuttakeV4B.setPosition(0);
                    OuttakeWrist.setPosition(.7);
                }
                if(actionTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(score);
                    actionTimer.resetTimer();
                    setPathValue(13);
                }
                break;

            case 13:
                if(actionTimer.getElapsedTimeSeconds() > 4.5) {
                    OuttakeClaw.setPosition(.45);
                    actionTimer.resetTimer();
                    setPathValue(14);
                }
                break;

            case 14:
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(pickupSample3);
                    actionTimer.resetTimer();
                    setPathValue(15);
                }
                break;
            case 15:
                if(actionTimer.getElapsedTimeSeconds() > 3) {
                    TargetLift = 800;
                    OuttakeV4B.setPosition(1);
                    OuttakeWrist.setPosition(.03);
                    TargetExtend = 415;
                    IntakeV4B.setPosition(.3);
                    if(actionTimer.getElapsedTimeSeconds() > 6) {
                        IntakeClaw.setPosition(1);
                    }
                    if(actionTimer.getElapsedTimeSeconds() > 8) {
                        IntakeV4B.setPosition(1);
                        TargetExtend = 1;
                    }
                    if(actionTimer.getElapsedTimeSeconds() > 10) {
                        TargetLift = 480;
                    }
                    if(actionTimer.getElapsedTimeSeconds() > 12) {
                        OuttakeClaw.setPosition(0);
                    }
                    if(actionTimer.getElapsedTimeSeconds() > 12.5) {
                        IntakeClaw.setPosition(0);
                        actionTimer.resetTimer();
                        setPathValue(16);
                    }
                }
                break;
            case 16:
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(lineupSample3);
                    actionTimer.resetTimer();
                    setPathValue(17);
                }
                break;
            case 17:
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    TargetLift = 2520;

                    OuttakeV4B.setPosition(0);
                    OuttakeWrist.setPosition(.7);
                }
                if(actionTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(score);
                    actionTimer.resetTimer();
                    setPathValue(17);
                }
                break;

            case 18:
                if(actionTimer.getElapsedTimeSeconds() > 4.5) {
                    OuttakeClaw.setPosition(.45);
                    actionTimer.resetTimer();
                    setPathValue(19);
                }
                break;
            case 19:
                if(actionTimer.getElapsedTimeSeconds() > 1) {
                    TargetLift = 400;
                    OuttakeV4B.setPosition(.25);
                    OuttakeWrist.setPosition(.7);
                    setPathValue(20);
                }
                break;
            case 20:
                if(actionTimer.getElapsedTimeSeconds() < 5) {
                    follower.followPath(park);
                }
                else{ requestOpModeStop(); }
        }
    }

    public void setPathValue(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        RunLift(TargetLift, MAX_TARGET_LIFT);
        RunIntake(TargetExtend, MAX_EXTENSION_LENGTH);


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Action Timer", actionTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        actionTimer = new Timer();

        // LIFT INIT

        LeftLift = hardwareMap.get(DcMotorEx.class, "Left Lift");
        RightLift = hardwareMap.get(DcMotorEx.class, "Right Lift");

        IntakeLeft = hardwareMap.get(DcMotorEx.class, "Intake Left");
        IntakeRight = hardwareMap.get(DcMotorEx.class, "Intake Right");

        LiftController = new PIDController(Lp, Li, Ld);
        ExtendController = new PIDController(Ep, Ei, Ed);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        IntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IntakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        IntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        IntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        IntakeV4B = hardwareMap.servo.get("Intake V4B");
        OuttakeV4B = hardwareMap.servo.get("Outtake V4B");

        IntakeV4B.setPosition(.8);
        OuttakeV4B.setPosition(1);

        RightIntakeWrist = hardwareMap.servo.get("Right Intake Wrist");
        LeftIntakeWrist = hardwareMap.servo.get("Left Intake Wrist");

        IntakeClaw = hardwareMap.servo.get("Intake Claw");
        OuttakeClaw = hardwareMap.servo.get("Outtake Claw");

        OuttakeWrist = hardwareMap.servo.get("Outtake Wrist");

        OuttakeWrist.setPosition(.03);

        IntakeClaw.setPosition(0);    // Closes Intake Claw

        OuttakeClaw.setPosition(0);   // Closes Outtake Claw

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }



    public void RunLift(int target, int MaxTargetLift) {
        TargetLift = target;
        LiftPos = LeftLift.getCurrentPosition();
        LiftController.setPID(Lp, Li, Ld);
        double Lpid = LiftController.calculate(LiftPos, TargetLift);
        double LiftFF = Math.cos(Math.toRadians(TargetLift / lift_ticks_in_degrees)) * Lf;
        double LiftPower = Lpid + LiftFF;
        LeftLift.setPower(LiftPower);
        RightLift.setPower(LiftPower);
    }

    public void RunIntake(int target, int MaxTargetExtend) {
        TargetExtend = target;
        ExtendPos = IntakeLeft.getCurrentPosition();
        ExtendController.setPID(Ep, Ei, Ed);
        double Epid = ExtendController.calculate(ExtendPos, TargetExtend);
        double ExtendFF = Math.cos(Math.toRadians(TargetExtend / extend_ticks_in_degrees)) * Ef;
        double ExtendPower = Epid + ExtendFF;
        IntakeLeft.setPower(ExtendPower);
        IntakeRight.setPower(ExtendPower);
    }
}
