package org.firstinspires.ftc.teamcode.opModes.working.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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


@Autonomous(name = "Right Auto", preselectTeleOp = "TeleOpWithCurrentSensing", group = "Main")
public class RightPushSamples extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;

    private int pathState;

    private DcMotorEx LeftLift, RightLift;

    private static PIDController LiftController;
    public static double Lp = 0.015, Li = 0, Ld = 0.0002;
    public static double Lf = 0.04;
    public static int TargetLift = 0;
    private static final double lift_ticks_in_degrees = 1.068055;

    public static double LiftPos;

    private static final int MAX_TARGET_LIFT = 2825;
    private double LiftPower = .9;

    Servo IntakeV4B;     // Chub Port 3 // Preset To Swing Out With X
    // Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------
    //  Servo RightHook = hardwareMap.servo.get("Right Hook");                // Chub Port 5 // Linked To LeftHook Activated At The Same Time
    Servo IntakeClaw, OuttakeV4B;
    Servo OuttakeClaw;
    Servo OuttakeWrist;

    private final Pose startPose = new Pose(5.8, 54, Math.toRadians(0));
    private final Pose setupPose1 = new Pose(26, 35, Math.toRadians(0));
    private final Pose setupPose2 = new Pose(58, 35, Math.toRadians(0));
    private final Pose lineupPose1_1 = new Pose(58, 23, Math.toRadians(0));
    private final Pose pushSamplePose1 = new Pose(12, 23, Math.toRadians(0));
    private final Pose lineupPose2_1 = new Pose(58, 23, Math.toRadians(0));
    private final Pose lineupPose2_2 = new Pose(58, 13, Math.toRadians(0));
    private final Pose pushSamplePose2 = new Pose(12, 13, Math.toRadians(0));
    private final Pose lineupPose3_1 = new Pose(58, 13, Math.toRadians(0));
    private final Pose lineupPose3_2 = new Pose(58, 9, Math.toRadians(0));
    private final Pose pushSamplePose3 = new Pose(12, 9, Math.toRadians(0));

    private Path setupPath1, setupPath2, lineupPath1, pushSample1Path, lineupPath2_1, lineupPath2_2, pushSample2Path, lineupPath3_1, lineupPath3_2, pushSample3Path;

    public void buildPaths() {
       // driveFarRight = new Path(new BezierLine(new Point(startPose), new Point(farRightPose)));
       // driveFarRight.setLinearHeadingInterpolation(startPose.getHeading(), farRightPose.getHeading());
        setupPath1 = new Path(new BezierLine(new Point(startPose), new Point(setupPose1)));
        setupPath1.setLinearHeadingInterpolation(startPose.getHeading(), setupPose1.getHeading());

        setupPath2 = new Path(new BezierLine(new Point(setupPose1), new Point(setupPose2)));
        setupPath2.setLinearHeadingInterpolation(setupPose1.getHeading(), setupPose2.getHeading());

        lineupPath1 = new Path(new BezierLine(new Point(setupPose2), new Point(lineupPose1_1)));
        lineupPath1.setLinearHeadingInterpolation(setupPose2.getHeading(), lineupPose1_1.getHeading());

        pushSample1Path = new Path(new BezierLine(new Point(lineupPose1_1), new Point(pushSamplePose1)));
        pushSample1Path.setLinearHeadingInterpolation(lineupPose1_1.getHeading(), pushSamplePose1.getHeading());

        lineupPath2_1 = new Path(new BezierLine(new Point(pushSamplePose1), new Point(lineupPose2_1)));
        lineupPath2_1.setLinearHeadingInterpolation(pushSamplePose1.getHeading(), lineupPose2_1.getHeading());

        lineupPath2_2 = new Path(new BezierLine(new Point(lineupPose2_1), new Point(lineupPose2_2)));
        lineupPath2_2.setLinearHeadingInterpolation(lineupPose2_1.getHeading(), lineupPose2_2.getHeading());

        pushSample2Path = new Path(new BezierLine(new Point(lineupPose2_2), new Point(pushSamplePose2)));
        pushSample2Path.setLinearHeadingInterpolation(lineupPose2_2.getHeading(), pushSamplePose2.getHeading());

        lineupPath3_1 = new Path(new BezierLine(new Point(pushSamplePose2), new Point(lineupPose3_1)));
        lineupPath3_1.setLinearHeadingInterpolation(pushSamplePose2.getHeading(), lineupPose3_1.getHeading());

        lineupPath3_2 = new Path(new BezierLine(new Point(lineupPose3_1), new Point(lineupPose3_2)));
        lineupPath3_2.setLinearHeadingInterpolation(lineupPose3_1.getHeading(), lineupPose3_2.getHeading());

        pushSample3Path = new Path(new BezierLine(new Point(lineupPose3_2), new Point(pushSamplePose3)));
        pushSample3Path.setLinearHeadingInterpolation(lineupPose3_2.getHeading(), pushSamplePose3.getHeading());
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(setupPath1);
                actionTimer.resetTimer();
                setPathValue(1);
                break;

            case 1:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(setupPath2);
                    actionTimer.resetTimer();
                    setPathValue(2);
                }
                break;
            case 2:
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(lineupPath1);
                    actionTimer.resetTimer();
                    setPathValue(3);
                }
                break;
            case 3:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(pushSample1Path);
                    actionTimer.resetTimer();
                    setPathValue(4);
                }
            case 4:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(lineupPath2_1);
                    actionTimer.resetTimer();
                    setPathValue(5);
                }
            case 5:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(lineupPath2_2);
                    actionTimer.resetTimer();
                    setPathValue(6);
                }
            case 6:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(pushSample2Path);
                    actionTimer.resetTimer();
                    setPathValue(7);
                }
            case 7:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(lineupPath3_1);
                    actionTimer.resetTimer();
                    setPathValue(8);
                }
            case 8:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(lineupPath3_2);
                    actionTimer.resetTimer();
                    setPathValue(9);
                }
            case 9:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(pushSample3Path);
                    actionTimer.resetTimer();
                    setPathValue(-1);
                }
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
        if(opModeTimer.getElapsedTimeSeconds() > 25) {
            OuttakeV4B.setPosition(.5);
            IntakeV4B.setPosition(.8);
            OuttakeWrist.setPosition(.25);
        }

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
        opModeTimer.resetTimer();

        // LIFT INIT

        LeftLift = hardwareMap.get(DcMotorEx.class, "Left Lift");
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

        IntakeV4B = hardwareMap.servo.get("Intake V4B");
        OuttakeV4B = hardwareMap.servo.get("Outtake V4B");

        IntakeV4B.setPosition(.8);
        OuttakeV4B.setPosition(1);


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
}
