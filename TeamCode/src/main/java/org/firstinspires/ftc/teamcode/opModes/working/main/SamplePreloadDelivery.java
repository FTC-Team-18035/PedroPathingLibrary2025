package org.firstinspires.ftc.teamcode.opModes.working.main;

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


@Autonomous(name = "Left Auto", preselectTeleOp = "TeleOpWithCurrentSensing", group = "Main")
public class SamplePreloadDelivery extends OpMode {

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

    private final Pose startPose = new Pose(11.5, 119, Math.toRadians(-45));

    private final Pose scorePose = new Pose(19, 128, Math.toRadians(-45));
    private final Pose endPose = new Pose(16, 130, Math.toRadians(-45)); // 11 to the left
    private final Pose pullForwardsPose = new Pose(23, 11, Math.toRadians(-45));
    private final Pose parkPose1 = new Pose(60, 97, Math.toRadians(90));
    private final Pose parkControlPose = new Pose(29, 97, Math.toRadians(90));
    private final Pose parkControlPose2 = new Pose(58, 140, Math.toRadians(90));
    private Path scorePath, endPath, pullForwardsPath, parkPath1, parkPath2;

    public void buildPaths() {
       // driveFarRight = new Path(new BezierLine(new Point(startPose), new Point(farRightPose)));
       // driveFarRight.setLinearHeadingInterpolation(startPose.getHeading(), farRightPose.getHeading());
        endPath = new Path(new BezierLine(new Point(scorePose), new Point(endPose)));
        endPath.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading());

        scorePath = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePath.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pullForwardsPath = new Path(new BezierLine(new Point(endPose), new Point(scorePose)));
        pullForwardsPath.setLinearHeadingInterpolation(endPose.getHeading(), scorePose.getHeading());

        parkPath1 = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkControlPose2), new Point(parkPose1)));
        parkPath1.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose1.getHeading());

    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(scorePath);
                actionTimer.resetTimer();
                opModeTimer.resetTimer();
                setPathValue(1);
                break;

            case 1:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    TargetLift = 2520;

                    OuttakeV4B.setPosition(0);
                    OuttakeWrist.setPosition(.7);
                    setPathValue(2);
                }
                break;

            case 2:
                    if (actionTimer.getElapsedTimeSeconds() > 3) {
                        follower.followPath(endPath);
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
                    follower.followPath(pullForwardsPath);
                    actionTimer.resetTimer();
                    setPathValue(5);
                }
                break;
            case 5:
                if(actionTimer.getElapsedTimeSeconds() > 3) {
                    TargetLift = 400;
                    OuttakeV4B.setPosition(.25);
                    OuttakeWrist.setPosition(.7);
                    setPathValue(6);
                }
                break;
            case 6:
                    follower.followPath(parkPath1);
                    setPathValue(-1);
                    break;
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

        if(opModeTimer.getElapsedTimeSeconds() > 15) {
            requestOpModeStop();
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
