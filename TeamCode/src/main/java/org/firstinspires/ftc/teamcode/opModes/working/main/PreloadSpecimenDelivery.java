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


@Autonomous(name = "Preload Specimen", group = "Main")
public class PreloadSpecimenDelivery extends OpMode {

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

    private double V4Bpos = .8;
    private double Flex = 0;
    private double Yaw = 0;

    private double LiftPower;
    private double ExtendPower;

    public double LeftServo;
    public double RightServo;

    Servo IntakeV4B;     // Chub Port 3 // Preset To Swing Out With X
    // Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------
    //  Servo RightHook = hardwareMap.servo.get("Right Hook");                // Chub Port 5 // Linked To LeftHook Activated At The Same Time
    Servo IntakeClaw, OuttakeV4B;
    Servo OuttakeClaw;
    Servo OuttakeWrist;

    Servo RightIntakeWrist;
    Servo LeftIntakeWrist;

    private final Pose startPose = new Pose(8, 56, Math.toRadians(-90));
    private final Pose awayWall = new Pose(26, 72, Math.toRadians(180));
    private final Pose scorePos = new Pose(33, 70, Math.toRadians(180));
    private final Pose parkPose = new Pose(10, 20, Math.toRadians(-90));



    private Path moveAway, score, park;

    public void buildPaths() {
        moveAway = new Path(new BezierLine(new Point(startPose), new Point(awayWall)));
        moveAway.setLinearHeadingInterpolation(startPose.getHeading(), awayWall.getHeading());

        score = new Path(new BezierLine(new Point(awayWall), new Point(scorePos)));
        score.setLinearHeadingInterpolation(awayWall.getHeading(), scorePos.getHeading());

        park = new Path(new BezierLine(new Point(scorePos), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePos.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(moveAway);
                TargetLift = 580;
                OuttakeV4B.setPosition(0);
                OuttakeWrist.setPosition(.7);
                actionTimer.resetTimer();
                opModeTimer.resetTimer();
                setPathValue(1);
                break;
            case 1:
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    TargetLift = 470;
                    setPathValue(2);
                }
                break;
            case 2:
                if(actionTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(score);
                    actionTimer.resetTimer();
                }

                if(actionTimer.getElapsedTimeSeconds() > 4) {
                    OuttakeClaw.setPosition(.45);
                    actionTimer.resetTimer();
                    setPathValue(3);
                }
                break;
            case 3:
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(park);
                    actionTimer.resetTimer();
                    setPathValue(-1);
                }
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
        RunIntake(TargetExtend, MAX_EXTENSION_LENGTH);
        Use4BarParts(Flex, Yaw, V4Bpos);

        if(opModeTimer.getElapsedTimeSeconds() >= 23) {
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

        OuttakeWrist.setPosition(.25);

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

    public void Use4BarParts(double Flex, double Yaw, double V4Bpos) {

        IntakeV4B.setPosition(V4Bpos);

        LeftServo = Math.max(0, Math.min(1, Flex - (.5 * Yaw)));
        RightServo = Math.max(0, Math.min(1, Flex + (.5 * Yaw)));
        LeftIntakeWrist.setPosition(LeftServo); //Sets servos to calculated positions
        RightIntakeWrist.setPosition(RightServo);
    }
}
