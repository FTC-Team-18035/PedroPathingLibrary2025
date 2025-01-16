package org.firstinspires.ftc.teamcode.pedroAutonomousPaths;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


@Autonomous(name = "Left Parking Auto", preselectTeleOp = "TeleOpWithPID")
public class ParkingPathLeft extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;
    private int pathState;

    public double LeftServo;
    public double RightServo;
    private double V4Bpos = 1;
    private double Flex = 0;
    private double Yaw = 0;

    Servo RightIntakeWrist; // Chub Port 1 // Increments Using Dpad Side Buttons?
    Servo LeftIntakeWrist;   // Chub Port 2 // Ideally Stick Controlled
    Servo IntakeV4B;     // Chub Port 3 // Preset To Swing Out With X
    // Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------
    //  Servo RightHook = hardwareMap.servo.get("Right Hook");                // Chub Port 5 // Linked To LeftHook Activated At The Same Time

    Servo IntakeClaw;

    Servo OuttakeClaw;            // Ehub Port 0 // If Slides Up O Activates This Claw
    Servo OuttakeWrist;          // Ehub Port 1 // Preset To Go To Delivery Position With Triangle
    Servo OuttakeV4B;

    // Old startPose private final Pose startPose = new Pose(10, 81, Math.toRadians(0));

    // Old moveLeftPose private final Pose moveLeftPose = new Pose(13, 111, Math.toRadians(0));
    // Old moveForwardPose private final Pose moveForwardPose = new Pose(62, 111, Math.toRadians(0));
    // Old parkingPose private final Pose parkingPose = new Pose(62, 87, Math.toRadians(0));

    private final Pose startPose = new Pose(8.6, 84, Math.toRadians(0));
    private final Pose moveLeftPose = new Pose(13, 111, Math.toRadians(0));
    private final Pose moveForwardPose = new Pose(66, 111, Math.toRadians(0));
    private final Pose parkingPose = new Pose(66, 91, Math.toRadians(0));


    private Path moveLeftPath, moveForwardPath, parkPath;

    public void buildPaths() {
        moveLeftPath = new Path(new BezierLine(new Point(startPose), new Point(moveLeftPose)));
        moveLeftPath.setLinearHeadingInterpolation(startPose.getHeading(), moveLeftPose.getHeading());

        moveForwardPath = new Path(new BezierLine(new Point(moveLeftPose), new Point(moveForwardPose)));
        moveForwardPath.setLinearHeadingInterpolation(moveLeftPose.getHeading(), moveForwardPose.getHeading());

        parkPath = new Path(new BezierLine(new Point(moveForwardPose), new Point(parkingPose)));
        parkPath.setLinearHeadingInterpolation(moveForwardPose.getHeading(), parkingPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(moveLeftPath);
                setPathValue(1);
                break;

            case 1:
                if(follower.getPose().getX() > (moveLeftPose.getX() - 1) && follower.getPose().getY() > (moveLeftPose.getY() - 1)) {
                    follower.followPath(moveForwardPath);
                    setPathValue(2);
                }
                break;

            case 2:
                if(follower.getPose().getX() > (moveForwardPose.getX() - 1) && follower.getPose().getY() > (moveForwardPose.getY() - 1)) {
                    follower.followPath(parkPath);
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

        if(opModeTimer.getElapsedTimeSeconds() >= 8) {
            OuttakeV4B.setPosition(.5);
            IntakeV4B.setPosition(.8);
            OuttakeWrist.setPosition(.25);
            if(opModeTimer.getElapsedTimeSeconds() >= 10) {
                requestOpModeStop();
            }
        }

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        RightIntakeWrist = hardwareMap.servo.get("Right Intake Wrist"); // Chub Port 1 // Increments Using Dpad Side Buttons?
        LeftIntakeWrist = hardwareMap.servo.get("Left Intake Wrist");   // Chub Port 2 // Ideally Stick Controlled
        IntakeV4B = hardwareMap.servo.get("Intake V4B");     // Chub Port 3 // Preset To Swing Out With X
        // Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------
        //  Servo RightHook = hardwareMap.servo.get("Right Hook");                // Chub Port 5 // Linked To LeftHook Activated At The Same Time

        IntakeClaw = hardwareMap.servo.get("Intake Claw");

        OuttakeClaw = hardwareMap.servo.get("Outtake Claw");            // Ehub Port 0 // If Slides Up O Activates This Claw
        OuttakeWrist = hardwareMap.servo.get("Outtake Wrist");          // Ehub Port 1 // Preset To Go To Delivery Position With Triangle
        OuttakeV4B = hardwareMap.servo.get("Outtake V4B");

        LeftServo = Math.max(0, Math.min(1, Flex - (.5 * Yaw)));
        RightServo = Math.max(0, Math.min(1, Flex + (.5 * Yaw)));

        LeftIntakeWrist.setPosition(LeftServo);    // Sets the intake wrist to the starting position // Left is 0 Right is 1
        RightIntakeWrist.setPosition(RightServo);   // Sets the intake wrist to the starting position // Left is 0 Right is 1
        IntakeV4B.setPosition(.8);   // Sets the intake virtual four bar to the starting position
        //RightIntakeV4B.setPosition(1);  // Sets the intake virtual four bar to the starting position

        IntakeClaw.setPosition(0);    // Closes Intake Claw

        OuttakeClaw.setPosition(0);   // Closes Outtake Claw

        OuttakeWrist.setPosition(0);    // Sets the outtake wrist to the starting position

        OuttakeV4B.setPosition(1);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
    }
}
