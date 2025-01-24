package org.firstinspires.ftc.teamcode.pedroAutonomousPaths;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

// ********************************************* Right Autonomous Template ******************************************
@Disabled
@Autonomous(name = "Right Template", preselectTeleOp = "TeleOpWithPID")
public class PathRight_Template extends OpMode {
    // Pedro variables *************************************************************************	
    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;
    private int pathState;
    private final Pose startPose = new Pose(8.6, 56, Math.toRadians(0));
    private final Pose parkingPose = new Pose(12, 10, Math.toRadians(0));
    private Path parkPath;

    // Hardware variables ***********************************************************************
    public double LeftServo;
    public double RightServo;
    private double V4Bpos = 1;
    private double Flex = 0;
    private double Yaw = 0;

    Servo RightIntakeWrist; // Chub Port 1 // Increments Using Dpad Side Buttons?
    Servo LeftIntakeWrist;   // Chub Port 2 // Ideally Stick Controlled
    Servo IntakeV4B;          // Chub Port 3 // Preset To Swing Out With X
    Servo IntakeClaw;
    Servo OuttakeV4B;
    Servo OuttakeClaw;        // Ehub Port 0 // If Slides Up O Activates This Claw
    Servo OuttakeWrist;        // Ehub Port 1 // Preset To Go To Delivery Position With Triangle

    // Pedro pathing methods *************************************************************************
    public void buildPaths() {
    parkPath = new Path(new BezierLine(new Point(startPose), new Point(parkingPose)));
    parkPath.setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
        case 0:
            follower.followPath(parkPath);
            setPathValue(-1);
            break;
 	}
     }

     public void setPathValue(int pState) {
	pathState = pState;
	pathTimer.resetTimer();
     }

    @Override
    public void loop() {		//  ****************** Start of code loop *****************************
        follower.update();
        autonomousPathUpdate();

        if(opModeTimer.getElapsedTimeSeconds() >= 5) {
            OuttakeV4B.setPosition(.5);
            IntakeV4B.setPosition(.8);
            OuttakeWrist.setPosition(.25);
            if(opModeTimer.getElapsedTimeSeconds() >= 7) {
                requestOpModeStop();
            }
    }

    // Feedback to Driver Hub ***************************************************************************
    telemetry.addData("path state", pathState);
    telemetry.addData("x", follower.getPose().getX());
    telemetry.addData("y", follower.getPose().getY());
    telemetry.addData("heading", follower.getPose().getHeading());
    telemetry.update();
    }

    @Override
    public void init() {                  // Initialization steps **************************************************

       // Pedro path timer creation
       pathTimer = new Timer();


       // Pedro pathing building
       Constants.setConstants(FConstants.class, LConstants.class);
       follower = new Follower(hardwareMap);
       follower.setStartingPose(startPose);
       buildPaths();

       // Robot hardware creation
       RightIntakeWrist = hardwareMap.servo.get("Right Intake Wrist"); // Chub Port 1 // Increments Using Dpad Side Buttons?
       LeftIntakeWrist = hardwareMap.servo.get("Left Intake Wrist");   // Chub Port 2 // Ideally Stick Controlled
       IntakeV4B = hardwareMap.servo.get("Intake V4B");                // Chub Port 3 // Preset To Swing Out With X
       IntakeClaw = hardwareMap.servo.get("Intake Claw");
       OuttakeV4B = hardwareMap.servo.get("Outtake V4B");
       OuttakeClaw = hardwareMap.servo.get("Outtake Claw");            // Ehub Port 0 // If Slides Up O Activates This Claw
       OuttakeWrist = hardwareMap.servo.get("Outtake Wrist");          // Ehub Port 1 // Preset To Go To Delivery Position With Triangle

       // Intake wrist control formula
       LeftServo = Math.max(0, Math.min(1, Flex - (.5 * Yaw)));
       RightServo = Math.max(0, Math.min(1, Flex + (.5 * Yaw)));

       // Robot hardware postitioning
       LeftIntakeWrist.setPosition(LeftServo);     // Sets the intake wrist to the starting position // Left is 0 Right is 1
       RightIntakeWrist.setPosition(RightServo);   // Sets the intake wrist to the starting position // Left is 0 Right is 1
       IntakeV4B.setPosition(.8);                  // Sets the intake virtual four bar to the starting position
       IntakeClaw.setPosition(0);                  // Closes Intake Claw
       OuttakeV4B.setPosition(1);                  // Sets Outtake V4B to starting position
       OuttakeClaw.setPosition(0);                 // Closes Outtake Claw
       OuttakeWrist.setPosition(0);                // Sets the outtake wrist to the starting position

    }

    @Override
    public void start() {
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
    }
}

