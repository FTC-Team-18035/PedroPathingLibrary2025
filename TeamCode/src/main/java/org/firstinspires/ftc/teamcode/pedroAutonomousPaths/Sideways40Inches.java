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

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Disabled
@Autonomous(name = "40 inches sideways")
public class Sideways40Inches extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;

    private int pathState;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    private final Pose endPose = new Pose(0, 40, Math.toRadians(0));

    private Path driveFarRight, driveFarLeft, driveNearLeft, driveNearRight, drive;

    public void buildPaths() {
       // driveFarRight = new Path(new BezierLine(new Point(startPose), new Point(farRightPose)));
       // driveFarRight.setLinearHeadingInterpolation(startPose.getHeading(), farRightPose.getHeading());

        drive = new Path(new BezierLine(new Point(startPose), new Point(endPose)));
        drive.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(drive);
                setPathValue(1);
                break;

            case 1:
                if(follower.getPose().getX() > (endPose.getX() - 1) && follower.getPose().getY() > (endPose.getY() - 1)) {
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
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }
}
