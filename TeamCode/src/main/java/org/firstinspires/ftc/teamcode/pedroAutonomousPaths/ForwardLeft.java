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

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


@Autonomous(name = "Forward Left 1 Block")
public class ForwardLeft extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;

    private int pathState;

    private final Pose startPose = new Pose(8.6, 16, Math.toRadians(0));

    private final Pose forwardPose = new Pose(32, 16, Math.toRadians(0));

    private final Pose leftPose = new Pose(32, 41, Math.toRadians(0));


    private Path driveForward, driveLeft;

    public void buildPaths() {
        driveForward = new Path(new BezierLine(new Point(startPose), new Point(forwardPose)));
        driveForward.setConstantHeadingInterpolation(forwardPose.getHeading());

        driveLeft = new Path(new BezierLine(new Point(forwardPose), new Point(leftPose)));
        driveLeft.setConstantHeadingInterpolation(leftPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(driveForward);
                setPathValue(1);
                break;

            case 1:
                if(follower.getPose().getX() > (forwardPose.getX() - 1) && follower.getPose().getY() > (forwardPose.getY() - 1)) {
                    follower.followPath(driveLeft);
                    setPathValue(2);
                }
                break;


            case 3:
                if(follower.getPose().getX() > (leftPose.getX() - 1) && follower.getPose().getY() > (leftPose.getY() - 1)) {
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
