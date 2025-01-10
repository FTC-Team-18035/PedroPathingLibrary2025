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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


@Autonomous(name = "Left Parking Auto")
public class ParkingPathLeft extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;
    private int pathState;

    private final Pose startPose = new Pose(10, 81, Math.toRadians(0));

    private final Pose moveLeftPose = new Pose(13, 111, Math.toRadians(0));
    private final Pose moveForwardPose = new Pose(62, 111, Math.toRadians(0));
    private final Pose parkingPose = new Pose(62, 87, Math.toRadians(0));

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
            requestOpModeStop();
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
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }
}
