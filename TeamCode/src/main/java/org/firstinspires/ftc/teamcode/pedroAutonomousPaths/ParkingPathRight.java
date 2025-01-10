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


@Autonomous(name = "Right Parking Auto")
public class ParkingPathRight extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;
    private int pathState;

    private final Pose startPose = new Pose(10, 57, Math.toRadians(0));

    private final Pose parkingPose = new Pose(13, 15, Math.toRadians(0));

    private Path parkPath;

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
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        if(opModeTimer.getElapsedTimeSeconds() >= 5) {
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
