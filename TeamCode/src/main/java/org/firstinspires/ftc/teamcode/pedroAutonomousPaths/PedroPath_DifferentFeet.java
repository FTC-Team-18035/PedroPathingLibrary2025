package org.firstinspires.ftc.teamcode.pedroAutonomousPaths;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Disabled
@Autonomous(name = "Pedro Distance Test")
public class PedroPath_DifferentFeet extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;

    private int pathState;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));



    private Path driveFarRight, driveFarLeft, driveNearLeft, driveNearRight;

    public void buildPaths() {
       // driveFarRight = new Path(new BezierLine(new Point(startPose), new Point(farRightPose)));
       // driveFarRight.setLinearHeadingInterpolation(startPose.getHeading(), farRightPose.getHeading());


    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                //follower.followPath(driveFarRight);
                //setPathValue(1);
                //break;

            case 1:
                //if(follower.getPose().getX() > (farRightPose.getX() - 1) && follower.getPose().getY() > (farRightPose.getY() - 1)) {
                  //  follower.followPath(driveFarLeft);
                   // setPathValue(-1);
                //}
                //break;


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
