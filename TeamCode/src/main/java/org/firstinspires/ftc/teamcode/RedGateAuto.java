package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "RedGateAuto", group = "Auto",  preselectTeleOp= "GoBildaTele")
public class RedGateAuto extends OpMode {

    Robot robot = new Robot();

    private Pose startPose = new Pose(122, 123, Math.toRadians(36));

    private Pose moveToGate = new Pose(120, 72, Math.toRadians(0));
    private Pose moveToGateControlPoint = new Pose(51, 64);

    private Pose PushGate = new Pose(130, 72);
    private Pose toEnd = new Pose(109, 10);
    private Pose toEndControlPoint = new Pose(28, 75);



    private Path movingToGate, pushing, ending;


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private void buildPaths() {
        movingToGate = new Path(new BezierCurve(startPose, moveToGate, moveToGateControlPoint));
        movingToGate.setLinearHeadingInterpolation(startPose.getHeading(), moveToGate.getHeading());

        pushing = new Path(new BezierLine(moveToGate, PushGate));
        pushing.setConstantHeadingInterpolation(PushGate.getHeading());

        ending = new Path(new BezierLine(PushGate, toEnd));
        ending.setConstantHeadingInterpolation(PushGate.getHeading());


    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(1);
                break;
            case 1:
                follower.followPath(movingToGate);
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pushing);
                    setPathState(3);
                }

                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(ending);
                    setPathState(4);
                }

                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void init() {

        robot.init(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    public void loop() {
        follower.update();
        autoPathUpdate();
//         Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    public void init_loop() {

    }

    public  void start() {
//
    }

    public void stop() {

    }


}
