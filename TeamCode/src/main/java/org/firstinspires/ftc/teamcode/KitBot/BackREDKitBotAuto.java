package org.firstinspires.ftc.teamcode.KitBot;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BackREDKitBotAuto", group = "Auto",  preselectTeleOp= "RED_KitBotTele")
@Disabled

public class BackREDKitBotAuto extends OpMode {

    KitBotRobot robot = new KitBotRobot();

    private Pose startPose = new Pose(85.78613861386138, 8.940594059405953, Math.toRadians(90));

    private Pose moveToShoot = new Pose(100, 95, Math.toRadians(50));

    private Pose toEnd = new Pose(80, 42, Math.toRadians(25));



    private Path movingToShoot, ending;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    ElapsedTime runtime = new ElapsedTime();
    int shootState = 0;

    private void buildPaths() {
        movingToShoot = new Path(new BezierCurve(startPose, moveToShoot));
        movingToShoot.setLinearHeadingInterpolation(startPose.getHeading(),moveToShoot.getHeading());

        ending = new Path(new BezierLine(moveToShoot, toEnd));
        ending.setConstantHeadingInterpolation(toEnd.getHeading());


    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(1);
                break;
            case 1:
                follower.followPath(movingToShoot);
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy()) {
                    switch (shootState) {

                        case 0:
                            robot.launcherStart(1450);
                            runtime.reset();
                            shootState = 1;
                            break;

                        case 1:
                            if (runtime.milliseconds() >= 1400) {
                                robot.startFeeder();
                                runtime.reset();
                                shootState = 2;
                            }
                            break;

                        case 2:
                            if (runtime.milliseconds() >= 90) {
                                robot.stopFeeder();
                                runtime.reset();
                                shootState = 3;
                            }
                            break;

                        case 3:
                            if (runtime.milliseconds() >= 1400) {
                                robot.startFeeder();
                                runtime.reset();
                                shootState = 4;
                            }
                            break;

                        case 4:
                            if (runtime.milliseconds() >= 90) {
                                robot.stopFeeder();
                                runtime.reset();
                                shootState = 5;
                            }
                            break;

                        case 5:
                            if (runtime.milliseconds() >= 1400) {
                                robot.startFeeder();
                                runtime.reset();
                                shootState = 6;
                            }
                            break;

                        case 6:
                            if (runtime.milliseconds() >= 90) {
                                robot.stopFeeder();
                                runtime.reset();
                                shootState = 7; // done
                            }
                            break;
                        case 7:
                            if (runtime.milliseconds() >= 400) {
                                robot.launcherStop();
                                runtime.reset();
                                shootState = 8;
                                setPathState(3);
                            }
                    }
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(ending);
                    setPathState(4);
                }
                break;
            case 4:
                robot.killMotors();
                setPathState(5);
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
        follower.setMaxPower(0.75);
        autoPathUpdate();
//         Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();


    }

}



