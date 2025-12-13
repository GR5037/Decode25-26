package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import android.os.SystemClock;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "BLUE_KitBotAuto", group = "Auto",  preselectTeleOp= "GoBildaTele")
public class BLUE_KitBotAuto extends OpMode {

    Robot robot = new Robot();

//    private Pose  startPose = new Pose(24, 127, Math.toRadians(323));
//
//    private Pose moveToPush = new Pose(-72, 9, Math.toRadians(90));
//    private Pose moveToPushControlPoint = new Pose(68, 66);
//
//    private Pose Push = new Pose(50, 8.5);
//    private Pose toEnd = new Pose(39, 39, Math.toRadians(90));
//
//
//
//    private Path movingToPush, pushing, ending;
//
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState;
//
//    private void buildPaths() {
//            movingToPush = new Path(new BezierCurve(startPose, moveToPush, moveToPushControlPoint));
//            movingToPush.setLinearHeadingInterpolation(startPose.getHeading(), moveToPush.getHeading());
//
//            pushing = new Path(new BezierLine(moveToPush, Push));
//            pushing.setConstantHeadingInterpolation(Push.getHeading());
//
//            ending = new Path(new BezierLine(Push, toEnd));
//            ending.setLinearHeadingInterpolation(Push.getHeading(), toEnd.getHeading());
//
//
//    }
//
//    public void autoPathUpdate() {
//        switch (pathState) {
//            case 0:
//                setPathState(1);
//                break;
//            case 1:
//                follower.followPath(movingToPush);
//                setPathState(2);
//                break;
////            case 2:
////                if (!follower.isBusy()) {
////                    follower.followPath(pushing);
////                    setPathState(3);
////                }
////
////                break;
////            case 3:
////                if (!follower.isBusy()) {
////                    follower.followPath(ending);
////                    setPathState(4);
////                }
////
////                break;
//        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }

    public void init() {

        robot.init(hardwareMap);

//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);

    }

    public void loop() {
//        follower.update();
//        autoPathUpdate();
        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();

    }


//    void loop() {
//
//    }
//
//     void init() {
//         robot.init(hardwareMap);
//
//     }
//
    public void init_loop() {

    }

    public  void start() {
        robot.launcherStart(1350);
        SystemClock.sleep(1600);
        robot.startFeeder();
        SystemClock.sleep(200);
        robot.stopFeeder();
        SystemClock.sleep(1500);
        robot.startFeeder();
        SystemClock.sleep(200);
        robot.stopFeeder();
        SystemClock.sleep(1500);
        robot.startFeeder();
        SystemClock.sleep(200);
        robot.stopFeeder();
        SystemClock.sleep(1000);

//        setPathState(0);
        robot.drive(-.5,0.0,0.0);
        SystemClock.sleep(300);
        robot.drive(0.0,0.0,-0.3);
        SystemClock.sleep(700);
        robot.drive(0.5,0.0,0.0);
        SystemClock.sleep(600);
        robot.drive(0.0,0.0,0.3);
        SystemClock.sleep(600);
        robot.killMotors();
        robot.launcherStop();
        robot.feederStop();

        requestOpModeStop();

//        robot.drive(-.5,0.0,0.0);
//        SystemClock.sleep(415);
//        robot.killMotors();
//
//        robot.drive(0.0,0.0,-0.1);
//        SystemClock.sleep(200);
//        robot.killMotors();
//
//        robot.launcherStart(1425);
//        SystemClock.sleep(1500);
//
//        int firstRecovery = 1500;
//        int secondRecovery = 1700;
//        int recovery = 1300;
//        int pause = 100;
//
//        for (int i = 0; i < 3; i++) {
//
//            robot.startFeeder();
//            SystemClock.sleep(pause);
//            robot.stopFeeder();
//
//            if (i == 0) {
//                SystemClock.sleep(firstRecovery);
//            } else if (i == 1) {
//                SystemClock.sleep(secondRecovery);
//            }
//            else {
//                SystemClock.sleep(recovery);
//            }
//        }
//
//
//        robot.drive(0.0,0.6,0.0);
//        SystemClock.sleep(500);
//
//        robot.launcherStop();
//        robot.killMotors();
//        robot.feederStop();
//
//        requestOpModeStop();

    }

    public void stop() {

   }


}
