package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.Robot.gateSwitch;
import static org.firstinspires.ftc.teamcode.Robot.launchAllSwitch;
import static org.firstinspires.ftc.teamcode.Robot.numberOfSpins;
import static org.firstinspires.ftc.teamcode.Robot.spindexPose;
import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RED BACK Auto", group = "Auto",  preselectTeleOp= "RedTele")

public class RedBACKAuto extends OpMode {
    static int Motif = 1; //1=GPP 2=PGP 3=PPG
    static int turretAnglePreload = -20;
    static int turretAngleIntake = 72;
    static double hoodPosition = 0.4;
    static double flywheelVelocity = 2260;

    Robot robot = new Robot();
    static int angle;


    static DcMotor leftFrontDrive;
    static DcMotor rightFrontDrive;
    static DcMotor leftBackDrive;
    static DcMotor rightBackDrive;
    static Servo transfer;
    static Servo leftKickstand;
    static Servo rightKickstand;
    static Servo hood;
    static ColorRangeSensor LTcolor;
    static ColorRangeSensor LBcolor;
    static ColorRangeSensor BTcolor;
    static ColorRangeSensor BBcolor;
    static ColorRangeSensor RTcolor;
    static ColorRangeSensor RBcolor;
    static DcMotorEx spindex;
    static DcMotorEx flywheel;
    static DcMotorEx intake;
    static DcMotorEx turret;
    static DigitalChannel gate;
    static AnalogInput ABSturret;
    static AnalogInput ABSspindex;

    private final Pose startPose = new Pose(88, 7.54, Math.toRadians(90));
    private final Pose launchPosition = new Pose(88, 20, Math.toRadians(0));
    private final Pose lineUpPose = new Pose(112, 9.5, Math.toRadians(0));
    private final Pose intakeIn = new Pose(131.5, 9.5, Math.toRadians(0));
    private final Pose intakeOut = new Pose(126, 9.5, Math.toRadians(0));
    private PathChain launchToIntake;
    private Path  launchPreload,  intakingOut, secondIntakingIn, launch;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    static ElapsedTime runtime = new ElapsedTime();

    private void buildPaths() {
        launchPreload = new Path(new BezierLine(startPose, launchPosition));
        launchPreload.setLinearHeadingInterpolation(startPose.getHeading(), launchPosition.getHeading());

        launchToIntake = follower.pathBuilder()
            .addPath(new BezierLine(launchPosition, lineUpPose))
            .setConstantHeadingInterpolation(lineUpPose.getHeading())
            .addPath(new BezierLine(lineUpPose, intakeIn))
            .setConstantHeadingInterpolation(intakeIn.getHeading())
            .build();

        intakingOut = new Path(new BezierLine(intakeIn, intakeOut));
        intakingOut.setConstantHeadingInterpolation(intakeOut.getHeading());

        secondIntakingIn = new Path(new BezierLine(intakeOut, intakeIn));
        secondIntakingIn.setConstantHeadingInterpolation(intakeIn.getHeading());

        launch = new Path(new BezierLine(intakeIn, launchPosition));
        launch.setConstantHeadingInterpolation(launchPosition.getHeading());
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                flywheel.setVelocity(flywheelVelocity);
                hood.setPosition(hoodPosition);
                spindex.setTargetPosition(0);
                spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spindex.setPower(1.0);
                aimTurret(66);
                follower.followPath(launchPreload);
                setPathState(pathState + 1);
                break;
            case 1:
                if (!follower.isBusy() && (flywheel.getVelocity() > (flywheelVelocity - 60))) {
                    launchAllSwitch = 1;
                    setPathState(pathState + 1);
                }
                break;
            case 2:
                if (launchAllSwitch == 0) {
                    follower.followPath(launchToIntake);
                    gateSwitch = 1;
                    setPathState(pathState + 1);
                }
                break;
            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(intakingOut);
                    setPathState(pathState + 1);
                }
                break;
            case 4:
                if (gateSwitch > 2 || pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(secondIntakingIn);
                    setPathState(pathState + 1);
                }
                break;
            case 5:
                if (gateSwitch > 6 || pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(launch);
                    intake.setPower(-0.3);
                    setPathState(pathState + 1);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    launchAllSwitch = 1;
                    setPathState(pathState + 1);
                }
                break;
            case 7:
                if (launchAllSwitch == 0) {
                    follower.followPath(launchToIntake);
                    gateSwitch = 1;
                    setPathState(pathState + 1);
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(launch);
                    setPathState(pathState + 1);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    launchAllSwitch = 1;
                    setPathState(pathState + 1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void init() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        transfer = hardwareMap.get(Servo.class, "transfer");
        leftKickstand = hardwareMap.get(Servo.class, "Lstand");
        rightKickstand = hardwareMap.get(Servo.class, "Rstand");
        hood = hardwareMap.get(Servo.class, "hood");

        LTcolor = hardwareMap.get(ColorRangeSensor.class, "LTcolor");
        LBcolor = hardwareMap.get(ColorRangeSensor.class, "LBcolor");
        BTcolor = hardwareMap.get(ColorRangeSensor.class, "BTcolor");
        BBcolor = hardwareMap.get(ColorRangeSensor.class, "BBcolor");
        RTcolor = hardwareMap.get(ColorRangeSensor.class, "RTcolor");
        RBcolor = hardwareMap.get(ColorRangeSensor.class, "RBcolor");

        spindex = hardwareMap.get(DcMotorEx.class, "spindex");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        spindex.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        turret.setDirection(DcMotorEx.Direction.REVERSE);

        spindex.setZeroPowerBehavior(FLOAT);
        flywheel.setZeroPowerBehavior(FLOAT);
        intake.setZeroPowerBehavior(BRAKE);
        turret.setZeroPowerBehavior(BRAKE);

        PIDFCoefficients flywheelpidfCoefficients = new PIDFCoefficients(Robot.flyP, Robot.flyI, Robot.flyD, Robot.flyF);
        PIDFCoefficients spindexpidfCoefficients = new PIDFCoefficients(0.3, 0.05, 0, 5);
        PIDFCoefficients turretpidCoefficients = new PIDFCoefficients(Robot.turP, Robot.turI, Robot.turD, Robot.turF);

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelpidfCoefficients);
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, spindexpidfCoefficients);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, turretpidCoefficients);
        turret.setPower(1.0);
        spindex.setTargetPositionTolerance(15);

        gate = hardwareMap.get(DigitalChannel.class, "gate");
        gate.setMode(DigitalChannel.Mode.INPUT);
        ABSturret = hardwareMap.get(AnalogInput.class, "ABSturret");
        ABSspindex = hardwareMap.get(AnalogInput.class, "ABSspindex");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        rightKickstand.setPosition(Robot.rkRaised);
        leftKickstand.setPosition(Robot.lkRaised);
        transfer.setPosition(Robot.transferRest);
        Robot.numberOfSpins = 0;
        Robot.numberOfBalls = 0;
        Robot.ejectSwitch = 0;
        Robot.kickstandSwitch = 0;
        Robot.launchOneSwitch = 0;
        Robot.launchAllSwitch = 0;
        Robot.gateSwitch = 0;
        Robot.spindexPose = 0;
        Robot.flywheelOn = false;

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    public void start() {

    }

    public void loop() {
        follower.update();
        autoPathUpdate();
        launchAll();
        autoIntake();
//         Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("velocity", flywheel.getVelocity());
        telemetry.addData("turret", turret.isBusy());
        telemetry.addData("follower", follower.isBusy());
        telemetry.update();

    }

    public void stop() {
        Robot.xPoseFromAuto = follower.getPose().getX();
        Robot.yPoseFromAuto = follower.getPose().getY();
        Robot.headingFromAuto = follower.getHeading();
    }


    public static void  launchAll() {
        switch (Robot.launchAllSwitch) { //launch all balls
            case 1:
                    transfer.setPosition(Robot.transfer1);
                    runtime.reset();
                    Robot.launchAllSwitch++;
                break;
            case 2:
                if (runtime.milliseconds() > 500) {
                    spin1CW();
                    Robot.launchAllSwitch++;
                }
                break;
            case 3:
                if (!spindex.isBusy()) {
                    transfer.setPosition(Robot.transfer2);
                    runtime.reset();
                    Robot.launchAllSwitch++;
                }
                break;
            case 4:
                if (runtime.milliseconds() > 500) {
                    spin1CW();
                    Robot.launchAllSwitch++;
                }
                break;
            case 5:
                if (!spindex.isBusy()) {
                    transfer.setPosition(Robot.transfer3);
                    runtime.reset();
                    Robot.launchAllSwitch++;
                }
                break;
            case 6:
                if (runtime.milliseconds() > 500) {
                    transfer.setPosition(Robot.transferRest);
                    runtime.reset();
                    Robot.launchAllSwitch++;
                }
                break;
            case 7:
                if (runtime.milliseconds() > 900) {
                    Robot.launchAllSwitch = 0;
                }
                break;
        }
    }

    public static void autoIntake() {
        switch (Robot.gateSwitch) {
            case 0:
                intake.setPower(0.0);
                break;
            case 1:
                intake.setPower(1.0);
                if (gate.getState()) {
                    Robot.gateSwitch++;
                }
                break;
            case 2:
                if (!gate.getState()) {
                    spin1CW();
                    Robot.gateSwitch++;
                }
                break;
            case 3:
                if (gate.getState()) {
                    Robot.gateSwitch++;
                }
                break;
            case 4:
                if (!gate.getState()) {
                    spin1CW();
                    Robot.gateSwitch++;
                }
                break;
            case 5:
                if (gate.getState()) {
                    Robot.gateSwitch++;
                }
                break;
            case 6:
                if (!gate.getState()) {
                    intake.setPower(-1.0);
                    runtime.reset();
                    Robot.gateSwitch++;
                }
                break;
            case 7:
                if (runtime.seconds() > 2) {
                    intake.setPower(-0.3);
                    gateSwitch++;
                }
                break;
        }
    }

    public static void spin1CW(){

        numberOfSpins--;
        spindexPose = spindexPose - 1333;
        if (numberOfSpins == 0) {
            spindexPose = 0;
        } else if (numberOfSpins % 3 == 0) {
            spindexPose--;
        }
        spindex.setTargetPosition(spindexPose);
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindex.setPower(1.0);
    }
    public static void spin2CW(){
        numberOfSpins = numberOfSpins - 2;
        spindexPose = spindexPose - 2666;
        if (numberOfSpins == 0) {
            spindexPose = 0;
        } else if (numberOfSpins % 3 == 0) {
            spindexPose--;
        }
        spindex.setTargetPosition(spindexPose);
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindex.setPower(1.0);
    }

    public static void aimTurret(double turretPose) {
        angle = (int) (((turretPose) /360) * 13332);
        turret.setTargetPosition(angle);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1.0);
    }

}



