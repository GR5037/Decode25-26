package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.Robot.flywheelVelocity;
import static org.firstinspires.ftc.teamcode.Robot.gateSwitch;
import static org.firstinspires.ftc.teamcode.Robot.hoodAngle;
import static org.firstinspires.ftc.teamcode.Robot.launchAllSwitch;
import static org.firstinspires.ftc.teamcode.Robot.numberOfSpins;
import static org.firstinspires.ftc.teamcode.Robot.sD;
import static org.firstinspires.ftc.teamcode.Robot.sF;
import static org.firstinspires.ftc.teamcode.Robot.sI;
import static org.firstinspires.ftc.teamcode.Robot.sP;
import static org.firstinspires.ftc.teamcode.Robot.spindexAllowedError;
import static org.firstinspires.ftc.teamcode.Robot.spindexPose;
import static org.firstinspires.ftc.teamcode.Robot.tD;
import static org.firstinspires.ftc.teamcode.Robot.tF;
import static org.firstinspires.ftc.teamcode.Robot.tI;
import static org.firstinspires.ftc.teamcode.Robot.tP;
import static org.firstinspires.ftc.teamcode.Robot.turretAllowedError;
import static org.firstinspires.ftc.teamcode.Robot.turretCompConstant;
import static org.firstinspires.ftc.teamcode.Robot.turretDistanceToGoal;
import static org.firstinspires.ftc.teamcode.Robot.xGoalCompConstant;
import static org.firstinspires.ftc.teamcode.Robot.xTurretPose;
import static org.firstinspires.ftc.teamcode.Robot.yGoalCompConstant;
import static org.firstinspires.ftc.teamcode.Robot.yRedGoal;
import static org.firstinspires.ftc.teamcode.Robot.yTurretPose;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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

@Autonomous(name = "MODDED RED FRONT Auto", group = "Auto",  preselectTeleOp= "NewTele")

public class MODDED_RedFRONTAuto extends OpMode {
    int Motif = 1; //1=GPP 2=PGP 3=PPG
    int turretOffset = 0;
    int spindexOffset = 0;
    boolean parkTime = false;

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
    static AnalogInput absTurret;
    static AnalogInput absSpindex;
    private TelemetryManager telemetryM;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    static ElapsedTime runtime = new ElapsedTime();
    public PathChain launch0;
    public PathChain intake1;
    public PathChain openGate;
    public PathChain launch1;
    public PathChain lineUpFor2;
    public PathChain intake2;
    public PathChain launch2;
    public PathChain firstLineUpFor3;
    public PathChain secondLineUpFor3;
    public PathChain intake3;
    public PathChain launch3;
    public PathChain goOpenGate;
    public PathChain park;
    public Pose startPose = new Pose(106.77, 119.24, 0.694);
    static double spindexError = 0, spindexLastError = 0, spindexDerivative = 0, spindexIntegralSum = 0, spindexPower = 0;
    static boolean spindexIsBusy = false;
    static double turretError = 0, turretLastError = 0, turretDerivative = 0, turretIntegralSum = 0, turretPower = 0;
    static boolean turretIsBusy = false;
    double compedXGoal;
    double compedYGoal;
    double currentTurretAngle = 0;
    double deltaTargetAngle = 0;
    double lastTurretAngle = 0;
    double totalTurretAngle = 0;
    static int turretEncoder = 0;
    static ElapsedTime spindexTimer = new ElapsedTime();
    static ElapsedTime turretTimer = new ElapsedTime();

    private Limelight3A limelight;

    private void buildPaths() {
        launch0 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(106.77, 119.24),
                                new Pose(88, 78)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intake1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88, 78),
                                new Pose(117, 77)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        openGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(117, 77),
                                new Pose(111, 74),
                                new Pose(115.5, 64)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        launch1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(115.5, 64),
                                new Pose(105.736, 74.313),
                                new Pose(90, 86)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        lineUpFor2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(90, 86),
                                new Pose(92.937, 62.032),
                                new Pose(90, 54)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        intake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(90, 54),
                                new Pose(115, 52.5)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        launch2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(115, 52.5),
                                new Pose(92, 61),
                                new Pose(87, 78.137)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        goOpenGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87, 78.137),
                                new Pose(115.5, 64)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

//        firstLineUpFor3 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(87, 78.137),
//                                new Pose(92, 48.127)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .setNoDeceleration()
//                .build();
//
//        secondLineUpFor3 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(92, 48.127),
//                                new Pose(93, 32)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-72), Math.toRadians(0))
//                .build();
//
//        intake3 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(93, 32),
//                                new Pose(120, 32)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .build();
//
//        launch3 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(120, 32),
//                                new Pose(86, 79)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(115.5, 64),
                                new Pose(109.5, 64)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(1.0);
                follower.followPath(launch0);
                setPathState(pathState + 1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    launchAllSwitch = 1;
                    setPathState(pathState + 1);
                }
                break;
            case 2:
                if (launchAllSwitch == 0) {
                    follower.followPath(intake1);
                    follower.setMaxPower(0.6);
                    gateSwitch = 1;
                    setPathState(pathState + 1);
                }
                break;
            case 3:
                if (!follower.isBusy() && (gateSwitch == 0 || pathTimer.getElapsedTimeSeconds() > 4)) {
                    follower.followPath(openGate);
                    follower.setMaxPower(0.8);
                    setPathState(pathState + 1);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    setPathState(pathState + 1);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    follower.followPath(launch1);
                    follower.setMaxPower(1.0);
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
                    follower.followPath(lineUpFor2);
                    setPathState(pathState + 1);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(intake2);
                    gateSwitch = 1;
                    follower.setMaxPower(0.6);
                    setPathState(pathState + 1);
                }
                break;
            case 9:
                if (!follower.isBusy() && (gateSwitch == 0 || pathTimer.getElapsedTimeSeconds() > 4)) {
                    follower.followPath(launch2);
                    follower.setMaxPower(1.0);
                    setPathState(pathState + 1);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    launchAllSwitch = 1;
                    setPathState(pathState + 1);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(goOpenGate);
                    follower.setMaxPower(0.7);
                    setPathState(pathState + 1);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    setPathState(pathState + 1);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(park);
                    follower.setMaxPower(0.7);
                    setPathState(pathState + 1);
                }
                break;
            case 14:
                if (!follower.isBusy()) {

                }
                break;

//            case 11:
//                if (launchAllSwitch == 0) {
//                    follower.followPath(firstLineUpFor3);
//                    setPathState(pathState + 1);
//                }
//                break;
//            case 12:
//                if (!follower.isBusy()) {
//                    follower.followPath(secondLineUpFor3);
//                    setPathState(pathState + 1);
//                }
//                break;
//            case 13:
//                if (!follower.isBusy()) {
//                    follower.followPath(intake3);
//                    gateSwitch = 1;
//                    follower.setMaxPower(0.7);
//                    setPathState(pathState + 1);
//                }
//                break;
//            case 14:
//                if (!follower.isBusy() && (gateSwitch == 0 || pathTimer.getElapsedTimeSeconds() > 4)) {
//                    follower.followPath(launch3);
//                    follower.setMaxPower(1.0);
//                    setPathState(pathState + 1);
//                }
//                break;
//            case 15:
//                if (!follower.isBusy()) {
//                    launchAllSwitch = 1;
//                    setPathState(pathState + 1);
//                }
//                break;
//            case 16:
//                if (launchAllSwitch == 0) {
//                    follower.followPath(park);
//                    parkTime = true;
//                    setPathState(pathState + 1);
//                }
//                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void init() {

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "fr");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "br");

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        transfer = hardwareMap.get(Servo.class, "transfer");
        leftKickstand = hardwareMap.get(Servo.class, "Lstand");
        rightKickstand = hardwareMap.get(Servo.class, "Rstand");
        hood = hardwareMap.get(Servo.class, "hood");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);

//        LTcolor = hardwareMap.get(NormalizedColorSensor.class, "LTcolor");
//        LBcolor = hardwareMap.get(NormalizedColorSensor.class, "LBcolor");
//        BTcolor = hardwareMap.get(NormalizedColorSensor.class, "BTcolor");
//        BBcolor = hardwareMap.get(NormalizedColorSensor.class, "BBcolor");
//        RTcolor = hardwareMap.get(NormalizedColorSensor.class, "RTcolor");
//        RBcolor = hardwareMap.get(NormalizedColorSensor.class, "RBcolor");
//
//        LTdist = hardwareMap.get(DistanceSensor.class, "LTcolor");
//        LBdist = hardwareMap.get(DistanceSensor.class, "LBcolor");
//        RTdist = hardwareMap.get(DistanceSensor.class, "RTcolor");
//        RBdist = hardwareMap.get(DistanceSensor.class, "RBcolor");
//        BTdist = hardwareMap.get(DistanceSensor.class, "BTcolor");
//        BBdist = hardwareMap.get(DistanceSensor.class, "BBcolor");

        spindex = hardwareMap.get(DcMotorEx.class, "spindex");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        spindex.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        turret.setDirection(DcMotorEx.Direction.REVERSE);

        spindex.setZeroPowerBehavior(BRAKE);
        flywheel.setZeroPowerBehavior(FLOAT);
        intake.setZeroPowerBehavior(BRAKE);
        turret.setZeroPowerBehavior(BRAKE);

        PIDFCoefficients flywheelpidfCoefficients = new PIDFCoefficients(Robot.flyP, Robot.flyI, Robot.flyD, Robot.flyF);

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelpidfCoefficients);
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gate = hardwareMap.get(DigitalChannel.class, "gate");
        gate.setMode(DigitalChannel.Mode.INPUT);
        absTurret = hardwareMap.get(AnalogInput.class, "ABSturret");
        absSpindex = hardwareMap.get(AnalogInput.class, "ABSspindex");

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
        spindexOffset = (int) ( ((absSpindex.getVoltage() / 3.3) * 4000) - 30);
        turretOffset  = (int) ( ((absTurret.getVoltage() / 3.3) * 4000) - 2550);

        limelight.start();
        opmodeTimer.resetTimer();
    }

    public void loop() {
        follower.update();
        turretPIDF();
        spindexPIDF();
        autoPathUpdate();
        launchAll();
        autoIntake();
        if (opmodeTimer.getElapsedTimeSeconds() > 28) {
            parkTime = true;
        }
        if (parkTime == false) {
            calculations();
        } else {
            turretEncoder = 0;
        }

        LLResult llResult = limelight.getLatestResult();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

//         Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("velocity", flywheel.getVelocity());
//        telemetry.addData("turret", turret.isBusy());
//        telemetry.addData("follower", follower.isBusy());
        telemetry.addData("goal", hoodAngle);
        telemetry.addData("now", hood.getPosition());

        telemetry.update();

    }

    public void stop() {
        Robot.xPoseFromAuto = follower.getPose().getX();
        Robot.yPoseFromAuto = follower.getPose().getY();
        Robot.headingFromAuto = follower.getHeading();
        Robot.teleIsRed = true;
    }


    public  void  launchAll() {
        switch (Robot.launchAllSwitch) { //launch all balls
            case 1:
                transfer.setPosition(Robot.transfer1);
                if (Robot.numberOfBalls > 0) {
                    Robot.numberOfBalls--;
                }
                runtime.reset();
                Robot.launchAllSwitch++;
                break;
            case 2:
                if (runtime.milliseconds() > Robot.transferOneTime) {
                    spin1CW();
                    Robot.launchAllSwitch++;
                }
                break;
            case 3:
                if (!spindexIsBusy && flywheel.getVelocity() > flywheelVelocity - 100) {
                    transfer.setPosition(Robot.transfer2);
                    if (Robot.numberOfBalls > 0) {
                        Robot.numberOfBalls--;
                    }
                    runtime.reset();
                    Robot.launchAllSwitch++;
                }
                break;
            case 4:
                if (runtime.milliseconds() > Robot.transferOneTime) {
                    spin1CW();
                    Robot.launchAllSwitch++;
                }
                break;
            case 5:
                if (!spindexIsBusy && flywheel.getVelocity() > flywheelVelocity - 100) {
                    transfer.setPosition(Robot.transfer3);
                    if (Robot.numberOfBalls > 0) {
                        Robot.numberOfBalls--;
                    }
                    runtime.reset();
                    Robot.launchAllSwitch++;
                }
                break;
            case 6:
                if (runtime.milliseconds() > Robot.transferOneTime) {
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

    public  void autoIntake() {
        switch (Robot.gateSwitch) {
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
                    Robot.gateSwitch++;
                    runtime.reset();
                }
                break;
            case 7:
                if (runtime.seconds() > 0.25) {
                    intake.setPower(-0.3);
                    Robot.gateSwitch = 0;
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

    }

    public void spindexPIDF(){
        spindexError = spindexPose - spindex.getCurrentPosition() - spindexOffset;
        if (Math.abs(spindexError) > spindexAllowedError) {
            spindexIsBusy = true;
            spindexDerivative = (spindexError - spindexLastError) / spindexTimer.milliseconds();
            spindexIntegralSum = spindexIntegralSum + (spindexError * spindexTimer.milliseconds());
            spindexPower = (sF * Math.signum(spindexError)) + (sP * spindexError) + (sI * spindexIntegralSum) + (sD * spindexDerivative);

            spindex.setPower(spindexPower);
            spindexLastError = spindexError;
            spindexTimer.reset();
        } else {
            spindex.setPower(0);
            spindexIsBusy = false;
        }
    }

    public void turretPIDF(){
        turretError =  turretEncoder - turret.getCurrentPosition() - turretOffset;
        if (Math.abs(turretError) > turretAllowedError) {
            turretIsBusy = true;
            turretDerivative = (turretError - turretLastError) / turretTimer.milliseconds();
            turretIntegralSum = turretIntegralSum + (turretError * turretTimer.milliseconds());
            turretPower = (tF * Math.signum(turretError)) + (tP * turretError) + (tI * turretIntegralSum) + (tD * turretDerivative);

            turret.setPower(turretPower);
            turretLastError = turretError;
            turretTimer.reset();
        } else {
            turret.setPower(0);
            turretIsBusy = false;
        }
    }

    public void calculations(){
        compedXGoal = Robot.xRedGoal + xGoalCompConstant * follower.getVelocity().getXComponent();
        compedYGoal = yRedGoal + yGoalCompConstant * follower.getVelocity().getYComponent();

        double robotHeading = follower.getPose().getHeading();

        Robot.xTurretPose = follower.getPose().getX() + 2.7062 * (Math.cos(robotHeading + Math.toRadians(85.24)));
        Robot.yTurretPose = follower.getPose().getY() + 2.7062 * (Math.sin(robotHeading + Math.toRadians(85.24)));

        Robot.turretDistanceToGoal = Math.hypot(Robot.xTurretPose - compedXGoal, Robot.yTurretPose - compedYGoal);

        Robot.velocity1 = Robot.velocityA * (Math.pow(Robot.turretDistanceToGoal, 2));
        Robot.velocity2 = Robot.velocityB * Robot.turretDistanceToGoal;
        Robot.flywheelVelocity = (int) (Robot.velocity1 + Robot.velocity2 + Robot.velocityC);
        if (Robot.flywheelVelocity > Robot.velocityMax) {
            Robot.flywheelVelocity = Robot.velocityMax;
        } else if (Robot.flywheelVelocity < Robot.velocityMin) {
            Robot.flywheelVelocity = Robot.velocityMin;
        }

        Robot.angle1 = Robot.angleA * (Math.pow(turretDistanceToGoal , 2));
        Robot.angle2 = Robot.angleB * turretDistanceToGoal;
        Robot.hoodAngle = (Robot.angle1 + Robot.angle2 + Robot.angleC);
        if (Robot.hoodAngle < Robot.hoodMax) {
            Robot.hoodAngle = Robot.hoodMax;
        } else if (Robot.hoodAngle > Robot.hoodMin) {
            Robot.hoodAngle = Robot.hoodMin;
        }

        currentTurretAngle = Math.atan2((compedYGoal - yTurretPose),(compedXGoal - xTurretPose)) - robotHeading;
        deltaTargetAngle = currentTurretAngle - lastTurretAngle;
        if (deltaTargetAngle < -Math.PI) {
            deltaTargetAngle += 2 * Math.PI;
        } else if (deltaTargetAngle > Math.PI) {
            deltaTargetAngle -= 2 * Math.PI;
        }
        totalTurretAngle += deltaTargetAngle;
        if (totalTurretAngle < -Math.toRadians(300)){
            totalTurretAngle = Math.PI / 2;
        } else if (totalTurretAngle > Math.toRadians(300)){
            totalTurretAngle = -Math.PI / 2;
        }
        lastTurretAngle = totalTurretAngle;
        turretEncoder = (int) ((totalTurretAngle / (2 * Math.PI)) * 13332);
        turretEncoder = (int) (turretEncoder + turretCompConstant * follower.getAngularVelocity());

        hood.setPosition(Robot.hoodAngle);
        flywheel.setVelocity(flywheelVelocity);
    }

}



