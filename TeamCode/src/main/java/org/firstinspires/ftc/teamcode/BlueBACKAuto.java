package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.Robot.dexP;
import static org.firstinspires.ftc.teamcode.Robot.gateSwitch;
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
import static org.firstinspires.ftc.teamcode.Robot.teleIsRed;
import static org.firstinspires.ftc.teamcode.Robot.turretAllowedError;

import com.bylazar.telemetry.PanelsTelemetry;
import static java.lang.Thread.sleep;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
import com.bylazar.telemetry.TelemetryManager;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLUE BACK Auto", group = "Auto",  preselectTeleOp= "NewTele")

public class BlueBACKAuto extends OpMode {
    int Motif = 1; //1=GPP 2=PGP 3=PPG
    private TelemetryManager telemetryM;
    double hoodPosition = 0.25;
    double flywheelVelocity = 2000;
    double velocityMin = flywheelVelocity - 100;
    int turretOffset = 0;
    int spindexOffset = 0;
    boolean parkTime = false;

    Robot robot = new Robot();
    static int angle;

    ElapsedTime loopTime = new ElapsedTime();
    static DcMotor leftFrontDrive;
    static DcMotor rightFrontDrive;
    static DcMotor leftBackDrive;
    static DcMotor rightBackDrive;
    static Servo transfer;
    static Servo leftKickstand;
    static Servo rightKickstand;
    static Servo hood;
//    static ColorRangeSensor LTcolor;
//    static ColorRangeSensor LBcolor;
//    static ColorRangeSensor BTcolor;
//    static ColorRangeSensor BBcolor;
//    static ColorRangeSensor RTcolor;
//    static ColorRangeSensor RBcolor;
    static DcMotorEx spindex;
    static DcMotorEx flywheel;
    static DcMotorEx intake;
    static DcMotorEx turret;
    static DigitalChannel gate;
    static AnalogInput absTurret;
    static AnalogInput absSpindex;

    private final Pose startPose = new Pose(56, 7.54, Math.toRadians(90));
    private final Pose launchPosition = new Pose(56, 20, Math.toRadians(180));
    private final Pose lineUpPose = new Pose(52, 12, Math.toRadians(180));
    private final Pose intakeIn = new Pose(11.5, 10, Math.toRadians(180));
    private final Pose intakeOut = new Pose(17, 10, Math.toRadians(180));
    private final Pose park = new Pose(34, 22, Math.toRadians(180));
    private final Pose lineUpForSpike = new Pose(43,34,Math.toRadians(180));
    private final Pose intakeSpike = new Pose(19,34,Math.toRadians(180));
    private final Pose launchSpike = new Pose(56,20,Math.toRadians(180));
    private PathChain launchToIntake;
    private Path  launchPreload,  intakingOut, secondIntakingIn, launch, liningup, intakingSpike, launchingSpike;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    static double spindexError = 0, spindexLastError = 0, spindexDerivative = 0, spindexIntegralSum = 0, spindexPower = 0;
    static boolean spindexIsBusy = false;
    static double turretError = 0, turretLastError = 0, turretDerivative = 0, turretIntegralSum = 0, turretPower = 0;
    static boolean turretIsBusy = false;
    static int turretEncoder = 0;
    static ElapsedTime spindexTimer = new ElapsedTime();
    static ElapsedTime turretTimer = new ElapsedTime();
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

        liningup = new Path(new BezierLine(launchPosition, lineUpForSpike));
        liningup.setConstantHeadingInterpolation(lineUpForSpike.getHeading());

        intakingSpike = new Path(new BezierLine(lineUpForSpike, intakeSpike));
        intakingSpike.setConstantHeadingInterpolation(intakeSpike.getHeading());

        launchingSpike = new Path(new BezierLine(intakeSpike,launchSpike));
        launchingSpike.setConstantHeadingInterpolation(launchSpike.getHeading());
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(1.0);
                flywheel.setVelocity(flywheelVelocity);
                hood.setPosition(hoodPosition);
                aimTurret(-70);
                follower.followPath(launchPreload);
                setPathState(pathState + 1);
                break;
            case 1:
                if (!follower.isBusy() && (flywheel.getVelocity() > velocityMin)  && !spindexIsBusy) {
                    launchAllSwitch = 1;
                    setPathState(pathState + 1);
                }
                break;
            case 2:
                if (launchAllSwitch > 6) {
                    follower.followPath(launchToIntake);
                    gateSwitch = 1;
                    setPathState(pathState + 1);
                }
                break;
            case 3:
                if (follower.getPose().getX() > 131 || !follower.isBusy()) {
                    follower.followPath(intakingOut);
                    setPathState(pathState + 1);
                }
                break;
            case 4:
                if (gateSwitch > 2 || pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(secondIntakingIn);
                    setPathState(pathState + 1);
                }
                break;
            case 5:
                if (gateSwitch > 6 || pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(launch);
                    setPathState(pathState + 1);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    gateSwitch = 0;
                    launchAllSwitch = 1;
                    setPathState(pathState + 1);
                }
                break;
            case 7:
                if (launchAllSwitch > 6){
                    follower.followPath(liningup);
                    gateSwitch = 1;
                    setPathState(pathState + 1);
                }
                break;
            case 8:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.7);
                    follower.followPath(intakingSpike);
                    setPathState(pathState + 1);
                }
                break;
            case 9:
                if (gateSwitch > 6 || pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(launchingSpike);
                    setPathState(pathState + 1);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    gateSwitch = 0;
                    launchAllSwitch = 1;
                    setPathState(pathState + 1);
                }
                break;
            case 11:
                if (launchAllSwitch > 6) {
                    follower.followPath(launchToIntake);
                    gateSwitch = 1;
                    setPathState(pathState + 1);
                }
                break;
            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(launch);
                    setPathState(10);
                }
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

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

        opmodeTimer.resetTimer();
    }

    public void loop() {
        follower.update();
        autoPathUpdate();
        turretPIDF();
        spindexPIDF();
        launchAll();
        autoIntake();



        if (opmodeTimer.getElapsedTimeSeconds() > 27 && !parkTime) {
            launchAllSwitch = 0;
            gateSwitch = 0;
            aimTurret(0);
            follower.breakFollowing();
            follower.holdPoint(park);
            parkTime = true;
            setPathState(-1);
        }
//         Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("velocity", flywheel.getVelocity());
//        telemetry.addData("turret", turret.isBusy());
//        telemetry.addData("follower", follower.isBusy());
//
//        telemetry.update();

        telemetryM.addData("loop time", loopTime.milliseconds());
        telemetryM.update();

        loopTime.reset();
    }

    public void stop() {
        Robot.xPoseFromAuto = follower.getPose().getX();
        Robot.yPoseFromAuto = follower.getPose().getY();
        Robot.headingFromAuto = follower.getHeading();
        Robot.teleIsRed = false;
    }

    public void launchAll() {
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

public  void aimTurret(double turretPose) {
    turretEncoder = (int) (((turretPose) /360) * 13332);
}

}



