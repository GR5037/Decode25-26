package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;


import static org.firstinspires.ftc.teamcode.Robot.angleA;
import static org.firstinspires.ftc.teamcode.Robot.ejectSwitch;
import static org.firstinspires.ftc.teamcode.Robot.flywheelOn;
import static org.firstinspires.ftc.teamcode.Robot.flywheelVelocity;
import static org.firstinspires.ftc.teamcode.Robot.headingFromAuto;
import static org.firstinspires.ftc.teamcode.Robot.hoodMax;
import static org.firstinspires.ftc.teamcode.Robot.hoodMin;
import static org.firstinspires.ftc.teamcode.Robot.launchAllSwitch;
import static org.firstinspires.ftc.teamcode.Robot.numberOfBalls;
import static org.firstinspires.ftc.teamcode.Robot.numberOfSpins;
import static org.firstinspires.ftc.teamcode.Robot.spindexPose;
import static org.firstinspires.ftc.teamcode.Robot.transferRest;
import static org.firstinspires.ftc.teamcode.Robot.turretDistanceToGoal;
import static org.firstinspires.ftc.teamcode.Robot.velocityA;
import static org.firstinspires.ftc.teamcode.Robot.velocityB;
import static org.firstinspires.ftc.teamcode.Robot.velocityC;
import static org.firstinspires.ftc.teamcode.Robot.velocityMin;
import static org.firstinspires.ftc.teamcode.Robot.xBlueGoal;
import static org.firstinspires.ftc.teamcode.Robot.xPoseFromAuto;
import static org.firstinspires.ftc.teamcode.Robot.xRedGoal;
import static org.firstinspires.ftc.teamcode.Robot.xTurretPose;
import static org.firstinspires.ftc.teamcode.Robot.yGoal;
import static org.firstinspires.ftc.teamcode.Robot.yPoseFromAuto;
import static org.firstinspires.ftc.teamcode.Robot.yTurretPose;
import com.qualcomm.hardware.lynx.LynxModule;


import com.bylazar.panels.Panels;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;


//@Configurable
@TeleOp(name = "BlueTele", group = "Tele")
public class BlueTele extends OpMode {
    private Limelight3A limelight;
    static DcMotor leftFrontDrive;
    static DcMotor rightFrontDrive;
    static DcMotor leftBackDrive;
    static DcMotor rightBackDrive;
    static Servo transfer;
    static Servo leftKickstand;
    static Servo rightKickstand;
    static Servo hood;
    static NormalizedColorSensor LTcolor;
    static NormalizedColorSensor LBcolor;
    static NormalizedColorSensor BTcolor;
    static NormalizedColorSensor BBcolor;
    static NormalizedColorSensor RTcolor;
    static NormalizedColorSensor RBcolor;
    static DistanceSensor LTdist;
    static DistanceSensor LBdist;
    static DistanceSensor RTdist;
    static DistanceSensor RBdist;
    static DistanceSensor BTdist;
    static DistanceSensor BBdist;
    static DcMotorEx spindex;
    static DcMotorEx flywheel;
    static DcMotorEx intake;
    static DcMotorEx turret;
    static DigitalChannel gate;
    static AnalogInput absTurret;
    static AnalogInput absSpindex;
    static double leftFrontPower;
    static double rightFrontPower;
    static double leftBackPower;
    static double rightBackPower;
    Robot robot = new Robot();
    private Follower follower;
    private TelemetryManager telemetryM;
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime loopTime = new ElapsedTime();
    double currentTurretAngle = 0;
    double deltaTargetAngle = 0;
    double lastTurretAngle = 0;
    double totalTurretAngle = 0;
    int turretEncoder = 0;
    double driveSpeed = 1.0;
    boolean holdPosition = false;
    public Pose savedPose;

    public enum spindexPosition {
        LEFT,
        BACK,
        RIGHT
    }
    public enum spindexColor {
        PURPLE,
        GREEN,
        NONE
    }
    List<LynxModule> allHubs;
    List<String> queue = new ArrayList<>();


    @Override
    public void init() {
        robot = new Robot();
        follower = Constants.createFollower(hardwareMap);
        follower.update();
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

        LTcolor = hardwareMap.get(NormalizedColorSensor.class, "LTcolor");
        LBcolor = hardwareMap.get(NormalizedColorSensor.class, "LBcolor");
        BTcolor = hardwareMap.get(NormalizedColorSensor.class, "BTcolor");
        BBcolor = hardwareMap.get(NormalizedColorSensor.class, "BBcolor");
        RTcolor = hardwareMap.get(NormalizedColorSensor.class, "RTcolor");
        RBcolor = hardwareMap.get(NormalizedColorSensor.class, "RBcolor");

        LTdist = hardwareMap.get(DistanceSensor.class, "LTcolor");
        LBdist = hardwareMap.get(DistanceSensor.class, "LBcolor");
        RTdist = hardwareMap.get(DistanceSensor.class, "BTcolor");
        RBdist = hardwareMap.get(DistanceSensor.class, "BBcolor");
        BTdist = hardwareMap.get(DistanceSensor.class, "RTcolor");
        BBdist = hardwareMap.get(DistanceSensor.class, "RBcolor");

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
        PIDFCoefficients spindexpidfCoefficients = new PIDFCoefficients(Robot.dexP, Robot.dexI, Robot.dexD, Robot.dexF);
        PIDFCoefficients turretpidCoefficients = new PIDFCoefficients(Robot.turP, Robot.turI, Robot.turD, Robot.turF);

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelpidfCoefficients);
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, spindexpidfCoefficients);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, turretpidCoefficients);
        spindex.setTargetPositionTolerance(20);

        gate = hardwareMap.get(DigitalChannel.class, "gate");
        gate.setMode(DigitalChannel.Mode.INPUT);
        absTurret = hardwareMap.get(AnalogInput.class, "ABSturret");
        absSpindex = hardwareMap.get(AnalogInput.class, "ABSspindex");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

    }

    @Override
    public void start(){
//        limelight.start();

        rightKickstand.setPosition(Robot.rkRaised);
        leftKickstand.setPosition(Robot.lkRaised);
        transfer.setPosition(Robot.transferRest);
        Robot.numberOfSpins = 0;
        Robot.numberOfBalls = 0;
        Robot.ejectSwitch = 0;
        Robot. kickstandSwitch = 0;
        Robot. launchOneSwitch = 0;
        Robot. launchAllSwitch = 0;
        Robot.gateSwitch = 0;
        Robot.spindexPose = 0;
        Robot.flywheelOn = true;
        follower.setStartingPose(new Pose(xPoseFromAuto, yPoseFromAuto, headingFromAuto));
        follower.startTeleOpDrive(true);
        runtime.reset();
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs){
            hub.clearBulkCache();
        }

        follower.update();


        //Read Sensors


//        LLResult llResult = limelight.getLatestResult();

//        NormalizedRGBA colors1 = LTcolor.getNormalizedColors();
//        NormalizedRGBA colors2 = LBcolor.getNormalizedColors();
//        NormalizedRGBA colors3 = BTcolor.getNormalizedColors();
//        NormalizedRGBA colors4 = BBcolor.getNormalizedColors();
//        NormalizedRGBA colors5 = RTcolor.getNormalizedColors();
//        NormalizedRGBA colors6 = RBcolor.getNormalizedColors();
//
//        double dist1 = LTdist.getDistance(DistanceUnit.CM);
//        double dist2 = LBdist.getDistance(DistanceUnit.CM);
//        double dist3 = RTdist.getDistance(DistanceUnit.CM);
//        double dist4 = RBdist.getDistance(DistanceUnit.CM);
//        double dist5 = BTdist.getDistance(DistanceUnit.CM);
//        double dist6 = BBdist.getDistance(DistanceUnit.CM);

        //Relocalise (not cam yet)
        if (gamepad1.yWasPressed() && gamepad1.dpadUpWasPressed()) {
            follower.setPose(new Pose(7.54, 8.83,Math.toRadians(180)));
        }

        //Drive
        if (gamepad1.right_trigger > 0.1) {
            if (!holdPosition) {
                follower.breakFollowing();
                savedPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
                holdPosition = true;
                follower.holdPoint(savedPose);
            }
        } else {
            if (holdPosition) {
                follower.startTeleOpDrive(true);
                holdPosition = false;
            }
            if (gamepad1.left_trigger > 0.1) {
                driveSpeed = 0.4;
            } else {
                driveSpeed = 1.0;
            }
            follower.setTeleOpDrive(driveSpeed * -gamepad1.left_stick_y, driveSpeed * -gamepad1.left_stick_x, driveSpeed * -gamepad1.right_stick_x, !gamepad1.left_bumper);
        }

        //Calculations
        double robotHeading = follower.getPose().getHeading();

        Robot.xTurretPose = follower.getPose().getX() + 2.7062 * (Math.cos(robotHeading + Math.toRadians(85.24)));
        Robot.yTurretPose = follower.getPose().getY() + 2.7062 * (Math.sin(robotHeading + Math.toRadians(85.24)));

        Robot.turretDistanceToGoal = Math.hypot(Robot.xTurretPose - Robot.xBlueGoal, Robot.yTurretPose - Robot.yGoal);

        Robot.velocity1 = Robot.velocityA * (Math.pow(Robot.turretDistanceToGoal, 2));
        Robot.velocity2 = Robot.velocityB * Robot.turretDistanceToGoal;
        Robot.flywheelVelocity = (int) (Robot.velocity1 + Robot.velocity2 + Robot.velocityC);
        if (Robot.flywheelVelocity > Robot.velocityMax) {
            Robot.flywheelVelocity = Robot.velocityMax;
        } else if (Robot.flywheelVelocity < Robot.velocityMin) {
            Robot.flywheelVelocity = Robot.velocityMin;
        }

        Robot.angle1 = Robot.angleA * (Math.pow(flywheelVelocity , 2));
        Robot.angle2 = Robot.angleB * flywheelVelocity;
        Robot.hoodAngle = (Robot.angle1 + Robot.angle2 + Robot.angleC);
        if (Robot.hoodAngle < Robot.hoodMax) {
            Robot.hoodAngle = Robot.hoodMax;
        } else if (Robot.hoodAngle > Robot.hoodMin) {
            Robot.hoodAngle = Robot.hoodMin;
        }



        currentTurretAngle = Math.atan2((yGoal - yTurretPose),(xBlueGoal - xTurretPose)) - robotHeading;
        deltaTargetAngle = currentTurretAngle - lastTurretAngle;
        if (deltaTargetAngle < -Math.PI){
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

        turret.setTargetPosition(turretEncoder);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1.0);

        if (ejectSwitch == 0) {
            hood.setPosition(Robot.hoodAngle);
        }


        //Flywheel
        if (gamepad2.left_bumper) {
            Robot.flywheelOn = true;
//            PIDFCoefficients flywheelpidfCoefficients = new PIDFCoefficients(Robot.flyP, Robot.flyI, Robot.flyD, Robot.flyF);
//            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelpidfCoefficients);
        } else if (gamepad2.right_bumper) {
            Robot.flywheelOn = false;
            PIDFCoefficients spindexpidfCoefficients = new PIDFCoefficients(Robot.dexP, Robot.dexI, Robot.dexD, Robot.dexF);
            spindex.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, spindexpidfCoefficients);
            PIDFCoefficients turretpidCoefficients = new PIDFCoefficients(Robot.turP, Robot.turI, Robot.turD, Robot.turF);
            turret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, turretpidCoefficients);
        }
        if (!Robot.flywheelOn) {
            flywheel.setVelocity(0.0);
        } else if (ejectSwitch == 0){
            flywheel.setVelocity(flywheelVelocity);
        }

        //Intake
        if (gamepad2.left_trigger > 0.1) {
            intake.setPower(gamepad2.left_trigger);
            switch (Robot.gateSwitch) { //auto rotate spindex when ball enter (currently backwards b/c bad gate mechanism)
                case 0:
                    if (gate.getState()) {
                        Robot.gateSwitch++;
                    }
                    break;
                case 1:
                    if (!gate.getState()) {
                        spin1CW();
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);
                        Robot.gateSwitch++;
                    }
                    break;
                case 2:
                    if (gate.getState()) {
                        Robot.gateSwitch++;
                    }
                    break;
                case 3:
                    if (!gate.getState()) {
                        spin1CW();
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);
                        Robot.gateSwitch++;
                    }
                    break;
                case 4:
                    if (gate.getState()) {
                        Robot.gateSwitch++;
                    }
                    break;
                case 5:
                    if (!gate.getState()) {
                        gamepad1.rumbleBlips(2);
                        gamepad2.rumbleBlips(2);
                        Robot.gateSwitch++;
                    }
                    break;
            }
        } else if (gamepad2.right_trigger > 0.1){
            intake.setPower(-gamepad2.right_trigger);
            Robot.gateSwitch = 0;
        } else {
            intake.setPower(0);
            Robot.gateSwitch = 0;
        }


        //Manual spindex rotate
        if (gamepad2.dpadLeftWasPressed()) {
            spin1CCW();
        }
        if (gamepad2.dpadRightWasPressed()) {
            spin1CW();
        }

        //Launch
        switch (Robot.launchOneSwitch) { //launch one ball then get next ready
            case 0:
                if (gamepad2.a) {
                    queue.clear();
                    transfer.setPosition(Robot.transfer1);
                    if (Robot.numberOfBalls > 0) {
                        Robot.numberOfBalls--;
                    }
                    runtime.reset();
                    Robot.launchOneSwitch++;
                }
                break;
            case 1:
                if (runtime.milliseconds() > Robot.transferOneTime) {
                    transfer.setPosition(Robot.transferRest);
                    runtime.reset();
                    Robot.launchOneSwitch++;
                }
                break;
            case 2:
                if (runtime.milliseconds() > Robot.transferOneTime) {
                    Robot.launchOneSwitch = 0;
                }
                break;
        }
        switch (Robot.launchAllSwitch) { //launch all balls
            case 0:
                if (gamepad2.bWasPressed()) {
                    queue.clear();
                    transfer.setPosition(Robot.transfer1);
                    if (Robot.numberOfBalls > 0) {
                        Robot.numberOfBalls--;
                    }
                    runtime.reset();
                    Robot.launchAllSwitch++;
                }
                break;
            case 1:
                if (runtime.milliseconds() > Robot.transferOneTime) {
                    spin1CW();
                    Robot.launchAllSwitch++;
                }
                break;
            case 2:
                if (!spindex.isBusy() && flywheel.getVelocity() > flywheelVelocity - 100) {
                    transfer.setPosition(Robot.transfer2);
                    if (Robot.numberOfBalls > 0) {
                        Robot.numberOfBalls--;
                    }
                    runtime.reset();
                    Robot.launchAllSwitch++;
                }
                break;
            case 3:
                if (runtime.milliseconds() > Robot.transferOneTime) {
                    spin1CW();
                    Robot.launchAllSwitch++;
                }
                break;
            case 4:
                if (!spindex.isBusy() && flywheel.getVelocity() > flywheelVelocity - 100) {
                    transfer.setPosition(Robot.transfer3);
                    if (Robot.numberOfBalls > 0) {
                        Robot.numberOfBalls--;
                    }
                    runtime.reset();
                    Robot.launchAllSwitch++;
                }
                break;
            case 5:
                if (runtime.milliseconds() > Robot.transferOneTime) {
                    transfer.setPosition(Robot.transferRest);
                    runtime.reset();
                    Robot.launchAllSwitch++;
                }
                break;
            case 6:
                if (runtime.milliseconds() > 900) {
                    Robot.launchAllSwitch = 0;
                }
                queue.clear();
                break;
        }
        if (gamepad2.bWasReleased()) {
            if (launchAllSwitch != 2 && launchAllSwitch != 4) {
                launchAllSwitch = 0;
                transfer.setPosition(transferRest);
            } else {
                launchAllSwitch = 0;
                ejectSwitch = 1;
            }
        }

        //Eject current ball
        switch (Robot.ejectSwitch) {
            case 0:
                if (gamepad2.dpad_up) {
                    ejectSwitch++;
                }
                break;
            case 1:
                queue.clear();
                hood.setPosition(hoodMax);
                flywheel.setVelocity(0);
                runtime.reset();
                Robot.ejectSwitch++;
                break;
            case 2:
                if (runtime.milliseconds() > 100 && !spindex.isBusy()) {
                    transfer.setPosition(Robot.transfer3);
                    if (Robot.numberOfBalls > 0) {
                        Robot.numberOfBalls--;
                    }
                    runtime.reset();
                    Robot.ejectSwitch++;
                }
                break;
            case 3:
                if (runtime.milliseconds() > Robot.transferOneTime) {
                    transfer.setPosition(Robot.transferRest);
                    Robot.ejectSwitch = 0;
                }
                break;
        }

        //Kickstand
        switch (Robot.kickstandSwitch) {
            case 0:
                if (gamepad2.dpadDownWasPressed()) {
                    leftKickstand.setPosition(Robot.lkLowered);
                    rightKickstand.setPosition(Robot.rkLowered);
                    flywheelOn = false;
                    Robot.kickstandSwitch++;
                }
                break;
            case 1:
                if (gamepad2.dpadDownWasPressed()) {
                    leftKickstand.setPosition(Robot.lkRaised);
                    rightKickstand.setPosition(Robot.rkRaised);
                    flywheelOn = true;
                    Robot.kickstandSwitch = 0;
                }
                break;
        }

        telemetryM.debug("ABS Spindex: " + absSpindex.getVoltage());
        telemetryM.debug("ABS Turret: " + absTurret.getVoltage());

//        if (gamepad2.xWasPressed()){
//            queuePurple();
//        }
//        if (gamepad2.yWasPressed()) {
//            queueGreen();
//        }

        // Telemetry
//        telemetryM.debug("current", currentTurretAngle);
//        telemetryM.debug("delta", deltaTargetAngle);
//        telemetryM.debug("total", totalTurretAngle);
//        telemetryM.debug("encoder", turretEncoder);
//
////        telemetry.addData("# of balls", Robot.numberOfBalls);
//        telemetry.addData("hood angle", Robot.hoodAngle);
//        telemetry.addData("flywheel velocity", flywheelVelocity);
//        telemetry.addData("turret distance to goal", turretDistanceToGoal);
//        telemetry.addData("total turret angle", totalTurretAngle);
//        telemetry.addData("turret encoder", turretEncoder);
////        telemetry.addData("llresult", llResult.isValid());
//        telemetry.addData("distance", RBcolor.getDistance(DistanceUnit.CM));
//        telemetry.addData("Color", RBcolor.getNormalizedColors());
//        telemetry.addData("RBG", RBcolor.green());
//        telemetry.addData("eject", ejectSwitch);
//        telemetry.addData("up", gamepad2.dpad_up);
//
//        telemetryM.debug(turretEncoder);
//        telemetryM.debug(turret.getCurrentPosition());
//
//        telemetryM.debug(flywheel.getVelocity());
//
//        telemetryM.debug(currentTurretAngle);
//
//        telemetryM.debug("distance", turretDistanceToGoal);
        while (loopTime.milliseconds() < 30) {

        }
        telemetryM.addData("loop time", loopTime.milliseconds());
        telemetryM.update();

        loopTime.reset();
    }

    public static void spin1CCW(){
        numberOfSpins++;
        spindexPose = spindexPose + 1333;
        if (numberOfSpins == 0) {
            spindexPose = 0;
        } else if (numberOfSpins % 3 == 0) {
            spindexPose++;
        }
        spindex.setTargetPosition(spindexPose);
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindex.setPower(0.8);
    }
    public static void spin2CCW(){
        numberOfSpins = numberOfSpins + 2;
        spindexPose = spindexPose + 2666;
        if (numberOfSpins == 0) {
            spindexPose = 0;
        } else if (numberOfSpins % 3 == 0) {
            spindexPose++;
        }
        spindex.setTargetPosition(spindexPose);
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindex.setPower(1.0);
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
        spindex.setPower(0.8);
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
//    public void backStatus() {
//        if ((BBcolor.getDistance(DistanceUnit.CM) < 5) || (BTcolor.getDistance(DistanceUnit.CM) < 5)) {
//            if ((hue3 >= 90 && hue3 <= 150) || (hue4 >= 90 && hue4 <= 150)) {
//                // Green
//            } else {
//                // Purple
//            }
//        }
//    }
//
//    public void leftStatus() {
//        if ((LBcolor.getDistance(DistanceUnit.CM) < 5) || (LTcolor.getDistance(DistanceUnit.CM) < 5)) {
//            if ((hue1 >= 90 && hue1 <= 150) || (hue2 >= 90 && hue2 <= 150)) {
//                // Green
//            } else {
//                // Purple
//            }
//        }
//    }
//
//    public void rightStatus() {
//        if ((RBcolor.getDistance(DistanceUnit.CM) < 5) || (RTcolor.getDistance(DistanceUnit.CM) < 5)) {
//            if ((hue5 >= 90 && hue5 <= 150) || (hue6 >= 90 && hue6 <= 150)) {
//                // Green
//            } else {
//                // Purple
//            }
//        }
//    }
//
//    public void queueGreen() {
//        queue.add("green");
//        if ((RBcolor.getDistance(DistanceUnit.CM) < 5) || (RTcolor.getDistance(DistanceUnit.CM) < 5)) {
//            if ((hue5 >= 90 && hue5 <= 150) || (hue6 >= 90 && hue6 <= 150)) {
//                telemetry.addLine("Green in Right");
//                telemetry.addData("Queue", queue);
//            }
//        } else if ((LBcolor.getDistance(DistanceUnit.CM) < 5) || (LTcolor.getDistance(DistanceUnit.CM) < 5)) {
//            if ((hue5 >= 90 && hue5 <= 150) || (hue6 >= 90 && hue6 <= 150)) {
//                spin1CCW();
//                telemetry.addLine("Green in Left");
//                telemetry.addData("Queue", queue);
//            }
//        } else if ((BBcolor.getDistance(DistanceUnit.CM) < 5) || (BTcolor.getDistance(DistanceUnit.CM) < 5)) {
//            if ((hue5 >= 90 && hue5 <= 150) || (hue6 >= 90 && hue6 <= 150)) {
//                spin1CW();
//                telemetry.addLine("Green in Back");
//                telemetry.addData("Queue", queue);
//            }
//        }
//        telemetry.update();
//    }
//
//    public void queuePurple() {
//        queue.add("purple");
//        if ((RBcolor.getDistance(DistanceUnit.CM) < 5) || (RTcolor.getDistance(DistanceUnit.CM) < 5)) {
//            if ((hue5 >= 165 && hue5 <= 240) || (hue6 >= 165 && hue6 <= 240)) {
//                telemetry.addLine("Purple in Right");
//                telemetry.addData("Queue", queue);
//            }
//        } else if ((LBcolor.getDistance(DistanceUnit.CM) < 5) || (LTcolor.getDistance(DistanceUnit.CM) < 5)) {
//            if ((hue5 >= 165 && hue5 <= 240) || (hue6 >= 165 && hue6 <= 240)) {
//                spin1CCW();
//                telemetry.addLine("Purple in Left");
//                telemetry.addData("Queue", queue);
//            }
//        } else if ((BBcolor.getDistance(DistanceUnit.CM) < 5) || (BTcolor.getDistance(DistanceUnit.CM) < 5)) {
//            if ((hue5 >= 165 && hue5 <= 240) || (hue6 >= 165 && hue6 <= 240)) {
//                spin1CW();
//                telemetry.addLine("Purple in Back");
//                telemetry.addData("Queue", queue);
//            }
//        }
//        telemetry.update();

//    }

    public void killMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}