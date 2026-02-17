package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;


import static org.firstinspires.ftc.teamcode.Robot.angleA;
import static org.firstinspires.ftc.teamcode.Robot.flywheelVelocity;
import static org.firstinspires.ftc.teamcode.Robot.numberOfSpins;
import static org.firstinspires.ftc.teamcode.Robot.spindexPose;
import static org.firstinspires.ftc.teamcode.Robot.turretDistanceToGoal;
import static org.firstinspires.ftc.teamcode.Robot.velocityA;
import static org.firstinspires.ftc.teamcode.Robot.velocityB;
import static org.firstinspires.ftc.teamcode.Robot.velocityC;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


//@Configurable
@TeleOp(name = "BlueTele", group = "Test")
public class BlueTele extends OpMode {
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
    static double leftFrontPower;
    static double rightFrontPower;
    static double leftBackPower;
    static double rightBackPower;
    Robot robot = new Robot();
    private Follower follower;
    private TelemetryManager telemetryM;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        robot = new Robot();
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Change
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Change
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Change
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD); // Change

        leftFrontDrive.setZeroPowerBehavior(FLOAT);
        rightFrontDrive.setZeroPowerBehavior(FLOAT);
        leftBackDrive.setZeroPowerBehavior(FLOAT);
        rightBackDrive.setZeroPowerBehavior(FLOAT);

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

        spindex.setDirection(DcMotorEx.Direction.FORWARD); // Change
        flywheel.setDirection(DcMotorEx.Direction.FORWARD); // Change
        intake.setDirection(DcMotorEx.Direction.REVERSE); // Change
        turret.setDirection(DcMotorEx.Direction.FORWARD); // Change

        spindex.setZeroPowerBehavior(BRAKE);
        flywheel.setZeroPowerBehavior(FLOAT);
        intake.setZeroPowerBehavior(BRAKE);
        turret.setZeroPowerBehavior(BRAKE);

        PIDFCoefficients flywheelpidfCoefficients = new PIDFCoefficients(Robot.flyP, Robot.flyI, Robot.flyD, Robot.flyF);
        PIDFCoefficients spindexpidfCoefficients = new PIDFCoefficients(1, 0.3, 0, 2);

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelpidfCoefficients);
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, spindexpidfCoefficients);
        spindex.setTargetPositionTolerance(10);

        gate = hardwareMap.get(DigitalChannel.class, "gate");
        gate.setMode(DigitalChannel.Mode.INPUT);
        ABSturret = hardwareMap.get(AnalogInput.class, "ABSturret");
        ABSspindex = hardwareMap.get(AnalogInput.class, "ABSspindex");
    }

    @Override
    public void start(){
        turret.setPower(0);
        hood.setPosition(0.3);
        rightKickstand.setPosition(Robot.rkRaised);
        leftKickstand.setPosition(Robot.lkRaised);
        transfer.setPosition(Robot.transferRest);

        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        //Relocalise (not cam yet)
        if (gamepad1.yWasPressed()) {
            follower.setPose(new Pose(7.54, 8.83,0));
        }

        //Calculations
        Robot.xTurretPose = follower.getPose().getX() + 2.7062 * (Math.cos(Math.toRadians(85.24)) + follower.getPose().getHeading());
        Robot.yTurretPose = follower.getPose().getY() + 2.7062 * (Math.sin(Math.toRadians(85.24)) + follower.getPose().getHeading());
        Robot.turretDistanceToGoal = Math.hypot(Robot.xTurretPose - Robot.xBlueGoal, Robot.yTurretPose - Robot.yGoal);

        Robot.velocity1 = Robot.velocityA * (Math.pow(Robot.turretDistanceToGoal , 2));
        Robot.velocity2 = Robot.velocityB * Robot.turretDistanceToGoal;
        Robot.flywheelVelocity = (int) (Robot.velocity1 + Robot.velocity2 + Robot.velocityC);
        if (Robot.flywheelVelocity > Robot.velocityMax) {
            Robot.flywheelVelocity = Robot.velocityMax;
        } else if (Robot.flywheelVelocity < Robot.velocityMin) {
            Robot.flywheelVelocity = Robot.velocityMin;
        }

        Robot.angle1 = Robot.angleA * (Math.pow(Robot.turretDistanceToGoal , 2));
        Robot.angle2 = Robot.angleB * Robot.turretDistanceToGoal;
        Robot.hoodAngle = (Robot.angle1 + Robot.angle2 + Robot.angleC);
        if (Robot.hoodAngle < Robot.hoodMax) {
            Robot.hoodAngle = Robot.hoodMax;
        } else if (Robot.hoodAngle > Robot.hoodMin) {
            Robot.hoodAngle = Robot.hoodMin;
        }

        hood.setPosition(Robot.hoodAngle);

        //Flywheel
        if (gamepad2.left_bumper) {
            Robot.flywheelOn = true;
//            PIDFCoefficients flywheelpidfCoefficients = new PIDFCoefficients(Robot.flyP, Robot.flyI, Robot.flyD, Robot.flyF);
//            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelpidfCoefficients);
        } else if (gamepad2.right_bumper) {
            Robot.flywheelOn = false;
        }
        if (Robot.flywheelOn) {
            flywheel.setVelocity(Robot.flywheelVelocity);
        } else {
            flywheel.setVelocity(0.0);
        }

        //Drive
        drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


        //Intake
        if (gamepad2.left_trigger > 0.1 && Robot.numberOfBalls < 3) {
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
                        Robot.numberOfBalls++;
                        Robot.gateSwitch++;
                    }
                break;
                case 2:
                    if (!spindex.isBusy()) {
                        Robot.gateSwitch = 0;
                    }
                break;
            }
        } else if (gamepad2.right_trigger > 0.1){
            intake.setPower(-gamepad2.right_trigger);
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
                    spin1CW();
                    Robot.launchOneSwitch++;
                }
            break;
            case 3:
                if (spindex.isBusy()) {
                    Robot.launchOneSwitch = 0;
                }
            break;
        }
        switch (Robot.launchAllSwitch) { //launch all balls
            case 0:
                if (gamepad2.b) {
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
                if (!spindex.isBusy()) {
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
                if (!spindex.isBusy()) {
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
            break;
        }



        //Eject current ball (not done)
        switch (Robot.ejectSwitch) {
            case 0:
                if (gamepad2.dpadUpWasPressed()) {
                    transfer.setPosition(Robot.transfer1);
                    if (Robot.numberOfBalls > 0) {
                        Robot.numberOfBalls--;
                    }
                    runtime.reset();
                    Robot.ejectSwitch++;
                }
            break;
            case 1:
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
                    Robot.kickstandSwitch++;
                }
            break;
            case 1:
                if (gamepad2.dpadDownWasPressed()) {
                    leftKickstand.setPosition(Robot.lkRaised);
                    rightKickstand.setPosition(Robot.rkRaised);
                    Robot.kickstandSwitch = 0;
                }
            break;
        }



        //Telemetry
        telemetry.addData("# of balls", Robot.numberOfBalls);
        telemetry.addData("hood andge", Robot.hoodAngle);
        telemetry.addData("velocity", flywheelVelocity);


        telemetry.update();

    }
    public static void drive(double forward, double strafe, double rotate){

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward - strafe - rotate) / denominator;
        rightFrontPower = (forward + strafe + rotate) / denominator;
        leftBackPower = (forward + strafe - rotate) / denominator;
        rightBackPower = (forward - strafe + rotate) / denominator;

        leftFrontDrive.setPower(-leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(-leftBackPower);
        rightBackDrive.setPower(-rightBackPower);
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
        spindex.setPower(1.0);
    }
    public void spin2CCW(){
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
        spindex.setPower(1.0);
    }
    public void spin2CW(){
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
    public void killMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}