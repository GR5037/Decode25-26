package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class limelightTest extends OpMode {
//    private DigitalChannel limitSwitch;
//
//    private NormalizedColorSensor colorSensor;
//    private DistanceSensor distanceSensor;

    private DcMotor motor;
    private AnalogInput encoder;

    static final double COUNTS_PER_REV = 4000;
    static final double VOLTS_TO_DEGREES = 360.0 / 3.3;
//    private DcMotor leftFrontDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightBackDrive = null;
//    double leftFrontPower;
//    double rightFrontPower;
//    double leftBackPower;
//    double rightBackPower;
//
//    private Limelight3A limelight;
//    private DcMotorEx turretMotor;
//    private IMU imu;
//
//    public static double P = 10.0;
//    public static double I = 6.0;
//    public static double D = 1.0;
//    public static double F = 0.0;
//
//    public static  double kPTurret = 0.02; // 0.03
//    public static double kI_TURRET = 0.0; // 0.0005
//    public static double kD_TURRET = 0.001;   // 0.085
//    private double previousTx = 0.0;
//    private double integral = 0.0;

    @Override
    public void init() {
        encoder = hardwareMap.get(AnalogInput.class, "encoder");
        motor = hardwareMap.get(DcMotor.class, "motor");



//        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSensor");
//        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
//
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
//
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        leftFrontDrive.setZeroPowerBehavior(BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(BRAKE);
//        leftBackDrive.setZeroPowerBehavior(BRAKE);
//        rightBackDrive.setZeroPowerBehavior(BRAKE);

        // limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");


//        PIDFCoefficients newPIDF = new PIDFCoefficients(P, I, D, F);
//        turretMotor.setVelocityPIDFCoefficients(newPIDF.p, newPIDF.i, newPIDF.d, newPIDF.f);
//
//        limelight.pipelineSwitch(0);
//
//        imu = hardwareMap.get(IMU.class, "imu");
//        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
//        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start() {

//        limelight.start();
    }


    @Override
    public void loop() {
        double currentVoltage = encoder.getVoltage();

        // Convert to degrees
        double currentDegrees = currentVoltage * VOLTS_TO_DEGREES;

        telemetry.addData("Encoder Voltage", currentVoltage);
        telemetry.addData("Encoder Degrees", currentDegrees);
        telemetry.update();

        // Example: Basic motor control based on position
        if (currentDegrees < 90) {
            motor.setPower(0.5);
        } else {
            motor.setPower(0);
        }


//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//
//        float hue = JavaUtil.colorToHue(colors.toColor());
//
//        // Read distance
//        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
//
//        // Telemetry to Driver Hub
//        telemetry.addData("Distance (cm)", "%.2f", distance);
//        telemetry.addData("Hue", "%.3f", hue);
//
//        if (limitSwitch.getState()) {
//            telemetry.addData("Sensor State", "Not Pressed/Magnet Out of Range");
//        }
//        else {
//            telemetry.addData("Sensor State", "PRESSED/Magnet In Range");
//        }
//
//        if (hue > 130 && hue < 160) {
//            telemetry.addData("Color", "Green");
//        } else if (hue > 230 && hue < 310) {
//            telemetry.addData("Color", "Purple");
//        }
//
//        telemetry.update();

//        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
//
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        limelight.updateRobotOrientation(orientation.getYaw());
//
//        LLResult llResult = limelight.getLatestResult();
//
//        if (llResult !=  null && llResult.isValid()) {
//            double tx = llResult.getTx();
//            double coverage = llResult.getTa();
//
//            telemetry.addData("Tx", tx);
//            telemetry.addData("Coverage %", coverage);
//
//            double pTerm = tx * kPTurret;
//
//            if (Math.abs(tx) < 20) {
//                integral += tx;
//            } else {
//                // Reset integral to prevent huge buildup
//                integral = 0;
//            }
//            double iTerm = integral * kI_TURRET;
//
//
//            double derivative = tx - previousTx;
//            double dTerm = derivative * kD_TURRET;
//
//            previousTx = tx;
//
//            double turretPower = pTerm + iTerm + dTerm;
//
//            turretPower = Math.max(Math.min(turretPower, 0.5), -0.5);
//            turretMotor.setPower(turretPower);
//            telemetry.addData("P", pTerm);
//            telemetry.addData("I", iTerm);
//            telemetry.addData("D", dTerm);
//            telemetry.addData("Turret Power", turretPower);
//        } else {
//            integral = 0;
//
//            turretMotor.setPower(0);
//            telemetry.addLine("No target detected");
//        }
//        telemetry.update();
    }

//    void mecanumDrive(double forward, double strafe, double rotate) {
//        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
//
//        leftFrontPower = (forward + strafe + rotate) / denominator;
//        rightFrontPower = (forward - strafe - rotate) / denominator;
//        leftBackPower = (forward - strafe + rotate) / denominator;
//        rightBackPower = (forward + strafe - rotate) / denominator;
//
//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);
//    }

    }
