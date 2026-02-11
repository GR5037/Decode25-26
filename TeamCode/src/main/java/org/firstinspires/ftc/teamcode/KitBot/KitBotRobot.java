package org.firstinspires.ftc.teamcode.KitBot;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.os.SystemClock;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public class KitBotRobot {
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx kickstand = null;
    private Limelight3A limelight;

    private final double FEED_TIME_SECONDS = 0.20;
    private final double STOP_SPEED = 0.0;
    private final double FULL_SPEED = 1.0;
    private final double LAUNCHER_TARGET_VELOCITY = 485;
    // 1125
    private final double LAUNCHER_MIN_VELOCITY = 150;
    // 1075
    public DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    ElapsedTime feederTimer = new ElapsedTime();

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.52544)
            .forwardZeroPowerAcceleration(-31.515593658598892)
            .lateralZeroPowerAcceleration(-58.830920754537615)
            .translationalPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.044, 0, 0.0, 0.023))
            .headingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.46,0,0.002,0.023))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04,0,0.0001,0.6,0.023))
            .centripetalScaling(0.0005)

            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(66.26776026928519)
            .yVelocity(54.264240895669296);



    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.5)
            .strafePodX(4.72440944882)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1.4,
            1);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState launchState;


    public void init(HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "fr");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "br");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        kickstand = hardwareMap.get(DcMotorEx.class, "kickstand");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
            // 150, 6, 1, 0
        // 300,0,0,10
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        launchState = LaunchState.IDLE;
        stopFeeder();
        stopLauncher();

    }

    public void drive(double forward, double strafe, double rotate){

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        double leftFrontPower = (forward + strafe + rotate) / denominator;
        double rightFrontPower = (forward - strafe - rotate) / denominator;
        double leftBackPower = (forward - strafe + rotate) / denominator;
        double rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }

    public void killMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void stopFeeder(){
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }

    public void startSlowFeeder() {
        leftFeeder.setPower(0.15);
        rightFeeder.setPower(0.15);
    }
    public void startFeeder(){
        leftFeeder.setPower(0.9);
        rightFeeder.setPower(0.9);
    }

    public void updateState(){
        switch (launchState) {
            case IDLE:
//                if (shotRequested) {
//                    launchState = LaunchState.SPIN_UP;
//                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
    public void startLauncher(){
        if (launchState == LaunchState.IDLE){
            launchState = LaunchState.SPIN_UP;
        }
    }

    public void shootArtifact() {
        startFeeder();
        launcherStart(1500);
        stopFeeder();
        SystemClock.sleep(2000);
        launcherStop();
    }
    public void stopLauncher(){
        stopFeeder();
        launcher.setVelocity(STOP_SPEED);
        launchState = LaunchState.IDLE;
    }

    public void launcherStart(double power){
        launcher.setVelocity(power);
    }

    public void launcherStop(){
        launcher.setVelocity(STOP_SPEED);
    }

    public void feederStart(){
        leftFeeder.setPower(FULL_SPEED);
        rightFeeder.setPower(FULL_SPEED);
    }

    public void feederStop(){
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }

    public String getState(){
        return launchState.toString();
    }

    public double getVelocity(){
        return launcher.getVelocity();
    }
}





