package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.os.SystemClock;

import androidx.core.math.MathUtils;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.ABSEncoder;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
public class Robot {

    static double flyP = 200;
    static double flyI = 0;
    static double flyD = 0;
    static double flyF = 13.1;
    static double dexP = 0.7;
    static double dexI = 0.01;
    static double dexD = 0.00001;
    static double dexF = 6;
    static double turP = 2.5;
    static double turI = 0.0;
    static double turD = 0.0;
    static double turF = 1.0;
    static double lkRaised = 0.0;
    static double lkLowered = 0.8;
    static double rkRaised = 1.0;
    static double rkLowered = 0.2;
    static double transferRest = 0.489;
    static double transfer1 = 0.414;
    static double transfer2 = 0.342;
    static double transfer3 = 0.269;
    static int transferOneTime = 270;
    static int spindexPose;
    static int numberOfSpins = 0;
    static int numberOfBalls = 0;
    static int ejectSwitch = 0;
    static int kickstandSwitch = 0;
    static int launchOneSwitch = 0;
    static int launchAllSwitch = 0;
    static int gateSwitch = 0;
    static boolean flywheelOn = false;
    static double xTurretPose;
    static double yTurretPose;
    static double turretDistanceToGoal;
    static int xBlueGoal = 11;
    static int xRedGoal = 133;
    static int yGoal = 136;
    static double velocityA = -0.011707;
    static double velocityB = 8.7054;
    static double velocityC = 1341.44834;
    static double velocity1;
    static double velocity2;
    static int velocityMax = 2400;
    static int velocityMin = 1500;
    static int flywheelVelocity;
    static double angleA = 0.0000180208;
    static double angleB = -0.00838942;
    static double angleC = 0.928578;
    static double angle1;
    static double angle2;
    static double hoodMax = 0.0;
    static double hoodMin = 0.73;
    static double hoodAngle;



//    private Limelight3A limelight;

    public static String data = "Default data";

    public static FollowerConstants followerConstants = new FollowerConstants() // All to change
            .mass(9.52544)
            .forwardZeroPowerAcceleration(-31.515593658598892)
            .lateralZeroPowerAcceleration(-58.830920754537615)
            .translationalPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.044, 0, 0.0, 0.023))
            .headingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.46, 0, 0.002, 0.023))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04, 0, 0.0001, 0.6, 0.023))
            .centripetalScaling(0.0005);


    public static MecanumConstants driveConstants = new MecanumConstants() // All to change
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


    public static PinpointConstants localizerConstants = new PinpointConstants() // All to change
            .forwardPodY(7.779)
            .strafePodX(8.0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1.4,
            1); // All to change


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build(); // All to change
    }

    public static void init(HardwareMap hardwareMap) {


//        follower = Constants.createFollower(hardwareMap);
//        follower.update();
    }
}



