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

import java.math.BigDecimal;


@Configurable
public class Robot {
    static double xPoseFromAuto = 0;
    static double yPoseFromAuto = 0;
    static double headingFromAuto = 0;
    static double flyP = 200;
    static double flyI = 0;
    static double flyD = 0;
    static double flyF = 13.1;
    static double dexP = 1.7;
    static double dexI = 0.0;
    static double dexD = 0.05;
    static double dexF = 5.5;
    static double turP = 0.6;
    static double turI = 0.001;
    static double turD = 0.000001;
    static double turF = 4;
    static double lkRaised = 0.0;
    static double lkLowered = 0.81;
    static double rkRaised = 1.0;
    static double rkLowered = 0.25;
    static double transferRest = 0.488;
    static double transfer1 = 0.408;
    static double transfer2 = 0.336;
    static double transfer3 = 0.262;
    static int transferOneTime = 380;
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
    static int xBlueGoal = 15;
    static int xRedGoal = 133;
    static int yGoal = 132;
    static double velocityA = 0.0110641;
    static double velocityB = 6.11461;
    static double velocityC = 1380.89071;
    static double velocity1;
    static double velocity2;
    static int velocityMax = 2400;
    static int velocityMin = 1500;
    static int flywheelVelocity;
    static double angleA = 6.84614E-7;
    static double angleB = -0.00327091;
    static double angleC = 4.16634;
    static double angle1;
    static double angle2;
    static double hoodMax = 0.0;
    static double hoodMin = 0.73;
    static double hoodAngle;
    static double turretOffsetFromAuto = 0;
    static boolean teleIsRed = true;
    static double sP = 0.0006, sI = 0, sD = 0.03, sF = 0.1, spindexAllowedError = 50;
    static double tP = 0.001, tI = 0, tD = 0.05, tF = 0.04, turretAllowedError = 50;
    static double turretCompConstant = -100;
    static double xGoalCompConstant = -0.5;
    static double yGoalCompConstant = -0.5;
    public static String data = "Default data";

}



