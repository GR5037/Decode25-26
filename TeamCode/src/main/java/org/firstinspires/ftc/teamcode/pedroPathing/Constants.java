package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.52544)
            .forwardZeroPowerAcceleration(-31.515593658598892)
            .lateralZeroPowerAcceleration(-58.830920754537615)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.044, 0, 0.0, 0.023))
            .headingPIDFCoefficients(new PIDFCoefficients(0.46,0,0.002,0.023))
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
            .forwardPodY(-7.77854)
            .strafePodX(8.0022)
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
}



//
//public class Constants {
//    public static FollowerConstants followerConstants = new FollowerConstants()
//            .mass(11.3398)
//            .forwardZeroPowerAcceleration(-33.81)
//            .lateralZeroPowerAcceleration(-62.34)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.001, 0.023))
//            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0, 0.025))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.6,0.0,0.05,0.6,0.025))
//            .centripetalScaling(.05)
//            ;
//
//
//
//
//    public static MecanumConstants driveConstants = new MecanumConstants()
//            .maxPower(1)
//            .rightFrontMotorName("fr")
//            .rightRearMotorName("br")
//            .leftFrontMotorName("fl")
//            .leftRearMotorName("bl")
//            .leftFrontMotorDirection(DcMotor.Direction.FORWARD)
//            .leftRearMotorDirection(DcMotor.Direction.FORWARD)
//            .rightFrontMotorDirection(DcMotor.Direction.REVERSE)
//            .rightRearMotorDirection(DcMotor.Direction.REVERSE)
//            .xVelocity(63.91)
//            .yVelocity(40.44)
//
//            ;
//
//    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .forwardPodY(-4.2)
//            .strafePodX(-2)
//            .distanceUnit(DistanceUnit.INCH)
//            .hardwareMapName("pinpoint")
//            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
//            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
//
//
//    public static PathConstraints pathConstraints = new PathConstraints(
//            0.99,
//            100,
//            1,
//            1);
//
//
//
//
//    public static Follower createFollower(HardwareMap hardwareMap) {
//        return new FollowerBuilder(followerConstants, hardwareMap)
//                .pathConstraints(pathConstraints)
//                .mecanumDrivetrain(driveConstants)
//                .pinpointLocalizer(localizerConstants)
//                .build();
//    }
//}
