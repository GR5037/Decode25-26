package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class limelightTest extends OpMode {

    private Limelight3A limelight;
    private DcMotorEx turretMotor;
    private IMU imu;

    public static double P = 10.0;
    public static double I = 6.0;
    public static double D = 1.0;
    public static double F = 0.0;

    public static  double kPTurret = 0.02; // 0.03
    public static double kI_TURRET = 0.0; // 0.0005
    public static double kD_TURRET = 0.001;   // 0.085
    private double previousTx = 0.0;
    private double integral = 0.0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        PIDFCoefficients newPIDF = new PIDFCoefficients(P, I, D, F);
        turretMotor.setVelocityPIDFCoefficients(newPIDF.p, newPIDF.i, newPIDF.d, newPIDF.f);

        limelight.pipelineSwitch(0);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start() {
        limelight.start();
    }


    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight.getLatestResult();

        if (llResult !=  null && llResult.isValid()) {
            double tx = llResult.getTx();
            double ty = llResult.getTy();
            double coverage = llResult.getTa();

            telemetry.addData("Tx", tx);
            telemetry.addData("Ty", ty);
            telemetry.addData("Coverage %", coverage);

            double pTerm = tx * kPTurret;

            if (Math.abs(tx) < 20) {
                integral += tx;
            } else {
                // Reset integral to prevent huge buildup
                integral = 0;
            }
            double iTerm = integral * kI_TURRET;


            double derivative = tx - previousTx;
            double dTerm = derivative * kD_TURRET;

            previousTx = tx;

            double turretPower = pTerm + iTerm + dTerm;

            turretPower = Math.max(Math.min(turretPower, 0.5), -0.5);
            turretMotor.setPower(turretPower);
            telemetry.addData("P", pTerm);
            telemetry.addData("I", iTerm);
            telemetry.addData("D", dTerm);
            telemetry.addData("Turret Power", turretPower);
        } else {
            integral = 0;

            turretMotor.setPower(0);
            telemetry.addLine("No target detected");
        }
        telemetry.update();
    }
}
