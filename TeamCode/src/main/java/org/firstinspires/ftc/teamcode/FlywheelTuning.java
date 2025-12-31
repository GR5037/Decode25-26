package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelTuning extends OpMode {

    public DcMotorEx flywheelMotor;

    public double highVelocity = 2300;
    public double lowVelocity = 1600;
    double curTargetVelocity = highVelocity;

    double F = 13.5;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    double curVelocity = 0.0;

    double error = 0.0;


    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");


    }

    @Override
    public void loop() {

        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheelMotor.setVelocity(curTargetVelocity);

        curVelocity = flywheelMotor.getVelocity();
        error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "&.2f", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%,4f (D-Pad U/D", P);
        telemetry.addData("Tuning F", "%,4f (D-Pad L/R", F);
        telemetry.addData("Step Sizes", "%.4f (B Button)", stepSizes[stepIndex]);



    }
}
