package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp(name="WithController", group="Robot")
public class MotorTest extends LinearOpMode {
    private DcMotorEx motor;

    // These variables can be tuned from the FTC Dashboard
    public static double P = 10.0;
    public static double I = 6.0;
    public static double D = 1.0;
    public static double F = 0.0;

    double targetVelocity = 0;
    boolean dpadUpPressed = false;
    boolean dpadDownPressed = false;


    @Override
    public void runOpMode() {

        // Get the motor from the hardware map
        motor = hardwareMap.get(DcMotorEx.class, "launcher");

        // Set the motor run mode to use encoders for velocity control
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorEx.Direction.REVERSE);

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Apply the PIDF coefficients from the dashboard
            PIDFCoefficients newPIDF = new PIDFCoefficients(P, I, D, F);
            motor.setVelocityPIDFCoefficients(newPIDF.p, newPIDF.i, newPIDF.d, newPIDF.f);

            if (gamepad1.dpad_up && !dpadUpPressed) {
                targetVelocity += 100;
            }
            dpadUpPressed = gamepad1.dpad_up; // Update the button's state for the next loop

            // Logic to decrease velocity on a single button press
            if (gamepad1.dpad_down && !dpadDownPressed) {
                targetVelocity -= 100;
            }
            dpadDownPressed = gamepad1.dpad_down; // Update the button's state for the next loop

            // Set the motor's target velocity
            motor.setVelocity(targetVelocity);

            // Get the current velocity for telemetry
            double motor_vel = motor.getVelocity();

            // Display telemetry for monitoring
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Actual Velocity", motor_vel);
            telemetry.update();



//            if (1960 <= motor_vel && motor_vel <= 2040) {
//                status = "Normal";
//            } else {
//                status = "Disrupted";
//                double disruptedTime = getRuntime();
//                telemetry.addData("Time of disruption", disruptedTime);
//            }

//            double power = 0;
//
//            if (gamepad1.dpad_up) {
//                power += 0.1;
//            }
//            if (gamepad1.dpad_down) {
//                power -= 0.1;
//            }
//
//            motor.setPower(power);
//
//            // Display telemetry for monitoring
//            telemetry.addData("Power", power);
////            telemetry.addData("Target Velocity", TARGET_VELOCITY);
////            telemetry.addData("Status", status);
//            telemetry.update();
        }
    }
}
