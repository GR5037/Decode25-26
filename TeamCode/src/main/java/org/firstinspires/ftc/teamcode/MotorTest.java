package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp(name="MotorTest", group="Robot")
@Disabled
public class MotorTest extends LinearOpMode {
    private DcMotorEx motor;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;


    // These variables can be tuned from the FTC Dashboard
    public static double P = 200.0;  //10 //1000 far
    public static double I = 0.0;   //6
    public static double D = 0.0;    //1
    public static double F = 13.1;   //0 //33.1 far

    public String status;

    double targetVelocity = 0;
    boolean dpadUpPressed = false;
    boolean dpadDownPressed = false;


    @Override
    public void runOpMode() {

//        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
//        // Drive practice robot:
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//
//        // Regular robot:
////        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
////        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftFrontDrive.setZeroPowerBehavior(BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(BRAKE);
//        leftBackDrive.setZeroPowerBehavior(BRAKE);
//        rightBackDrive.setZeroPowerBehavior(BRAKE);



        // Get the motor from the hardware map
        motor = hardwareMap.get(DcMotorEx.class, "launcher");

        // Set the motor run mode to use encoders for velocity control
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorEx.Direction.FORWARD);

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
//            mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

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



            if (1960 <= motor_vel && motor_vel <= 2040) {
                status = "Normal";
            } else {
                status = "Disrupted";
                double disruptedTime = getRuntime();
                telemetry.addData("Time of disruption", disruptedTime);
            }

            double power = 0;

            if (gamepad1.dpad_up) {
                power += 0.1;
            }
            if (gamepad1.dpad_down) {
                power -= 0.1;
            }

            motor.setPower(power);

            // Display telemetry for monitoring
            telemetry.addData("Power", power);
//            telemetry.addData("Target Velocity", TARGET_VELOCITY);
//            telemetry.addData("Status", status);
            telemetry.update();
        }
    }
    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }
}
