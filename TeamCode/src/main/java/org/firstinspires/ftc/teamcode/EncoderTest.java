package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Encoder Ticks Per Rev Test", group = "Test")
public class EncoderTest extends OpMode {

    private DcMotorEx motor;
    private TouchSensor gateSwitch;

    private int startPosition = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor"); // CHANGE NAME IF NEEDED

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready");
        telemetry.addLine("Press A to reset encoder");
        telemetry.addLine("Rotate motor 1 full revolution");
        telemetry.addLine("Press B to read ticks");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Reset encoder
        if (gamepad1.a) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            startPosition = 0;
        }

        // Capture starting position
        if (gamepad1.x) {
            startPosition = motor.getCurrentPosition();
        }

        // Read ticks after 1 full revolution
        if (gamepad1.b) {
            int currentPosition = motor.getCurrentPosition();
            int ticksPerRev = Math.abs(currentPosition - startPosition);

            telemetry.addData("Ticks Per Revolution", ticksPerRev);
        }
        telemetry.addData("switch", gateSwitch.isPressed());
        telemetry.addData("Current Encoder", motor.getCurrentPosition());
        telemetry.update();
    }
}
