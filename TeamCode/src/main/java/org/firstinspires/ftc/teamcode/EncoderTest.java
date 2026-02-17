package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Disabled
@TeleOp(name = "EncoderTest", group = "Test")
public class EncoderTest extends OpMode {


    private int startPosition = 0;

    @Override
    public void init() {

        telemetry.addData("Data", Robot.data);
        telemetry.update();
    }

    @Override
    public void loop() {


    }
}
