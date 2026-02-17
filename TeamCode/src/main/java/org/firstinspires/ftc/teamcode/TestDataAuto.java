package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.KitBot.KitBotRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "TestDataAuto", group = "Auto", preselectTeleOp= "EncoderTest")

public class TestDataAuto extends OpMode {

//    KitBotRobot robot = new KitBotRobot();


    public void init() {
        telemetry.addData("Original data", Robot.data);
        telemetry.update();

//        robot.init(hardwareMap);



    }

    public void loop() {

        Robot.data = "Data has been CHANGED";

    }

}