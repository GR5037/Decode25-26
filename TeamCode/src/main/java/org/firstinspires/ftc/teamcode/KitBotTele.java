/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "KitBotTele", group = "StarterBot")

public class KitBotTele extends OpMode {
    Robot robot = new Robot();
    private boolean flywheelOn = false;
    private boolean lbPressed = false;

    double targetVelocity = 0;
    boolean dpadUpPressed = false;
    boolean dpadDownPressed = false;

    @Override
    public void init() {
        robot.init(hardwareMap);



        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);

//        robot.updateState();

//        if (gamepad1.y) {
//           robot.startLauncher();
//        } else if (gamepad1.b) {
//            robot.stopLauncher();
//        }


        if (gamepad1.left_bumper && !lbPressed){
            if (flywheelOn){
                robot.launcher.setVelocity(0.0);
                flywheelOn = false;
            } else {
                robot.launcher.setVelocity(1500.0);
                flywheelOn = true;
                dpadUpPressed = gamepad1.dpad_up;
                dpadDownPressed = gamepad1.dpad_down;
                if (gamepad1.dpad_down && !dpadDownPressed) {
                    targetVelocity -= 100;
                }
                if (gamepad1.dpad_up && !dpadUpPressed) {
                    targetVelocity += 100;
                }
            }
            lbPressed = true;
        }

        if (!gamepad1.left_bumper && lbPressed) {
            lbPressed = false;
        }

//        if (gamepad1.left_bumper){
//            robot.launcherStart();
//        } else {
//            robot.launcherStop();
//        }

        if (gamepad1.right_trigger > 0.5 && flywheelOn){
            robot.feederStart();
        } else if (gamepad1.right_trigger < 0.5){
            robot.feederStop();
        }

//        if (gamepad1.dpad_up && !dpadUpPressed) {
//            targetVelocity += 100;
//        }
//        dpadUpPressed = gamepad1.dpad_up; // Update the button's state for the next loop
//
//        // Logic to decrease velocity on a single button press
//        if (gamepad1.dpad_down && !dpadDownPressed) {
//            targetVelocity -= 100;
//        }
//        dpadDownPressed = gamepad1.dpad_down; // Update the button's state for the next loop
//
//        // Set the motor's target velocity
//        robot.launcher.setVelocity(targetVelocity);

//        if (gamepad1.a) {
//            robot.startFeeder();
//        } else if (gamepad1.x) {
//            robot.stopFeeder();
//        }

        telemetry.addData("State", robot.getState());
        telemetry.addData("motorSpeed", robot.launcher.getVelocity());
        telemetry.addData("Launcher wheel encoder reading", robot.launcher.getVelocity());
        telemetry.update();
    }


    @Override
    public void stop() {
    }


}