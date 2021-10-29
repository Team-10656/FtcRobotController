/* Copyright (c) 2017 FIRST. All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This will eventually be test code for Spherical Wheel Driving.
 * Majority of Spherical Code here in C++: https://github.com/XRobots/BallWheels/tree/main/Code/BallWheels02
 * Explanation of how it moves at 6:08: https://www.youtube.com/watch?v=zKLMCO0-How
 */

@TeleOp(name="Spherical Code", group="Linear Opmode")
@Disabled
public class SphericalWheelCode extends LinearOpMode {

    // Declare OpMode members.
    // The motor names are temporary for now
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorOne = null;
    private DcMotor motorTwo = null;
    private DcMotor motorThree = null;
    private DcMotor flyWheel = null;
    // If we use Jacob's clamp idea
    /*
    private Servo leftClamp = null;
    private Servo rightClamp = null;
    private Servo armGear = null; // If a servo works
    private DcMotor armGear = null; // If we need to use a motor
    */
    // If we use Josh's claw idea
    /*
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private DcMotor arm = null;
    private Servo arm = null; // If we want to use a servo
     */

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorOne  = hardwareMap.get(DcMotor.class, "motor_one");
        motorTwo = hardwareMap.get(DcMotor.class, "motor_two");
        motorThree = hardwareMap.get(DcMotor.class, "motor_three");
        flyWheel = hardwareMap.get(DcMotor.class, "fly_wheel");
        // If we use Jacob's Clamp idea:
        /*
        leftClamp = hardwareMap.get(Servo.class, "left_clamp");
        rightClamp = hardwareMap.get(Servo.class, "right_clamp");
        armGear = hardwareMap.get(/*DcMotor*//*Servo.class, "arm_gear");
         */
        // If we use Josh's Claw idea:
        /*
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw);
        arm = hardwareMap.get(/*Servo*//*DcMotor.class, "arm");
         */


        // The directions for the wheel motors are yet to be determined
        motorOne.setDirection(DcMotor.Direction.FORWARD);
        motorTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        motorThree.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setDirection(DcMotor.Direction.FORWARD);
        // If we use Jacob's clamp idea:
        /*
        leftClamp.setDirection(Servo.Direction.REVERSE);
        rightClamp.setDirection(Servo.Direction.FORWARD);
        armGear.setDirection(/*DcMotor*//*Servo.Direction.FORWARD);
         */
        //If we use Josh's claw idea:
        /*
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        arm.setDirection(/*Servo*//*DcMotor.Direction.FORWARD);
         */

        motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorThree.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorThree.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armGear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double onePower;
            double twoPower;
            double threePower;
            double flyWheelPower;
            // If we use Jacob's clamp idea:
            /*
            double clampPosition;
            double gearPosition;
            //double gearPower;
            */
            // If we use Josh's claw idea:
            /*
            double clawPosition
            double armPower
            //double armPosition
             */


            // Fly wheel code, sets the value of the fly wheel according to the button pressed
            if (gamepad2.right_trigger != 0) {
                flyWheelPower = 1;
            }
            else {
                flyWheelPower = 0;
            }

            // If we use Jacob's clamp idea:
/*
            if (gamepad2.dpad_right) {
                clampPosition = 1;
            } else if (gamepad2.dpad_left) {
                clampPosition = -1;
            } else {
                clampPosition = 0;
            }

            if (gamepad2.dpad_up) {
                gearPosition = 1;
                //gearPower = 1;
            } else if (gamepad2.dpad_down) {
                gearPosition = -1;
                //gearPower = -1;
            } else {
                gearPosition = 0;
                //gearPower = 0;
            }
 */

            //If we use Josh's claw idea
/*
            if (gamepad2.dpad_right) {
                clawPosition = 1;
            } else if (gamepad2.dpad_left) {
                clawPosition = -1;
            } else {
                clawPosition = 0;
            }

            if (gamepad2.dpad_up) {
                armPower = 1;
                //armPosition = 1;
            } else if (gamepad2.dpad_down) {
                armPower = -1;
                //armPosition = -1;
            } else {
                armPower = 0;
                //armPosition = 0;
            }
 */


            // Send calculated power to wheels
            motorOne.setPower(onePower);
            motorTwo.setPower(twoPower);
            motorThree.setPower(threePower);
            flyWheel.setPower(flyWheelPower);
            /*
            leftClamp.setPosition(clampPosition);
            rightClamp.setPosition(clampPosition);
            armGear.setPosition(gearPosition);
            //armGear.setPower(gearPower);
            */
            /*
            leftClaw.setPosition(clawPosition);
            rightClaw.setPosition(clawPosition);
            arm.setPower(armPower);
            //arm.setPosition(armPosition);
             */

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "one (%.2f), two (%.2f), three (%.2f)", onePower, twoPower, threePower);
            telemetry.addData("Motors", "fly wheel (%.2f)"/*, clamp (%.2f), arm gear (%.2f)"*//*, claw (%.2f), arm (%.2f)"*/
                    , flyWheelPower/*, clampPosition, gearPosition/*, gearPower*//*clawPosition, armPower/*, armPosition*/);
            telemetry.update();
            telemetry.update();
        }
    }
}
