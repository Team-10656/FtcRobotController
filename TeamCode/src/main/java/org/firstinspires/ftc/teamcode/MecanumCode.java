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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum Code", group="Linear Opmode")
//@Disabled
public class MecanumCode extends LinearOpMode {

    // Creates all the motor and servo variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private DcMotor leftFront = null;
    private DcMotor flywheel = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private DcMotor armOne = null;
    private DcMotor armTwo = null;

//    DigitalChannel armTest;

    static final double DEFAULT_SPEED = 1;
    static final double PRECISION_SPEED = 0.5;
    static final double ARM_SPEED = 0.8;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Defines all the names of the motors and servos
        leftRear  = hardwareMap.get(DcMotor.class, "left_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        armOne = hardwareMap.get(DcMotor.class, "arm_one");
        armTwo = hardwareMap.get(DcMotor.class, "arm_two");
//        armTest = hardwareMap.get(DigitalChannel.class, "arm_test");

        // Sets the direction of all the motors and servos
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        armOne.setDirection(DcMotor.Direction.FORWARD);
        armTwo.setDirection(DcMotor.Direction.REVERSE);

//        armTest.setMode(DigitalChannel.Mode.INPUT);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftRearPower;
            double rightFrontPower;
            double rightRearPower;
            double leftFrontPower;
            double flyWheelPower;
            double clawPosition;
            double armPower;

            // Defines the variables for the parabolic drive
            double driveCurve = 0.33;
            double driveDeadzone = 0;
            double driveParabola = 0;
            double strafeParabola = 0;
            double turnParabola = 0;

            // Checks if the joystick is in the deadzone or not.
            // If its not, it will multiply the joystick position by ten,
            // square it, and then multiply the curve by it
            if (gamepad1.left_stick_y < driveDeadzone && gamepad1.left_stick_y > -driveDeadzone) {
                driveParabola = 0;
            } else if (gamepad1.left_stick_y < 0) {
                driveParabola = -driveCurve * ((gamepad1.left_stick_y * 10) * (gamepad1.left_stick_y * 10));
            } else if (gamepad1.left_stick_y > 0) {
                driveParabola = driveCurve * ((gamepad1.left_stick_y * 10) * (gamepad1.left_stick_y * 10));
            }

            if (gamepad1.left_stick_x < driveDeadzone && gamepad1.left_stick_x > -driveDeadzone) {
                strafeParabola = 0;
            } else if (gamepad1.left_stick_x < 0) {
                strafeParabola = -driveCurve * ((gamepad1.left_stick_x * 10) * (gamepad1.left_stick_x * 10));
            } else if (gamepad1.left_stick_x > 0) {
                strafeParabola = driveCurve * ((gamepad1.left_stick_x * 10) * (gamepad1.left_stick_x * 10));
            }

            if (gamepad1.right_stick_x < driveDeadzone && gamepad1.right_stick_x > -driveDeadzone) {
                turnParabola = 0;
            } else if (gamepad1.right_stick_x < 0) {
                turnParabola = -driveCurve * ((gamepad1.right_stick_x * 10) * (gamepad1.right_stick_x * 10));
            } else if (gamepad1.right_stick_x > 0) {
                turnParabola = driveCurve * ((gamepad1.right_stick_x * 10) * (gamepad1.right_stick_x * 10));
            }

            // turns the parabolic values into values the motors can use
            double drive = driveParabola / 10;
            double strafe = -strafeParabola / 10;
            double turn = turnParabola / 10;

            // Sets the power that the motors will get and also tests to see if it should move at half speed or not
            if (drive != 0 && gamepad1.left_trigger != 0 || turn != 0 && gamepad1.left_trigger != 0 || strafe != 0 && gamepad1.left_trigger != 0) {
                turn = gamepad1.right_stick_x / 3;
                leftFrontPower = Range.clip(strafe + drive - turn, -PRECISION_SPEED, PRECISION_SPEED);
                leftRearPower = Range.clip(strafe - drive + turn, -PRECISION_SPEED, PRECISION_SPEED);
                rightRearPower = Range.clip(strafe + drive + turn, -PRECISION_SPEED, PRECISION_SPEED);
                rightFrontPower = Range.clip(strafe - drive - turn, -PRECISION_SPEED, PRECISION_SPEED);
            } else {
                leftFrontPower = Range.clip(strafe + drive - turn, -DEFAULT_SPEED, DEFAULT_SPEED);
                leftRearPower = Range.clip(strafe - drive + turn, -DEFAULT_SPEED, DEFAULT_SPEED);
                rightRearPower = Range.clip(strafe + drive + turn, -DEFAULT_SPEED, DEFAULT_SPEED);
                rightFrontPower = Range.clip(strafe - drive - turn, -DEFAULT_SPEED, DEFAULT_SPEED);
            }

            // Fly wheel code, sets the value of the fly wheel according to the button pressed
            if (gamepad2.right_bumper) {
                flyWheelPower = 1;
            } else if (gamepad2.left_bumper) {
                flyWheelPower = -1;
            } else {
                flyWheelPower = 0;
            }


            if (gamepad2.right_trigger != 0) {
                clawPosition = gamepad2.right_trigger;
            } else {
                clawPosition = 0;
            }

            double armCurve = 0.33;
            double armDeadzone = 0;
            double armParabola = 0;

            if (gamepad2.right_stick_y < armDeadzone && gamepad2.right_stick_y > -armDeadzone) {
                armParabola = 0;
            } else if (gamepad2.right_stick_y < 0) {
                armParabola = -armCurve * ((gamepad2.right_stick_y * 100) * (gamepad2.right_stick_y * 100));
            } else if (gamepad2.right_stick_y > 0) {
                armParabola = armCurve * ((gamepad2.right_stick_y * 100) * (gamepad2.right_stick_y * 100));
            }

            armParabola = (((ARM_SPEED / 100) * armParabola) / 100);
//            if (armTest.getState()) {
            if (armParabola > 0.01 || armParabola < -0.01) {
                armPower = armParabola;
            } else {
                armPower = -0.0005;
            }
//                telemetry.addData("Digital Touch", "Is Not Pressed");
//            } else {
//                armPower = 0;
//                telemetry.addData("Digital Touch", "Is Pressed");
//            }

            // Sets the power of the motors and servos
            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);
            flywheel.setPower(flyWheelPower);
            leftClaw.setPosition(clawPosition);
            rightClaw.setPosition(clawPosition);
            armOne.setPower(armPower);
            armTwo.setPower(armPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left rear (%.2f), right rear (%.2f), left front (%.2f), right front (%.2f)"
                    , leftRearPower, rightRearPower, leftFrontPower, rightFrontPower);
            telemetry.addData("Motors", "flywheel (%.2f), claw (%.2f), arm (%.2f)", flyWheelPower, clawPosition, armPower);
            telemetry.update();
        }
    }
}