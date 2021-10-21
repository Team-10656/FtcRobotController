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

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
public class MecanumCode extends LinearOpMode {

    // Creates all the motor and servo variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private DcMotor leftFront = null;
    private DcMotor rightFlyWheel = null;
    private DcMotor leftFlyWheel = null;
    private DcMotor intakeMotor = null;
    private DcMotor conveyorMotor = null;
    private Servo push = null;
    public double powerLevel = 0;
//    private Servo arm = null;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Defines all the names of the motors and servos
        leftRear  = hardwareMap.get(DcMotor.class, "left_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFlyWheel = hardwareMap.get(DcMotor.class, "right_fly_wheel");
        leftFlyWheel = hardwareMap.get(DcMotor.class, "left_fly_wheel");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyor");
        push = hardwareMap.get(Servo.class, "pusher");
//        arm = hardwareMap.get(Servo.class, "arm");


        // Sets the direction of all the motors and servos
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFlyWheel.setDirection(DcMotor.Direction.FORWARD);
        leftFlyWheel.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        conveyorMotor.setDirection(DcMotor.Direction.REVERSE);
        push.setDirection(Servo.Direction.REVERSE);
//        arm.setDirection(Servo.Direction.FORWARD);


        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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
            double rightFlyWheelPower;
            double leftFlyWheelPower;
            double intakeMotorPower;
            double conveyorMotorPower;
            double pushPosition;
//            double armPosition;

            // Defines the variables for the parabolic drive
            double curve = 0.33;
            double deadzone = 0;
            double driveParabola = 0;
            double strafeParabola = 0;
            double turnParabola = 0;

            // Checks if the joystick is in the deadzone or not.
            // If its not, it will multiply the joystick position by ten,
            // square it, and then multiply the curve by it
            if (gamepad1.left_stick_y < deadzone && gamepad1.left_stick_y > -deadzone) {
                driveParabola = 0;
            }  else if (gamepad1.left_stick_y < 0) {
                driveParabola = -curve * ((gamepad1.left_stick_y * 10) * (gamepad1.left_stick_y * 10));
            } else if (gamepad1.left_stick_y > 0) {
                driveParabola = curve * ((gamepad1.left_stick_y * 10) * (gamepad1.left_stick_y * 10));
            }

            if (gamepad1.left_stick_x < deadzone && gamepad1.left_stick_x > -deadzone) {
                strafeParabola = 0;
            } else if (gamepad1.left_stick_x < 0) {
                strafeParabola = -curve * ((gamepad1.left_stick_x * 10) * (gamepad1.left_stick_x * 10));
            } else if (gamepad1.left_stick_x > 0) {
                strafeParabola = curve * ((gamepad1.left_stick_x * 10) * (gamepad1.left_stick_x * 10));
            }

            if (gamepad1.right_stick_x < deadzone && gamepad1.right_stick_x > -deadzone) {
                turnParabola = 0;
            }  else if (gamepad1.right_stick_x < 0) {
                turnParabola = -curve * ((gamepad1.right_stick_x * 10) * (gamepad1.right_stick_x * 10));
            } else if (gamepad1.right_stick_x > 0) {
                turnParabola = curve * ((gamepad1.right_stick_x * 10) * (gamepad1.right_stick_x * 10));
            }

            // turns the parabolic values into values the motors can use
            double drive = driveParabola / 10;
            double strafe = -strafeParabola / 10;
            double turn = turnParabola / 10;

            // Sets the power that the motors will get and also tests to see if it should move at half speed or not
            if (drive != 0 && gamepad1.left_trigger != 0 || turn != 0 && gamepad1.left_trigger != 0 || strafe != 0 && gamepad1.left_trigger != 0) {
                turn = gamepad1.right_stick_x / 3;
                leftFrontPower = Range.clip(strafe + drive - turn, -0.5, 0.5);
                leftRearPower = Range.clip(strafe - drive + turn, -0.5, 0.5);
                rightRearPower = Range.clip(strafe + drive + turn, -0.5, 0.5);
                rightFrontPower = Range.clip(strafe - drive - turn, -0.5, 0.5);
            } else {
                leftFrontPower = Range.clip(strafe + drive - turn, -1.0, 1.0);
                leftRearPower = Range.clip(strafe - drive + turn, -1.0, 1.0);
                rightRearPower = Range.clip(strafe + drive + turn, -1.0, 1.0);
                rightFrontPower = Range.clip(strafe - drive - turn, -1.0, 1.0);
            }

            // Arm code, tests if the right bumper is pressed or not and then moves the arm accordingly
//            if (gamepad2.right_bumper) {
//                armPosition = 1;
//            } else {
//                armPosition = 0;
//            }

            // Intake System code, tests if the left trigger is pressed or if the down dpad is pressed
            // and then either moves the intake system forwards or backwards
            if (gamepad2.left_trigger != 0) {
                intakeMotorPower = 0.75;
                conveyorMotorPower = 1;
            } else if (gamepad2.y) {
                intakeMotorPower = -0.75;
                conveyorMotorPower = -1;
            } else {
                intakeMotorPower = 0;
                conveyorMotorPower = 0;
            }

            // Fly wheel code, sets the value of the fly wheels according to the button pressed
            //note: ring flies more level when the flywheels spin at different powers.
            //note: a difference of 1.5 seems to work well

            if (gamepad2.a) {
                rightFlyWheelPower = 0.325;
                leftFlyWheelPower = 0.425;
            }
            else if (gamepad2.b) {
                rightFlyWheelPower = 0.25;
                leftFlyWheelPower = 0.35;
            }
            else if (gamepad2.y) {
                rightFlyWheelPower = -0.5;
                leftFlyWheelPower = -0.5;
            }
            else {
                rightFlyWheelPower = 0;
                leftFlyWheelPower = 0;
            }


            // Pusher code, tests left bumper to see if the pusher should move or not
            if (gamepad2.left_bumper) {
                pushPosition = 1;
            } else {
                pushPosition = 0;
            }



            // Sets the power of the motors and servos
            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);
            rightFlyWheel.setPower(rightFlyWheelPower);
            leftFlyWheel.setPower(leftFlyWheelPower);
            intakeMotor.setPower(intakeMotorPower);
            conveyorMotor.setPower(conveyorMotorPower);
            push.setPosition(pushPosition);
//            arm.setPosition(armPosition);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left rear (%.2f), right rear (%.2f), left front (%.2f), right front (%.2f)"
                    , leftRearPower, rightRearPower, leftFrontPower, rightFrontPower);
            telemetry.addData("Motors", "right fly wheel (%.2f), left fly wheel (%.2f), intake (%.2f), conveyor belt (%.2f), pusher (%.2f)"/*, arm (%.2f)"*/
                    , rightFlyWheelPower, leftFlyWheelPower, intakeMotorPower, conveyorMotorPower, pushPosition/*, armPower*/);
            telemetry.update();
        }
    }
}