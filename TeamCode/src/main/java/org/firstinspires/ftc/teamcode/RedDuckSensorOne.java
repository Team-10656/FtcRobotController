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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Red Duck Sensor", group="Red")
//@Disabled
public class RedDuckSensorOne extends LinearOpMode {
    // Sets the runtime variable to the elapsed time within autonomous
    private ElapsedTime runtime = new ElapsedTime();

    // Creates variables for all the motors and servos
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private DcMotor leftFront = null;
    private DcMotor flywheel = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private DcMotor armOne = null;
    private DcMotor armTwo = null;
    private DistanceSensor distance;


    // Sets the motor specifications as variables
    // This has the information for rev motors: https://docs.revrobotics.com/rev-control-system/sensors/encoders/motor-based-encoders
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.8;


    // called when init button is  pressed.

    @Override
    public void runOpMode() throws InterruptedException {

        // Defines the names of all the motors and servos
        leftRear  = hardwareMap.get(DcMotor.class, "left_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        armOne = hardwareMap.get(DcMotor.class, "arm_one");
        armTwo = hardwareMap.get(DcMotor.class, "arm_two");
        distance = hardwareMap.get(DistanceSensor.class, "distance");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distance;


        // sets the direction of the motors
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        armOne.setDirection(DcMotor.Direction.REVERSE);
        armTwo.setDirection(DcMotor.Direction.FORWARD);


        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d", leftRear.getCurrentPosition()
                , rightFront.getCurrentPosition(), leftFront.getCurrentPosition(), rightRear.getCurrentPosition());
        telemetry.update();


        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        leftClaw.setPosition(0);
        rightClaw.setPosition(0);

        sleep(2000);

        armOne.setPower(0.6);
        armTwo.setPower(0.6);

        sleep(1000);

        armOne.setPower(0.0005);
        armTwo.setPower(0.0005);

        while(distance.getDistance(DistanceUnit.INCH) <= 12) {
            rightFront.setPower(0.8);
            leftFront.setPower(0.8);
            rightRear.setPower(0.8);
            leftRear.setPower(0.8);
        }

        rightFront.setPower(-0.8);
        leftFront.setPower(-0.8);
        rightRear.setPower(-0.8);
        leftRear.setPower(-0.8);

        sleep(100);

        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);

        sleep(500);

        encoderDrive(TURN_SPEED, 39, -39, 4);

        if (distance.getDistance(DistanceUnit.INCH) <= 8) {
            encoderDrive(TURN_SPEED, -19, 19, 2);

            armOne.setPower(-0.6);
            armTwo.setPower(-0.6);

            sleep(600);

            armOne.setPower(0.0005);
            armTwo.setPower(0.0005);

            encoderDrive(DRIVE_SPEED, 26, 26, 2);

            encoderDrive(TURN_SPEED, -19, 19, 2);
        } else {

            encoderDrive(TURN_SPEED, -19, 19, 2);

            encoderDrive(DRIVE_SPEED, 6, 6, 1);

            encoderDrive(TURN_SPEED, 19, -19, 2);

            if (distance.getDistance(DistanceUnit.INCH) <= 8) {
                encoderDrive(TURN_SPEED, -20, 20, 2);

                armOne.setPower(-0.6);
                armTwo.setPower(-0.6);

                sleep(335);

                armOne.setPower(0.0005);
                armTwo.setPower(0.0005);

                encoderDrive(DRIVE_SPEED, 24, 24, 2);

                encoderDrive(TURN_SPEED, -19, 19, 2);
            } else {

                encoderDrive(TURN_SPEED, -19, 19, 2);

                encoderDrive(DRIVE_SPEED, 22, 22, 2);

                encoderDrive(TURN_SPEED, -19, 19, 2);

            }
        }

        encoderDrive(TURN_SPEED, 19, -19, 2);

        encoderDrive(DRIVE_SPEED / 2, -36, -36, 4);

        encoderDrive(TURN_SPEED, -10,10, 1);

        encoderDrive(DRIVE_SPEED / 2, -12, -12, 2);

        rightFront.setPower(-0.01);
        leftFront.setPower(-0.01);
        rightRear.setPower(-0.01);
        leftRear.setPower(-0.01);

        sleep(200);

        flywheel.setPower(0.8);
        sleep(3000);
        flywheel.setPower(0);

        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);

        encoderDrive(DRIVE_SPEED,  12,  12, 2);

        encoderDrive(TURN_SPEED, 10, -10, 2);

        encoderDrive(DRIVE_SPEED, 100, 100, 11);

        armOne.setPower(0);
        armTwo.setPower(0);

    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTargetOne;
        int newRightTargetOne;
        int newLeftTargetTwo;
        int newRightTargetTwo;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTargetOne = leftRear.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTargetOne = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftTargetTwo = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTargetTwo = rightRear.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            leftRear.setTargetPosition(newLeftTargetOne);
            rightFront.setTargetPosition(newRightTargetOne);
            leftFront.setTargetPosition(newLeftTargetTwo);
            rightRear.setTargetPosition(newRightTargetTwo);

            // Turn On RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftRear.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftFront.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftRear.isBusy() && rightFront.isBusy() && leftFront.isBusy() && rightRear.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTargetOne,  newRightTargetOne, newLeftTargetTwo, newRightTargetTwo);
                telemetry.addData("Path2",  "Running at %7d :%7d", leftRear.getCurrentPosition(), rightFront.getCurrentPosition()
                        , leftFront.getCurrentPosition(), rightRear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftRear.setPower(0);
            rightFront.setPower(0);
            leftFront.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

              sleep(250);   // optional pause after each move
        }
    }

    public void encoderDrive(double speed, double leftFrontInches, double rightFrontInches, double leftRearInches, double rightRearInches, double timeoutS) {
        int newLeftTargetOne;
        int newRightTargetOne;
        int newLeftTargetTwo;
        int newRightTargetTwo;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTargetOne = leftRear.getCurrentPosition() + (int)(leftRearInches * COUNTS_PER_INCH);
            newRightTargetOne = rightFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftTargetTwo = leftFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightTargetTwo = rightRear.getCurrentPosition() + (int)(rightRearInches * COUNTS_PER_INCH);

            leftRear.setTargetPosition(newLeftTargetOne);
            rightFront.setTargetPosition(newRightTargetOne);
            leftFront.setTargetPosition(newLeftTargetTwo);
            rightRear.setTargetPosition(newRightTargetTwo);

            // Turn On RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftRear.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftFront.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftRear.isBusy() && rightFront.isBusy() && leftFront.isBusy() && rightRear.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTargetOne,  newRightTargetOne, newLeftTargetTwo, newRightTargetTwo);
                telemetry.addData("Path2",  "Running at %7d :%7d", leftRear.getCurrentPosition(), rightFront.getCurrentPosition()
                        , leftFront.getCurrentPosition(), rightRear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftRear.setPower(0);
            rightFront.setPower(0);
            leftFront.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}