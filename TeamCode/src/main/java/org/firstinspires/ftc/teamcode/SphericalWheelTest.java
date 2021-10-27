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
import com.qualcomm.robotcore.util.Range;


/**
 * This will eventually be test code for Spherical Wheel Driving.
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
public class SphericalWheelTest extends LinearOpMode {

    // Declare OpMode members.
    // The motor names are temporary for now
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorOne = null;
    private DcMotor motorTwo = null;
    private DcMotor motorThree = null;

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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // The directions for the motors are yet to be determined
        motorOne.setDirection(DcMotor.Direction.FORWARD);
        motorTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        motorThree.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double onePower;
            double twoPower;
            double threePower;
            // These variables below are all the variables I believe to be relevant - Jacob
            // All the variables below were type int
            // May be buttons?
            double RLR = 0;
            double RFB = 0;
            double RT = 0;
            double LLR = 0;
            double LFB = 0;
            double LT = 0;
            // Should be directions?
            double forwards;
            double backwards;
            double left;
            double right;
            double CW;
            double CCW;
            // May be temp
            double A2output;
            double A3output;
            double A4output;
            double A5output;
            double A10output;
            double A11output;

            //Majority of Spherical Code here: https://github.com/XRobots/BallWheels/tree/main/Code/BallWheels02
            //Explanation of how it moves at 6:08: https://www.youtube.com/watch?v=zKLMCO0-How

            // This code is the code I believe to be relevant, still has to be edited - Jacob
            if (RFB >= 1) {
                forwards = RFB;
            }
            else if (RFB <= -1) {
                backwards = Math.abs(RFB);
            }
            else {
                forwards = 0;
                backwards = 0;
            }

            if (RLR >= 1) {
                right = RLR;
            }
            else if (RLR <= -1) {
                left = Math.abs(RLR);
            }
            else {
                left = 0;
                right = 0;
            }

            if (LT >= 1) {
                CW = LT;
            }
            else if (LT <=-1) {
                CCW = Math.abs(LT);
            }
            else {
                CW = 0;
                CCW = 0;
            }

            A2output = backwards + (left*0.5) + CCW;
            A3output = forwards + (right*0.5) + CW;
            A4output = backwards + (right*0.5) + CW;
            A5output = forwards + (left*0.5) + CCW;
            A10output = right + CCW;
            A11output = left + CW;

            // Send calculated power to wheels
            motorOne.setPower(onePower);
            motorTwo.setPower(twoPower);
            motorThree.setPower(threePower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "one (%.2f), two (%.2f), three (%.2f)", onePower, twoPower, threePower);
            telemetry.update();
        }
    }
}
