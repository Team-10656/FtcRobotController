package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous Mode", group="Exercises")
@Disabled
public class MecanumAutonomous extends LinearOpMode {
    // Sets the runtime variable to the elapsed time within autonomous
    private ElapsedTime runtime = new ElapsedTime();

    // Creates variables for all the motors and servos
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private DcMotor leftFront = null;
    private DcMotor flyWheel = null;


    // Sets the motor specifications as variables
    static final double COUNTS_PER_MOTOR_REV = 960;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.6;


    // called when init button is  pressed.

    @Override
    public void runOpMode() throws InterruptedException
    {

        // Defines the names of all the motors and servos
        leftRear  = hardwareMap.get(DcMotor.class, "left_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        flyWheel = hardwareMap.get(DcMotor.class, "fly_wheel");

        // sets the direction of the motors
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // resets encoders (doesnt work?)
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d", leftRear.getCurrentPosition()
                , rightFront.getCurrentPosition(), leftFront.getCurrentPosition(), rightRear.getCurrentPosition());
        telemetry.update();


        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // drives to about the blue dot on the field
        encoderDrive(DRIVE_SPEED,  10,  10, 0.5);

        encoderDrive(TURN_SPEED, 45, -45, 2.0);

        encoderDrive(DRIVE_SPEED, 80, 80, 5.0);

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

            //  sleep(250);   // optional pause after each move
        }
    }
}