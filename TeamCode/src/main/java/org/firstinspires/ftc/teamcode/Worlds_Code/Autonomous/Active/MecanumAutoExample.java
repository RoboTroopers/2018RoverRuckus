/*
 *
 *     Copyright (c) 2018 FTC Team 15167 Robo Troopers (http://robotroopers.org)
 *
 *     Permission is hereby granted, free of charge, to any person obtaining a copy
 *     of this software and associated documentation files (the "Software"), to deal
 *     in the Software without restriction, including without limitation the rights
 *     to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *     copies of the Software, and to permit persons to whom the Software is
 *     furnished to do so, subject to the following conditions:
 *
 *     The above copyright notice and this permission notice shall be included in all
 *     copies or substantial portions of the Software.
 *
 *     THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *     IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *     FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *     AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *     LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *     OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *     SOFTWARE.
 *
 */


package org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.Active;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

import java.util.Set;


@Disabled
@Autonomous(name = "Example Auto", group = "Working")

public class MecanumAutoExample extends LinearOpMode {

    private ElapsedTime       runtime = new ElapsedTime();

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;


    private static final double COUNTS_PER_MOTOR_REV = 383.6; //TODO: Set this to whatever your encoder counts are
    private static final double DRIVE_GEAR_REDUCTION = 1.0;   //TODO: Self explanatory
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double PI = 3.14159265359;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);


    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        //wait for game to start (driver presses PLAY)
        waitForStart();

        //TODO: Write what you want the robot to do in auto here

    }


    public void strafe(double speed, double FRBLInches, int timeoutS) {
        int newFRBLTarget;
        int newFLBRTarget;
        double FLBRInches = FRBLInches * -1;

        allMotorsResetEncoder();


        if(opModeIsActive())
        {
            newFRBLTarget = (frontRight.getCurrentPosition() + backLeft.getCurrentPosition())/2 + (int)(FRBLInches * COUNTS_PER_INCH);
            newFLBRTarget = (frontLeft.getCurrentPosition() + backRight.getCurrentPosition())/2 + (int)(FLBRInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newFLBRTarget);
            backRight.setTargetPosition(newFLBRTarget);
            frontRight.setTargetPosition(newFRBLTarget);
            backLeft.setTargetPosition(newFRBLTarget);

            allMotorsToPosition();

            runtime.reset();

            frontRight.setPower(Math.abs(speed));
            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            while(opModeIsActive() && (runtime.seconds() < timeoutS)
                    && (frontLeft.isBusy()) && (frontRight.isBusy()) &&
                    (backLeft.isBusy()) && (backRight.isBusy()))
            {
                motorTelemetryPower();
            }

            allMotorsZero();
            allMotorsResetEncoder();
            allMotorsRunUsing();
        }
    }

    public void diagonal(double speed, double Inches, int direction, int timeoutS)
    {
        int newFLBRTarget;
        int newFRBLTarget;

        if(opModeIsActive())
        {
            switch (direction)
            {
                case 1:
                    newFLBRTarget = (frontLeft.getCurrentPosition() + backRight.getCurrentPosition())/2 + (int)(Inches * COUNTS_PER_INCH);

                    frontLeft.setTargetPosition(newFLBRTarget);
                    backRight.setTargetPosition(newFLBRTarget);

                    allMotorsToPosition();

                    runtime.reset();

                    frontLeft.setPower(Math.abs(speed));
                    backRight.setPower(Math.abs(speed));
                    frontRight.setPower(-0.01);
                    backLeft.setPower(-0.01);

                    while(opModeIsActive() && (runtime.seconds() < timeoutS) && (frontLeft.isBusy()) && (backRight.isBusy()))
                    {
                        motorTelemetryPower();
                    }

                    allMotorsZero();
                    allMotorsRunUsing();

                    break;

                case 2:
                    newFRBLTarget = (frontRight.getCurrentPosition() + backRight.getCurrentPosition())/2 + (int)(Inches * COUNTS_PER_INCH);

                    frontRight.setTargetPosition(newFRBLTarget);
                    backLeft.setTargetPosition(newFRBLTarget);

                    allMotorsToPosition();

                    runtime.reset();

                    frontRight.setPower(Math.abs(speed));
                    backLeft.setPower(Math.abs(speed));
                    frontLeft.setPower(-0.01);
                    backRight.setPower(-0.01);

                    while (opModeIsActive() && (runtime.seconds() < timeoutS) && (frontLeft.isBusy()) && (backLeft.isBusy()))
                    {
                        motorTelemetryPower();
                    }

                    allMotorsZero();
                    allMotorsRunUsing();

                    break;
            }

        }

        sleep(50); //Conserve CPU power
    }

    public void mecaDriveWithRampUp(double LSpeed, double RSpeed, double inches, double rampUp, double timeoutS)
    {
        //Initialize variables

        int newLeftTarget;
        int newRightTarget;

        //Determine new target positions to pass onto motor controller

        newLeftTarget = (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition())/2 + (int)(inches * COUNTS_PER_INCH);
        newRightTarget = (frontRight.getCurrentPosition() + backRight.getCurrentPosition())/2 + (int)(inches * COUNTS_PER_INCH);

        //Reset timeout time and start motion
        runtime.reset();

        while(opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (Math.abs((frontLeft.getCurrentPosition() + backLeft.getCurrentPosition())/2) < newLeftTarget)  &&
                (Math.abs((frontRight.getCurrentPosition() + backRight.getCurrentPosition())/2) < newRightTarget)) {

            double NLSpeed;
            double NRSpeed;

            // "Meat" of the ramp up code

            double R = runtime.seconds();

            if (R < rampUp) {
                double ramp = R / rampUp;
                NLSpeed = LSpeed * ramp;
                NRSpeed = RSpeed * ramp;
            }

            //If the ramp-up time has passed, then the set speed will be the end speed

            else {
                NLSpeed = LSpeed;
                NRSpeed = RSpeed;
            }

            //Pass these values onto the motors

            frontLeft.setPower(NLSpeed);
            backLeft.setPower(NLSpeed);

            frontRight.setPower(NRSpeed);
            backRight.setPower(NRSpeed);

        }

            telemetry.addData("Path1", "Running to %7d, %7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d, %7d",
                             (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition())/2,
                                     (frontRight.getCurrentPosition() + backRight.getCurrentPosition())/2);

            //resetC can check encoder values quickly

            double resetC = (Math.abs((frontLeft.getCurrentPosition() + backLeft.getCurrentPosition())/2) +
                             Math.abs((frontRight.getCurrentPosition() + backRight.getCurrentPosition())/2));

            allMotorsResetEncoder();

            while(Math.abs(resetC) > 0){
                allMotorsResetEncoder();

                resetC = (Math.abs((frontLeft.getCurrentPosition() + backLeft.getCurrentPosition())/2) +
                        Math.abs((frontRight.getCurrentPosition() + backRight.getCurrentPosition())/2));
            }

            allMotorsRunUsing();

    }



    private void motorTelemetryPower()
    {
        telemetry.addData("FrontLeftPower",frontLeft.getPower());
        telemetry.addData("FrontRightPower",frontRight.getPower());
        telemetry.addData("BackLeftPower",backLeft.getPower());
        telemetry.addData("BackRightPower",backRight.getPower());
        telemetry.update();
    }


    private void allMotorsResetEncoder()
    {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    private void allMotorsZero()
    {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }


    private void allMotorsToPosition()
    {
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private void allMotorsRunUsing()
    {
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}