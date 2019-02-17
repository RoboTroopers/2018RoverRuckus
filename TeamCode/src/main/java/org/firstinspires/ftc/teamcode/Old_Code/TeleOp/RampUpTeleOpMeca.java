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

package org.firstinspires.ftc.teamcode.Old_Code.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
@TeleOp(name = "RampUp TeleOp", group = "Working")
public class RampUpTeleOpMeca extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    private DcMotor        frontLeft;
    private DcMotor        frontRight;
    private DcMotor        backLeft;
    private DcMotor        backRight;
    private DcMotor        actuator;
    private DcMotor        pulley;
    private DcMotor        intake;
    private CRServo        outtake;



    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    @Override
    public void runOpMode()
    {

        actuator = hardwareMap.get(DcMotor.class, "actuator");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        pulley = hardwareMap.get(DcMotor.class, "pulley");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(CRServo.class, "outtake");



        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        pulley.setDirection(DcMotor.Direction.REVERSE);




        waitForStart();

        if (opModeIsActive()) {
        }

        while (opModeIsActive())
        {
                if(gamepad1.x)
                {
                    actuator.setPower(gamepad2.left_stick_y);
                    pulley.setPower(gamepad2.right_stick_y);

                    int intakeVar;

                    if(gamepad2.left_bumper)
                    {
                        intakeVar = 1;
                    }

                    else if(gamepad2.right_bumper)
                    {
                        intakeVar = -1;
                    }

                    else
                    {
                        intakeVar = 0;
                    }

                    intake.setPower(intakeVar);

                    if(gamepad2.left_trigger == 1)
                    {
                        outtake.setPower(1);
                    }

                    else if(gamepad2.right_trigger == 1)
                    {
                        outtake.setPower(-1);
                    }

                    else
                    {
                        outtake.setPower(0);
                    }

                    if((-gamepad1.left_stick_y) < -0.75)
                    {
                        frontLeft.setPower((-gamepad1.left_stick_y) + 0.75);
                        backLeft.setPower((-gamepad1.left_stick_y) + 0.75);
                    }

                    else if((-gamepad1.left_stick_y) > 0.75)
                    {
                        frontLeft.setPower((-gamepad1.left_stick_y) - 0.75);
                        backLeft.setPower((-gamepad1.left_stick_y) - 0.75);
                    }

                    else
                    {
                        frontLeft.setPower(0);
                        backRight.setPower(0);
                    }

                    if((-gamepad1.right_stick_y) < -0.75)
                    {
                        frontRight.setPower((-gamepad1.right_stick_y) + 0.75);
                        backRight.setPower((-gamepad1.right_stick_y) + 0.75);
                    }

                    else if((-gamepad1.right_stick_y) > 0.75)
                    {
                        frontRight.setPower((-gamepad1.right_stick_y) - 0.75);
                        backRight.setPower((-gamepad1.right_stick_y) - 0.75);
                    }

                    else
                    {
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                    }

                    if(gamepad1.left_bumper)
                    {
                        runtime.reset();
                        double R = runtime.seconds();
                        double NFRBLSpeed;
                        double NFLBRSpeed;

                        if(R < 2.5)
                        {
                            double ramp = R / 2.5;
                            NFRBLSpeed = 0.75 * ramp;
                            NFLBRSpeed = NFRBLSpeed * -1;
                        }

                        else
                        {
                            NFRBLSpeed = 0.75;
                            NFLBRSpeed = NFRBLSpeed * -1;
                        }

                        frontLeft.setPower(NFLBRSpeed);
                        backRight.setPower(NFLBRSpeed);
                        frontRight.setPower(NFRBLSpeed);
                        backLeft.setPower(NFRBLSpeed);
                    }

                    if(gamepad1.right_bumper)
                    {

                        runtime.reset();
                        double R = runtime.seconds();
                        double NFRBLSpeed;
                        double NFLBRSpeed;

                        if(R < 2.5)
                        {
                            double ramp = R / 2.5;
                            NFLBRSpeed = 0.75 * ramp;
                            NFRBLSpeed = NFLBRSpeed * -1;
                        }

                        else
                        {
                            NFLBRSpeed = 0.75;
                            NFRBLSpeed = NFLBRSpeed * -1;
                        }

                        frontLeft.setPower(NFLBRSpeed);
                        backRight.setPower(NFLBRSpeed);
                        frontRight.setPower(NFRBLSpeed);
                        backLeft.setPower(NFRBLSpeed);
                    }

                }

                else
                {

                    actuator.setPower(gamepad2.left_stick_y);
                    pulley.setPower(gamepad2.right_stick_y);

                    int intakeVar;

                    if(gamepad2.left_bumper)
                    {
                        intakeVar = 1;
                    }

                    else if(gamepad2.right_bumper)
                    {
                        intakeVar = -1;
                    }

                    else
                    {
                        intakeVar = 0;
                    }

                    intake.setPower(intakeVar);

                    if(gamepad2.left_trigger == 1)
                    {
                        outtake.setPower(1);
                    }

                    else if(gamepad2.right_trigger == 1)
                    {
                        outtake.setPower(-1);
                    }

                    else
                    {
                        outtake.setPower(0);
                    }

                    double threshold = 0.157;

                    if(Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold)
                    {
                        frontRight.setPower(((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)));
                        backLeft.setPower(((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)));
                        frontLeft.setPower(((-gamepad1.left_stick_y) - (-gamepad1.left_stick_x)));
                        backRight.setPower(((-gamepad1.left_stick_y) - (-gamepad1.left_stick_x)));
                    }

                    else
                    {
                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);
                    }

                    if(Math.abs(gamepad1.right_stick_x) > threshold)
                    {
                        frontRight.setPower((-gamepad1.right_stick_x));
                        frontLeft.setPower((gamepad1.right_stick_x));
                        backLeft.setPower((gamepad1.right_stick_x));
                        backRight.setPower((-gamepad1.right_stick_x));
                    }

                    if(gamepad1.left_bumper)
                    {
                        runtime.reset();
                        double R = runtime.seconds();
                        double NFRBLSpeed;
                        double NFLBRSpeed;

                        if(R < 2.5)
                        {
                            double ramp = R / 2.5;
                            NFRBLSpeed = 0.75 * ramp;
                            NFLBRSpeed = NFRBLSpeed * -1;
                        }

                        else
                        {
                            NFRBLSpeed = 0.75;
                            NFLBRSpeed = NFRBLSpeed * -1;
                        }

                        frontLeft.setPower(NFLBRSpeed);
                        backRight.setPower(NFLBRSpeed);
                        frontRight.setPower(NFRBLSpeed);
                        backLeft.setPower(NFRBLSpeed);
                    }

                    if(gamepad1.right_bumper)
                    {

                        runtime.reset();
                        double R = runtime.seconds();
                        double NFRBLSpeed;
                        double NFLBRSpeed;

                        if(R < 2.5)
                        {
                            double ramp = R / 2.5;
                            NFLBRSpeed = 0.75 * ramp;
                            NFRBLSpeed = NFLBRSpeed * -1;
                        }

                        else
                        {
                            NFLBRSpeed = 0.75;
                            NFRBLSpeed = NFLBRSpeed * -1;
                        }

                        frontLeft.setPower(NFLBRSpeed);
                        backRight.setPower(NFLBRSpeed);
                        frontRight.setPower(NFRBLSpeed);
                        backLeft.setPower(NFRBLSpeed);
                    }

                }



        }




            telemetry.addData("front left power", frontLeft.getPower());
            telemetry.addData("front right power", frontRight.getPower());
            telemetry.addData("back left power", backLeft.getPower());
            telemetry.addData("back right power", backRight.getPower());
            telemetry.addData("pulley power", pulley.getPower());
            telemetry.addData("actuator power", actuator.getPower());
            telemetry.update();

    }
}