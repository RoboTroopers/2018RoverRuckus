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
package org.firstinspires.ftc.teamcode.Worlds_Code.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * CLICK FOR DA QUEEN
 * *CLICK CLICK CLICK CLICK CLICK*
 */

@TeleOp(name = "Ugandan Knuckles")
public class UgandanKnuckles extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor actuator;
    private DcMotor pulley;
    private DcMotor outtakePulley;
    private Servo   outtake;
    private Servo   intakeRotate;
    private CRServo intake;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    @Override
    public void runOpMode() {

        frontLeft     = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight    = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft      = hardwareMap.get(DcMotor.class, "backLeft");
        backRight     = hardwareMap.get(DcMotor.class, "backRight");

        outtakePulley = hardwareMap.get(DcMotor.class, "outtakePulley");
        outtake       = hardwareMap.get(Servo.class, "outtake");
        actuator      = hardwareMap.get(DcMotor.class, "actuator");

        intake        = hardwareMap.get(CRServo.class, "intake");
        pulley        = hardwareMap.get(DcMotor.class, "pulley");
        intakeRotate  = hardwareMap.get(Servo.class, "intakeRotate");


        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        waitForStart();

        if (opModeIsActive()) {
        }

        while (opModeIsActive())
        {


            //Drive motor control

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



            // Rest of the motors control

            pulley.setPower(-gamepad2.left_stick_y);

            if(gamepad2.dpad_up)
            {
                actuator.setTargetPosition(-7000);
                actuator.setPower(1);

                while(actuator.isBusy())
                {
                    telemetry.addData("actuator is going up","");
                    telemetry.update();
                }

                actuator.setPower(0);
            }


            else if(gamepad2.dpad_down)
            {
                actuator.setTargetPosition(0);
                actuator.setPower(1);

                while(actuator.isBusy())
                {
                    telemetry.addData("actuator is going down","");
                    telemetry.update();
                }

                actuator.setPower(0);
            }


            /*if(gamepad1.dpad_up)
            {
                actuator.setPower(1);
            }

            else if(gamepad1.dpad_down)
            {
                actuator.setPower(-1);
            }

            else {
                actuator.setPower(0);
            }*/


            // Servo control

            if(gamepad2.b)
            {
                intakeRotate.setPosition(0.603);
            }

            else if(gamepad2.y)
            {
                intakeRotate.setPosition(0);
            }



            outtakePulley.setPower(gamepad2.right_stick_y);

            if(gamepad2.left_trigger == 1)
            {
                intake.setPower(1);
            }

            else if(gamepad2.right_trigger == 1)
            {
                intake.setPower(-1);
            }

            else {
                intake.setPower(0);
            }



            if(gamepad2.right_bumper)
            {
                outtake.setPosition(0);
            }

            else if(gamepad2.left_bumper)
            {
                outtake.setPosition(0.4);
            }

            else {
                outtake.setPosition(0);
            }



            /*if(gamepad2.left_bumper)
            {
                outtakePulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outtakePulley.setTargetPosition(-1304);
                outtakePulley.setPower(1);

                while(outtakePulley.isBusy())
                {
                    telemetry.addData("outtake pulley pos",outtakePulley.getCurrentPosition());
                    telemetry.update();
                }

                outtakePulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                outtakePulley.setPower(0.33);

                outtake.setPosition(0.4);
                sleep(500);
                outtake.setPosition(0);
                sleep(200);

                outtakePulley.setPower(0);
            }*/


            // Telemetry
            telemetry.addData("front left power", frontLeft.getPower());
            telemetry.addData("front right power", frontRight.getPower());
            telemetry.addData("back left power", backLeft.getPower());
            telemetry.addData("back right power", backRight.getPower());
            telemetry.addData("actuator power", actuator.getPower());
            telemetry.addData("pulley power", pulley.getPower());
            telemetry.addData("outtake pulley power", outtakePulley.getPower());

            telemetry.addData("actuator pos", actuator.getCurrentPosition());
            telemetry.addData("pulley pos", pulley.getCurrentPosition());
            telemetry.addData("outtake pulley pos", outtakePulley.getCurrentPosition());
            telemetry.addData("outtake pos", outtake.getPosition());

            telemetry.addData("intake power", intake.getPower());
            telemetry.update();
        }
    }
}
