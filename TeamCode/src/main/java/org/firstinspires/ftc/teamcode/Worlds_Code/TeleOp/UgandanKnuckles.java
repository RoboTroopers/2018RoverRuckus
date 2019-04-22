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

import com.qualcomm.hardware.motors.Matrix12vMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubMotor;

/**
 * CLICK FOR DA QUEEN
 * *CLICK CLICK CLICK CLICK CLICK*
 */

@TeleOp(name = "Ugandan Knuckles")
public class UgandanKnuckles extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    private DcMotor actuator;
    private DcMotor pulley;
    private DcMotor outtakePulley;
    private Servo   outtake;
    private Servo   intakeRotate;
    private Servo   jammer;
    private CRServo intake;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        pulley        = hardwareMap.get(DcMotor.class, "pulley");
        outtakePulley = hardwareMap.get(DcMotor.class, "outtakePulley");
        actuator      = hardwareMap.get(DcMotor.class, "actuator");

        intake        = hardwareMap.get(CRServo.class, "intake");
        jammer        = hardwareMap.get(Servo.class, "pepeJAM");
        intakeRotate  = hardwareMap.get(Servo.class, "intakeRotate");
        outtake       = hardwareMap.get(Servo.class, "outtake");


        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();

        if (opModeIsActive()) {
        }

        while (opModeIsActive())
        {


            //Drive motor control

            double threshold = 0.157;

            if(Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold)
            {
                rightFront.setPower(((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)));
                leftRear.setPower(((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)));
                leftFront.setPower(((-gamepad1.left_stick_y) - (-gamepad1.left_stick_x)));
                rightRear.setPower(((-gamepad1.left_stick_y) - (-gamepad1.left_stick_x)));
            }

            else
            {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
            }

            if(Math.abs(gamepad1.right_stick_x) > threshold)
            {
                rightFront.setPower((-gamepad1.right_stick_x));
                leftFront.setPower((gamepad1.right_stick_x));
                leftRear.setPower((gamepad1.right_stick_x));
                rightRear.setPower((-gamepad1.right_stick_x));
            }



            // Rest of the motors control

            outtakePulley.setPower(gamepad2.right_stick_y);

            if(outtakePulley.getPower() < -0.75)
            {
                outtakePulley.setPower(gamepad2.right_stick_y + 0.25);
            }

            else {
                outtakePulley.setPower(gamepad2.right_stick_y);
            }


            if(gamepad2.right_bumper)
            {
                if(-gamepad2.left_stick_y > 0.5)
                {
                    pulley.setPower(-gamepad2.left_stick_y - 0.25);
                }

                if(-gamepad2.left_stick_y < -0.5)
                {
                    pulley.setPower(-gamepad2.left_stick_y + 0.25);
                }
            }

            else {
                pulley.setPower(-gamepad2.left_stick_y);
            }




            if(gamepad2.dpad_up)
            {
                actuator.setPower(1);
            }

            else if(gamepad2.dpad_down)
            {
                actuator.setPower(-1);
            }

            else {
                actuator.setPower(0);
            }




            // Servo control

            if(gamepad2.dpad_left)
            {
                jammer.setPosition(0.2);
                sleep(500);
            }

            else if(gamepad2.dpad_right)
            {
                jammer.setPosition(0);
                sleep(500);
            }



            if(gamepad2.b)
            {
                intakeRotate.setPosition(0.73); //.725
            }

            else if(gamepad2.y)
            {
                intakeRotate.setPosition(0);
            }



            if(gamepad2.right_trigger == 1)
            {
                intake.setPower(1);
            }

            else if(gamepad2.left_trigger ==  1)
            {
                intake.setPower(-1);
            }

            else {
                intake.setPower(0);
            }



            if(gamepad2.left_bumper)
            {
                outtake.setPosition(0.5);
            }

            else if(outtakePulley.getPower() < 0)
            {
                outtake.setPosition(0.24);
            }

            else {
                outtake.setPosition(0);
            }


            // Telemetry
            telemetry.addData("front left power", leftFront.getPower());
            telemetry.addData("front right power", rightFront.getPower());
            telemetry.addData("back left power", leftRear.getPower());
            telemetry.addData("back right power", rightRear.getPower());
            telemetry.addData("actuator power", actuator.getPower());
            telemetry.addData("pulley power", pulley.getPower());
            telemetry.addData("outtake pulley power", outtakePulley.getPower());

            telemetry.addData("actuator pos", actuator.getCurrentPosition());
            telemetry.addData("pulley pos", pulley.getCurrentPosition());
            telemetry.addData("outtake pulley pos", outtakePulley.getCurrentPosition());
            telemetry.addData("outtake pos", outtake.getPosition());

            telemetry.addData("intake power", intake.getPower());
            telemetry.addData("intake pos", intakeRotate.getPosition());
            telemetry.update();
        }
    }
}
