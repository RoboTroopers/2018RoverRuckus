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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "Retard", group = "Working")
public class MecaTeleOp extends LinearOpMode {

    private ElapsedTime    runtime = new ElapsedTime();

    private DcMotor        frontLeft, frontRight, backLeft, backRight, actuator, pulley, intake, arm;
    private Servo          outtake;
    private CRServo        armIntake;
    private DigitalChannel limitSwitch;



    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    @Override
    public void runOpMode() {

        actuator = hardwareMap.get(DcMotor.class, "actuator");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        pulley = hardwareMap.get(DcMotor.class, "pulley");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(Servo.class, "outtake");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        arm = hardwareMap.get(DcMotor.class, "arm");
        armIntake = hardwareMap.get(CRServo.class, "armIntake");


        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        pulley.setDirection(DcMotor.Direction.REVERSE);
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);


        waitForStart();



        if (opModeIsActive()) {
        }

        while (opModeIsActive())
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
                outtake.setPosition(0.35);
                sleep(500);
            }

            else if(gamepad2.right_trigger == 1)
            {
                outtake.setPosition(0.185);
                sleep(500);
            }

            else
            {
                outtake.setPosition(0.01);
            }




            if(gamepad1.left_bumper)
            {
                arm.setPower(0.3);
            }

            else if(gamepad1.right_bumper)
            {
                arm.setPower(-0.3);
            }

            else {
                arm.setPower(0);
            }



            if(gamepad1.left_trigger == 1)
            {
                armIntake.setPower(-1);
            }

            else if(gamepad1.right_trigger == 1)
            {
                armIntake.setPower(1);
            }

            else {
                armIntake.setPower(0);
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


            telemetry.addData("front left power", frontLeft.getPower());
            telemetry.addData("front right power", frontRight.getPower());
            telemetry.addData("back left power", backLeft.getPower());
            telemetry.addData("back right power", backRight.getPower());
            telemetry.addData("intake power", intake.getPower());
            telemetry.addData("outtake position", outtake.getPosition());
            telemetry.addData("arm power", arm.getPower());

            if (limitSwitch.getState()) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
            }

            telemetry.addData("actuator pos", actuator.getCurrentPosition());
            telemetry.update();

        }

    }
}