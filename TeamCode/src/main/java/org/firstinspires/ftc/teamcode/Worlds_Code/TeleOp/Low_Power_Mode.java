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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * CLICK FOR DA QUEEN
 * *CLICK CLICK CLICK CLICK CLICK*
 */

@Disabled
@TeleOp(name = "Low Power Ugandan Knuckles")
public class Low_Power_Mode extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor pulley;
    private DcMotor outtakePulley;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        pulley     = hardwareMap.get(DcMotor.class, "pulley");
        outtakePulley = hardwareMap.get(DcMotor.class, "outtakePulley");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();

        if (opModeIsActive()) {
        }

        while (opModeIsActive())
        {

            //Drive motor control

            double threshold = 0.7;

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

            if(Math.abs(gamepad2.right_stick_y) > threshold)
            {
                pulley.setPower(gamepad2.right_stick_y);
            }

            else
            {
                pulley.setPower(0);
            }

            outtakePulley.setPower(gamepad2.left_stick_y);


            // Telemetry
            telemetry.addData("front left power", frontLeft.getPower());
            telemetry.addData("front right power", frontRight.getPower());
            telemetry.addData("back left power", backLeft.getPower());
            telemetry.addData("back right power", backRight.getPower());
        }
    }
}