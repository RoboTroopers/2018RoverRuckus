package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;


//Version 1.4

@TeleOp(name = "MecaWorking TeleOp", group = "Working")
public class MecaTeleOp extends LinearOpMode {

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
    public void runOpMode() {

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

                intake.setPower(intakeVar/1.5);

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

                intake.setPower(intakeVar/1.5);

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
                    frontRight.setPower(((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x))/2);
                    backLeft.setPower(((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x))/2);
                    frontLeft.setPower(((-gamepad1.left_stick_y) - (-gamepad1.left_stick_x))/2);
                    backRight.setPower(((-gamepad1.left_stick_y) - (-gamepad1.left_stick_x))/2);
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
                    frontRight.setPower((-gamepad1.right_stick_x/2));
                    frontLeft.setPower((gamepad1.right_stick_x/2));
                    backLeft.setPower((gamepad1.right_stick_x/2));
                    backRight.setPower((-gamepad1.right_stick_x)/2);
                }



            }



            telemetry.addData("front left power", frontLeft.getPower());
            telemetry.addData("front right power", frontRight.getPower());
            telemetry.addData("back left power", backLeft.getPower());
            telemetry.addData("back right power", backRight.getPower());
            telemetry.addData("intake power", intake.getPower());
            telemetry.addData("outtake power", outtake.getPower());
            telemetry.update();

        }

    }
}