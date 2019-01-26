package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Retard", group = "Working")
public class MecaTeleOp extends LinearOpMode {

    private ElapsedTime    runtime = new ElapsedTime();

    private DcMotor        frontLeft, frontRight, backLeft, backRight, actuator, pulley, intake;
    private Servo          outtake, nLatch;
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
        nLatch = hardwareMap.get(Servo.class, "nLatch");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");


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


            if(gamepad2.dpad_down)
            {
                nLatch.setPosition(0.0);
            }
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

            /*if(gamepad2.y)
            {
                runtime.reset();
                outtake.setPosition(0.15);
            }

            else if(gamepad2.a)
            {
                runtime.reset();
                outtake.setPosition(0.35)
            }

            if(runtime.seconds() < 2)
            {
                outtake.setPosition(0.01);
            }*/


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