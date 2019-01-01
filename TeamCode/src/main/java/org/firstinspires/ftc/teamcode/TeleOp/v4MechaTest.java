package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


//Version 1.4

@TeleOp(name = "I Hope This Works", group = "Working")
public class v4MechaTest extends LinearOpMode {

    private DcMotor        frontLeft;
    private DcMotor        frontRight;
    private DcMotor        backLeft;
    private DcMotor        backRight;
    private DcMotor        actuator;





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




        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);






        waitForStart();
        if (opModeIsActive()) {
        }

        while (opModeIsActive())
        {

            //either use this with rotation on the right joystick and normal movement on the left joystick

            /*

double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
double rightX = gamepad1.right_stick_x;
final double v1 = r * Math.cos(robotAngle) + rightX;
final double v2 = r * Math.sin(robotAngle) - rightX;
final double v3 = r * Math.sin(robotAngle) + rightX;
final double v4 = r * Math.cos(robotAngle) - rightX;

frontLeft.setPower(v1);
frontRight.setPower(v2);
backLeft.setPower(v3)
backRight.setPower(v4);


*/



            //or this, which is tank drive with normal joystick and button movement




            frontLeft.setPower((-gamepad1.left_stick_y));
            backLeft.setPower((-gamepad1.left_stick_y));
            frontRight.setPower((-gamepad1.right_stick_y));
            backRight.setPower((-gamepad1.right_stick_y));

            if(gamepad1.left_bumper)
            {
                frontLeft.setPower(-1);
                backRight.setPower(-1);
                frontRight.setPower(1);
                backLeft.setPower(1);
            }

            if(gamepad1.right_bumper)
            {
                frontLeft.setPower(1);
                backRight.setPower(1);
                frontRight.setPower(-1);
                backLeft.setPower(-1);
            }

            if(gamepad1.left_trigger == 1)
            {
                frontRight.setPower(1);
                backLeft.setPower(1);
                frontLeft.setPower(0);
                backRight.setPower(0);
            }

            if(gamepad1.right_trigger == 1)
            {
                frontLeft.setPower(1);
                backRight.setPower(1);
                frontRight.setPower(0);
                backLeft.setPower(0);
            }

            actuator.setPower(gamepad2.left_stick_y);


            telemetry.addData("front left power", frontLeft.getPower());
            telemetry.addData("front right power", frontRight.getPower());
            telemetry.addData("back left power", backLeft.getPower());
            telemetry.addData("back right power", backRight.getPower());
            telemetry.update();






            /*

            frontLeft.setPower((gamepad1.left_stick_y)/1.5);
            backLeft.setPower((gamepad1.right_stick_y)/1.5);
            frontRight.setPower((gamepad2.left_stick_y)/1.5);
            backRight.setPower((gamepad2.right_stick_y)/1.5);

            */


            /*

            this is what i use for vex meca (test)

            double threshold = 0.157;

            if(Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold)
            {
                frontRight.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x));
                frontLeft.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x));
                backRight.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x));
                backLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x));
            }

            if(Math.abs(gamepad1.right_stick_x) > threshold)
            {
                frontRight.setPower((-gamepad1.right_stick_x));
                frontLeft.setPower((-gamepad1.right_stick_x));
                backLeft.setPower((gamepad1.right_stick_x));
                backRight.setPower((gamepad1.right_stick_x));
            }

            */


        }

        }
    }