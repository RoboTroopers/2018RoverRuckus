package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;



@TeleOp(name = "SPICEMEMEISBACK", group = "Working")
public class RampUpTeleOpMeca extends LinearOpMode {

    private DcMotor        frontLeft;
    private DcMotor        frontRight;
    private DcMotor        backLeft;
    private DcMotor        backRight;
    private DcMotor        actuator;
    private DcMotor        pulley;
    private CRServo        intake;
    private ElapsedTime       runtime = new ElapsedTime();





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
        intake = hardwareMap.get(CRServo.class, "intake");




        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        pulley.setDirection(DcMotor.Direction.REVERSE);





        waitForStart();
        if (opModeIsActive()) {
        }

        while (opModeIsActive())
        {

            double threshold = 0.157;

            if(Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold)
            {
                frontRight.setPower((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x));
                backLeft.setPower((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x));
                frontLeft.setPower((-gamepad1.left_stick_y) - (-gamepad1.left_stick_x));
                backRight.setPower((-gamepad1.left_stick_y) - (-gamepad1.left_stick_x));
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
                frontRight.setPower(-gamepad1.right_stick_x);
                frontLeft.setPower(gamepad1.right_stick_x);
                backLeft.setPower(gamepad1.right_stick_x);
                backRight.setPower(-gamepad1.right_stick_x);
            }

            actuator.setPower(gamepad2.left_stick_y);
            pulley.setPower(gamepad2.right_stick_y);

            if(gamepad2.left_trigger == 1)
            {
                intake.setPower(-1);
            }

            else if(gamepad2.right_trigger == 1)
            {
                intake.setPower(1);

            }

            else
            {
                intake.setPower(0);
            }

            if(gamepad1.left_bumper)
            {
                runtime.reset();
                double R = runtime.seconds();
                double NFRBLSpeed;
                double NFLBRSpeed;

                if(R > 2)
                {
                    double ramp = R / 1.5;
                    NFRBLSpeed = 0.5 * ramp;

                }

                else
                {
                    NFRBLSpeed = 0.5;
                }

            }







            telemetry.addData("front left power", frontLeft.getPower());
            telemetry.addData("front right power", frontRight.getPower());
            telemetry.addData("back left power", backLeft.getPower());
            telemetry.addData("back right power", backRight.getPower());
            telemetry.addData("pulley power", pulley.getPower());
            telemetry.addData("actuator power", actuator.getPower());
            telemetry.addData("left stick x", -gamepad1.left_stick_x);
            telemetry.addData("left stick y", -gamepad1.left_stick_y);
            telemetry.addData("right stick x", -gamepad1.right_stick_x);
            telemetry.addData("right stick y", -gamepad1.right_stick_y);
            telemetry.update();

        }

    }
}