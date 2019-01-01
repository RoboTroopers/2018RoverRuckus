package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


//Version 1.4

@TeleOp(name = "Neumann Wheels Test", group = "Working")
public class NeumannsTest extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor pulley;
    private DcMotor actuator;
    private CRServo intake;
    private Servo latch;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        pulley = hardwareMap.get(DcMotor.class, "pulley");
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        intake = hardwareMap.get(CRServo.class, "intake");
        latch = hardwareMap.get(Servo.class, "latch");


        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            //YOU DIDLY DONE IT
        }

        while (opModeIsActive()) {

            frontLeft.setPower(-gamepad1.left_stick_y);
            backLeft.setPower(-gamepad1.left_stick_y);
            frontRight.setPower(-gamepad1.right_stick_y);
            backRight.setPower(-gamepad1.right_stick_y);

            pulley.setPower(-gamepad2.right_stick_y);
            actuator.setPower(-gamepad2.left_stick_y);

            if(gamepad2.left_trigger == 1)
            {
                intake.setPower(1);
            }

            else if(gamepad2.right_trigger == 1)
            {
                intake.setPower(-1);
            }

            else
            {
                intake.setPower(0);
            }

            if(gamepad2.right_bumper)
            {
                latch.setPosition(0.5);
            }

            else if(gamepad2.left_bumper)
            {
                latch.setPosition(0);
            }

            telemetry.addData("front left power", frontLeft.getPower());
            telemetry.update();

        }
    }
}