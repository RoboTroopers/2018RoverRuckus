package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Spice MemesTHETHIRDONE", group = "Working")
public class SpiceMemes extends LinearOpMode {

    private DcMotor        left_drive;
    private DcMotor        right_drive;
    private DcMotor        actuator;
    private CRServo        intake;
    private Servo          latch;
    private DcMotor        pulley;
    private DigitalChannel limitSwitch;

    static final double COUNTS_PER_MOTOR_REV = 1440; //counts per rotation for encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double ACTUATOR_GEAR_DIAMETER = 1.5;
    static final double PI = 3.14159265359;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 0.75;
    static final double LONG_DISTANCE_SPEED = 0.75;
    static final double ACTUATOR_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (ACTUATOR_GEAR_DIAMETER * PI);


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        left_drive  = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        actuator    = hardwareMap.get(DcMotor.class, "actuator");
        intake      = hardwareMap.get(CRServo.class, "intake");
        latch       = hardwareMap.get(Servo.class, "latch");
        pulley      = hardwareMap.get(DcMotor.class, "pulley");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");



        right_drive.setDirection(DcMotor.Direction.REVERSE);

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        if (opModeIsActive()) {
        }

        while (opModeIsActive())
        {

            left_drive.setPower(-gamepad1.left_stick_y);
            right_drive.setPower(-gamepad1.right_stick_y);
            actuator.setPower(gamepad2.left_stick_y);
            pulley.setPower(-gamepad2.right_stick_y);



            if(gamepad2.x){
                latch.setPosition(0.42);
            }


            else if(gamepad2.b){
                latch.setPosition(0.0);
            }



            if(gamepad2.right_trigger == 1.0)
            {
                intake.setPower(1.0);
            }

            else if(gamepad2.left_trigger == 1.0)
            {
                intake.setPower(-1.0);
            }

            else
            {
                intake.setPower(0.0);
            }


            limitSwitch.setMode(DigitalChannel.Mode.INPUT);

            if (limitSwitch.getState()) {
                // button is pressed.
                telemetry.addData("Button", "NOT PRESSED");
            } else {
                // button is not pressed.
                telemetry.addData("Button", "PRESSED");
            }

            telemetry.addData("ButtonValue", limitSwitch.getState());
            telemetry.addData("Left Pow", left_drive.getPower());
            telemetry.addData("Right Pow", right_drive.getPower());
            telemetry.addData("Actuator Pow", actuator.getPower());
            telemetry.addData("Pulley", pulley.getPower());
            telemetry.addData("Pulley Pos", pulley.getCurrentPosition());
            telemetry.addData("Latch Pos", latch.getPosition());
            telemetry.addData("Actuator Pos", actuator.getCurrentPosition());
            telemetry.addData("Actautor Inch Thrust", actuator.getCurrentPosition() / ACTUATOR_COUNTS_PER_INCH);
            telemetry.update();
        }
    }
}