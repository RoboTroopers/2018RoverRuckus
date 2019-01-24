package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;


@Autonomous(name = "Chungus Single Crater", group = "Working")

public class ChungusSingleCrater extends LinearOpMode {

    private ElapsedTime       runtime = new ElapsedTime();

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor actuator;
    private DcMotor intake;
    private Servo   outtake;
    private DcMotor pulley;
    private GoldAlignDetector detector;
    private DigitalChannel limitSwitch;



    private static final double COUNTS_PER_MOTOR_REV = 1440; //counts per rotation for encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double ACTUATOR_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double ACTUATOR_GEAR_DIAMETER = 1.5;
    private static final double PI = 3.14159265359;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    private static final double ACTUATOR_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * ACTUATOR_GEAR_REDUCTION) / (ACTUATOR_GEAR_DIAMETER * PI);



    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(Servo.class, "outtake");
        pulley = hardwareMap.get(DcMotor.class, "pulley");


        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();


        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);


        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 350; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 50; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment


        detector.enable(); // Start the detector!


        //wait for game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos", detector.getXPosition()); // Gold X position.
        telemetry.update();

        int POSVAR = 0;

        if(!limitSwitch.getState())
        {
            POSVAR = 1;
        }

        if(limitSwitch.getState() == true)
        {
            POSVAR = 2;
        }

        unlatch();

        allMotorsResetEncoder();
        allMotorsToPosition();

        mecaDrive(0.25, 7.5, 7.5, 5);
        strafe(0.25,-8,10);

        switch(POSVAR)
        {
            case 1:
                if (detector.getAligned()) {
                    intake.setPower(0.5);
                    mecaDrive(0.8,20,20,5);
                    intake.setPower(0);
                    requestOpModeStop();
                }

                else {
                    strafe(0.25,10,5);
                }

                if (detector.getAligned()) {
                    intake.setPower(0.5);
                    mecaDrive(0.8,20,20,5);
                    intake.setPower(0);
                    requestOpModeStop();
                }

                else {
                    strafe(0.25,11,5);
                    intake.setPower(0.5);
                    mecaDrive(0.8,20,20,5);
                    intake.setPower(0);
                    requestOpModeStop();
                }

                break;

            case 2:

                if (detector.getAligned()) {
                    intake.setPower(0.5);
                    mecaDrive(0.8,20,20,5);
                    intake.setPower(0);
                    mecaDrive(0.5,6,-6,5);
                    strafe(0.5,-5,5);
                    mecaDrive(0.5,12,12,5);
                    pulley.setPower(1);
                    outtake.setPosition(0.35);
                    sleep(1000);
                    outtake.setPosition(0);
                    sleep(500);
                    outtake.setPosition(0.35);
                    sleep(500);
                    pulley.setPower(-1);
                    outtake.setPosition(0.0);
                    sleep(500);
                    requestOpModeStop();
                }

                else {
                    strafe(0.25,10,5);
                }

                if (detector.getAligned()) {
                    intake.setPower(-1);
                    mecaDrive(0.8,15,15,5);
                    intake.setPower(0);
                    pulley.setPower(1);
                    outtake.setPosition(0.35);
                    sleep(1000);
                    outtake.setPosition(0);
                    sleep(500);
                    outtake.setPosition(0.35);
                    sleep(500);
                    pulley.setPower(-1);
                    outtake.setPosition(0.0);
                    sleep(500);
                    requestOpModeStop();
                }

                else {
                    strafe(0.25,11,5);
                    intake.setPower(-1);
                    mecaDrive(0.8,12,12,5);
                    intake.setPower(0);
                    mecaDrive(0.5,-6,6,5);
                    strafe(0.5,-5,5);
                    mecaDrive(0.5,12,12,5);
                    pulley.setPower(1);
                    outtake.setPosition(0.35);
                    sleep(1000);
                    outtake.setPosition(0);
                    sleep(500);
                    outtake.setPosition(0.35);
                    sleep(500);
                    pulley.setPower(-1);
                    outtake.setPosition(0.0);
                    sleep(500);
                    requestOpModeStop();
                }


        }
    }


    public void mecaDrive(double speed, double leftInches, double rightInches, double timeoutS){
        int newLeftTarget;
        int newRightTarget;

        if(opModeIsActive())
        {
            newLeftTarget = (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition())/2 + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = (frontRight.getCurrentPosition() + backRight.getCurrentPosition())/2 + (int)(rightInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newLeftTarget);
            backLeft.setTargetPosition(newLeftTarget);
            frontRight.setTargetPosition(newRightTarget);
            backRight.setTargetPosition(newRightTarget);

            allMotorsToPosition();

            runtime.reset();

            frontLeft.setPower((Math.abs(speed)));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            while(opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy()) && (frontRight.isBusy()) &&
                    (backLeft.isBusy()) && (backRight.isBusy()))
            {
                telemetry.addData("FrontLeftPower",frontLeft.getPower());
                telemetry.addData("FrontRightPower",frontRight.getPower());
                telemetry.addData("BackLeftPower",backLeft.getPower());
                telemetry.addData("BackRightPower",backRight.getPower());
                telemetry.update();
            }

            allMotorsZero();
            allMotorsResetEncoder();
            allMotorsRunUsing();


        }
    }


    public void strafe(double speed, double FRBLInches, int timeoutS) {
        int newFRBLTarget;
        int newFLBRTarget;
        double FLBRInches = FRBLInches * -1;

        allMotorsResetEncoder();


        if(opModeIsActive())
        {
            newFRBLTarget = (frontRight.getCurrentPosition() + backLeft.getCurrentPosition())/2 + (int)(FRBLInches * COUNTS_PER_INCH);
            newFLBRTarget = (frontLeft.getCurrentPosition() + backRight.getCurrentPosition())/2 + (int)(FLBRInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newFLBRTarget);
            backRight.setTargetPosition(newFLBRTarget);
            frontRight.setTargetPosition(newFRBLTarget);
            backLeft.setTargetPosition(newFRBLTarget);

            allMotorsToPosition();

            runtime.reset();

            frontRight.setPower(Math.abs(speed));
            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            while(opModeIsActive() && (runtime.seconds() < timeoutS)
                    && (frontLeft.isBusy()) && (frontRight.isBusy()) &&
                    (backLeft.isBusy()) && (backRight.isBusy()))
            {
                motorTelemetryPower();
            }

            allMotorsZero();
            allMotorsResetEncoder();
            allMotorsRunUsing();
        }
    }

    public void diagonal(double speed, double Inches, int direction, int timeoutS)
    {
        int newFLBRTarget;
        int newFRBLTarget;

        if(opModeIsActive())
        {
            switch (direction)
            {
                case 1:
                    newFLBRTarget = (frontLeft.getCurrentPosition() + backRight.getCurrentPosition())/2 + (int)(Inches * COUNTS_PER_INCH);

                    frontLeft.setTargetPosition(newFLBRTarget);
                    backRight.setTargetPosition(newFLBRTarget);

                    allMotorsToPosition();

                    runtime.reset();

                    frontLeft.setPower(Math.abs(speed));
                    backRight.setPower(Math.abs(speed));
                    frontRight.setPower(-0.01);
                    backLeft.setPower(-0.01);

                    while(opModeIsActive() && (runtime.seconds() < timeoutS) && (frontLeft.isBusy()) && (backRight.isBusy()))
                    {
                        motorTelemetryPower();
                    }

                    allMotorsZero();
                    allMotorsRunUsing();

                    break;

                case 2:
                    newFRBLTarget = (frontRight.getCurrentPosition() + backRight.getCurrentPosition())/2 + (int)(Inches * COUNTS_PER_INCH);

                    frontRight.setTargetPosition(newFRBLTarget);
                    backLeft.setTargetPosition(newFRBLTarget);

                    allMotorsToPosition();

                    runtime.reset();

                    frontRight.setPower(Math.abs(speed));
                    backLeft.setPower(Math.abs(speed));
                    frontLeft.setPower(-0.01);
                    backRight.setPower(-0.01);

                    while (opModeIsActive() && (runtime.seconds() < timeoutS) && (frontLeft.isBusy()) && (backLeft.isBusy()))
                    {
                        motorTelemetryPower();
                    }

                    allMotorsZero();
                    allMotorsRunUsing();

                    break;
            }

        }
    }



    private void motorTelemetryPower()
    {
        telemetry.addData("FrontLeftPower",frontLeft.getPower());
        telemetry.addData("FrontRightPower",frontRight.getPower());
        telemetry.addData("BackLeftPower",backLeft.getPower());
        telemetry.addData("BackRightPower",backRight.getPower());
        telemetry.update();
    }



    private void allMotorsResetEncoder()
    {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



    private void allMotorsZero()
    {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }



    private void allMotorsToPosition()
    {
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    private void allMotorsRunUsing()
    {
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    private void actuatorMovement(double speed, double inches, double timeoutS){

        int newActuatorTarget;

        if(opModeIsActive())
        {
            newActuatorTarget = actuator.getCurrentPosition() + (int)(inches * ACTUATOR_COUNTS_PER_INCH);
            actuator.setTargetPosition(newActuatorTarget);

            actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            actuator.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (actuator.isBusy()))
            {

                telemetry.addData("Path1","Running to %7d", newActuatorTarget);
                telemetry.addData("Path2","Running from %7d", actuator.getCurrentPosition());
                telemetry.update();
            }


            //Stop all motion
            actuator.setPower(0);

            //Turn off RUN_TO_POSITION
            actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // sleep(250); //optional pause after each move
        }
    }

    public void unlatch()
    {
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuatorMovement(1,-39,10);
        strafe(0.25,-2.5,5);
    }
}