/*
 *
 *     Copyright (c) FTC Team 15167 Robo Troopers (http://robotroopers.org)
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

package org.firstinspires.ftc.teamcode.Old_Code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;




@Disabled
@Autonomous(name = "Literally PepeHands But Android", group = "Old")

public class FirstQualAuto extends LinearOpMode {

    private ElapsedTime       runtime = new ElapsedTime();

    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor actuator;
    private CRServo intake;
    private Servo   latch;
    private DcMotor pulley;
    private DigitalChannel limitSwitch;



    private static final double COUNTS_PER_MOTOR_REV = 1440; //counts per rotation for encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double ACTUATOR_GEAR_DIAMETER = 1.5;
    private static final double PI = 3.14159265359;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    private static final double DRIVE_SPEED = 1.0;
    private static final double TURN_SPEED = 0.75;
    private static final double LONG_DISTANCE_SPEED = 0.75;
    private static final double ACTUATOR_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (ACTUATOR_GEAR_DIAMETER * PI);
    @Override

    public void runOpMode()
    {

        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        intake = hardwareMap.get(CRServo.class, "intake");
        latch = hardwareMap.get(Servo.class, "latch");
        pulley = hardwareMap.get(DcMotor.class, "pulley");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");


        telemetry.addData("Status", "Resetting Encoders" );
        telemetry.update();


        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_drive.setDirection(DcMotor.Direction.REVERSE);

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);


        //wait for game to start (driver presses PLAY)
        waitForStart();


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //copyPasteDrive(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup)
        //actuatorMovement(DRIVE_SPEED, 6.5, 4.0);g
        //latch.setPosition(.45);
        //sleep(1000);
    /*encoderDrive(LONG_DISTANCE_SPEED , (7.8*Math.sqrt(2))+4 , (7.8*Math.sqrt(2))+3 , 5.0); //goes forward to the sampling minerals
    encoderDrive(TURN_SPEED,  -(3.5*PI)+1 , (3.5*PI)+0 , 4.0);
    encoderDrive(LONG_DISTANCE_SPEED , (16.5*Math.sqrt(2))+2.5 , (16.5*Math.sqrt(2))+1.5 , 4.0);
    encoderDrive(TURN_SPEED, -(3.42*PI)+1 , (3.42*PI) , 4.0);
    encoderDrive(LONG_DISTANCE_SPEED , 61 , 58 , 5.0);
    encoderDrive(TURN_SPEED, -4 , 0 , 5.0);
    pulley.setPower(1);
    sleep(2000);
    pulley.setPower(0);
    intake.setPower(1);
    sleep(5000);
    intake.setPower(0);
    encoderDrive(LONG_DISTANCE_SPEED , -70 , -72 , 4.0);
    encoderDrive(TURN_SPEED, -(3*PI)+1 , (3*PI) , 4.0);
    encoderDrive(DRIVE_SPEED, -52, -50, 4.0);*/
    /*actuatorMovement(DRIVE_SPEED , -6.5 , 4.0);
    latch.setPosition(0.42);
    sleep(2000);
    copyPasteDrive(DRIVE_SPEED, DRIVE_SPEED, 24 , 10 , 2 );
    encoderDrive(TURN_SPEED , -(3.5*PI)+1 , (3.5*PI)+0 , 4.0);
    copyPasteDrive(DRIVE_SPEED, DRIVE_SPEED, 24 , 10 , 2);
    encoderDrive(TURN_SPEED , -(3.5 * PI)+1 , (3.5 * PI) , 4.0);
    copyPasteDrive(DRIVE_SPEED, DRIVE_SPEED, 72 , 10 , 2);
    intake.setPower(1);
    sleep(5000);
    intake.setPower(0);
    copyPasteDrive(DRIVE_SPEED , DRIVE_SPEED , -60 , 10 , 2);
    encoderDrive(TURN_SPEED, -(3*PI)+1 , (3*PI) , 4.0);
    encoderDrive(DRIVE_SPEED, -52, -50, 4.0);
    */

        //Pepega(double Lspeed, double Rspeed, double Inches,  double timeoutS, double rampup,
        // int leftDirection, int rightDirection)

        int POSVAR = 0;

        if(limitSwitch.getState())
        {
            POSVAR = 1;
        }

        if(!limitSwitch.getState())
        {
            POSVAR = 2;
        }

        switch(POSVAR)
        {
            case 1:                                                  //facing crater

                limitSwitch.setMode(DigitalChannel.Mode.INPUT);



                if (limitSwitch.getState() == true)
                {
                    // button is pressed.
                    telemetry.addData("Button", "NOT PRESSED");
                    telemetry.update();
                }

                else
                {
                    // button is not pressed.
                    telemetry.addData("Button", "PRESSED");
                    telemetry.update();
                }

                actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                actuatorMovement(1,-23,10);
                latch.setPosition(0.42);
                sleep(1200);
                actuatorMovement(1,8,10);

                Pepega(DRIVE_SPEED,DRIVE_SPEED,60,10,0,1,1);                // drive straight toward crater
                Pepega(TURN_SPEED,TURN_SPEED,(3*PI),10,2,-1,1);           // turn 90 degrees left
                Pepega(DRIVE_SPEED,DRIVE_SPEED,26,10,0.5,1,1);                // go forward toward wall
                Pepega(TURN_SPEED,TURN_SPEED,(((3*PI)+2.5)/2),10,3,-1,1);     // turn 45 degrees right for last drive towards depot
                Pepega(DRIVE_SPEED,0.925,45,10,0.5,1,1);                  // drive toward and into depot
                pulley.setPower(1);                                           // raise pulley
                intake.setPower(1);                                           // drop marker
                sleep(500);                                                  // runs LINES 153 & 126 for 2000 MS (2 seconds)
                pulley.setPower(0);                                           // stops pulley
                intake.setPower(0);                                           // stops intake
                Pepega(0.68,DRIVE_SPEED,73,10,0.5,-1,-1);
                //Pepega(DRIVE_SPEED,DRIVE_SPEED,(1*((3*PI))),10,0,1,-1);
                //Pepega(DRIVE_SPEED,DRIVE_SPEED,6.5,3,0,-1,-1);
                Pepega(DRIVE_SPEED,DRIVE_SPEED,(3*PI),10,0,1,-1);
                Pepega(DRIVE_SPEED,DRIVE_SPEED,45,10,0,1,1);


                break;

            case 2:

                limitSwitch.setMode(DigitalChannel.Mode.INPUT);

                if (limitSwitch.getState() == true)
                {
                    // button is pressed.
                    telemetry.addData("Button", "NOT PRESSED");


                }

                else
                {
                    // button is not pressed.
                    telemetry.addData("Button", "PRESSED");

                }
                telemetry.addData("Touch Sensor Data", limitSwitch.getState());
                telemetry.update();

                actuator.setMode(DcMotor.RunMode.RESET_ENCODERS);
                actuatorMovement(1,-23,10);
                latch.setPosition(0.42);
                sleep(1200);
                actuatorMovement(1,8,10);

                Pepega(DRIVE_SPEED,DRIVE_SPEED, 55 , 10 , 1.5 , 1 , 1 );

                pulley.setPower(1);                                           // raise pulley
                intake.setPower(1);                                           // drop marker
                sleep(500);                                                  // runs LINES 125 & 126 for 2000 MS (2 seconds)
                pulley.setPower(0);                                           // stops pulley
                intake.setPower(0);                                          // stops intake
                Pepega(TURN_SPEED,TURN_SPEED, 1.8*(3*PI) , 10 , 1.5 , 1 ,-1);
                Pepega(0.75,DRIVE_SPEED,68,10,0.5,1,1);
                Pepega(1,1,(3*PI)-3,5,0,1,-1);
                Pepega(1,1,(3*PI),5,0,1,1);  // straight
                Pepega(1,1,(3*PI)+4,5,0,-1,1);
                Pepega(1,1,40,10,0,1,1);
                //Pepega(DRIVE_SPEED,DRIVE_SPEED,(1*((3*PI))),10,0,1,-1);
                //Pepega(DRIVE_SPEED,DRIVE_SPEED,6.5,3,0,-1,-1);
                //Pepega(DRIVE_SPEED,DRIVE_SPEED,(3*PI),10,0,-1,1);
                //Pepega(DRIVE_SPEED,DRIVE_SPEED,50,10,0,1,1);
                break;
        }


        //Pepega(1,1,(5/2)*((3*PI)+1.0),10,0,-1,1);
    /*Pepega(DRIVE_SPEED,DRIVE_SPEED,(((3*PI)+0.5)/2),10,0.5,1,-1); // turns 45 degrees right and aims towards mid of lander and crater
    Pepega(DRIVE_SPEED,DRIVE_SPEED,30,10,0.5,-1,-1);                // goes backwards towards mid between lander and crater
    Pepega(DRIVE_SPEED,DRIVE_SPEED,(((3*PI)+8)/2),10,0.5,-1,1);     // turns 45 degrees right and aims for crater
    Pepega(DRIVE_SPEED,DRIVE_SPEED,44,10,0,-1,-1);                  // fully backwards into crater
    */


        //encoderDrive(TURN_SPEED,  -(3.5*PI)+1 , (3.5*PI)+0 , 4.0);
        //copyPasteDrive(TURN_SPEED , TURN_SPEED , -24, 24 , 4.0 );
    /*copyPasteDrive(LONG_DISTANCE_SPEED , LONG_DISTANCE_SPEED , (16.5*Math.sqrt(2))+2.5 , (16.5*Math.sqrt(2))+1.5 , 4.0 );
    copyPasteDrive(TURN_SPEED , TURN_SPEED, -(3.42*PI)+1 , (3.42*PI) , 4.0 );
    copyPasteDrive(LONG_DISTANCE_SPEED , LONG_DISTANCE_SPEED , 61 , 58 , 5.0);
    copyPasteDrive(TURN_SPEED , TURN_SPEED ,  -4 , 0 , 5.0);
    intake.setPower(-1.0);
    sleep(3000);
    copyPasteDrive(LONG_DISTANCE_SPEED , LONG_DISTANCE_SPEED ,  -70 , -72 , 4.0);
    copyPasteDrive(TURN_SPEED , TURN_SPEED ,  -(3*PI)+1 , (3*PI) , 4.0);
    copyPasteDrive(DRIVE_SPEED , DRIVE_SPEED ,  -52 , -50 , 4.0);*/




        telemetry.addData("Path", "Complete" );
        telemetry.update();

    }


    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS){
        int newLeftTarget;
        int newRightTarget;



        if(opModeIsActive())
        {
            newLeftTarget = left_drive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = right_drive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            left_drive.setTargetPosition(newLeftTarget);
            right_drive.setTargetPosition(newRightTarget);

            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            left_drive.setPower(Math.abs(speed));
            right_drive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (left_drive.isBusy() && right_drive.isBusy()))
            {

                telemetry.addData("Path1","Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2","Running from %7d :%7d",
                        left_drive.getCurrentPosition(),
                        right_drive.getCurrentPosition());
                telemetry.update();
            }


            //Stop all motion
            left_drive.setPower(0);
            right_drive.setPower(0);

            //Turn off RUN_TO_POSITION
            left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // sleep(250); //optional pause after each move
        }
    }



    public void actuatorMovement(double speed, double inches, double timeoutS){

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
                telemetry.addData("Path2","Running from %7d",
                        actuator.getCurrentPosition());
                telemetry.update();
            }


            //Stop all motion
            actuator.setPower(0);

            //Turn off RUN_TO_POSITION
            actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // sleep(250); //optional pause after each move
        }
    }

    public void spiceDrive(double speed, double leftInches, double rightInches, double timeoutS){
        int newLeftTarget;
        int newRightTarget;



        if(opModeIsActive())
        {

            newLeftTarget = left_drive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = right_drive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);


            left_drive.setTargetPosition(newLeftTarget);
            right_drive.setTargetPosition(newRightTarget);

            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(opModeIsActive()) {

                if((Math.abs(newRightTarget)) - Math.abs(right_drive.getCurrentPosition())
                        >= (4/5)*(newRightTarget)){
                    //left_drive.setPower(Math.abs((1/10)*speed));
                    //right_drive.setPower(Math.abs((1/10)*speed));
                    newLeftTarget = left_drive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                    newRightTarget = right_drive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

                    left_drive.setTargetPosition(newLeftTarget);
                    right_drive.setTargetPosition(newRightTarget);

                    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    left_drive.setPower(Math.abs((1/10)*speed));
                    right_drive.setPower(Math.abs((1/10)*speed));

                    runtime.reset();
                }

                else if(Math.abs(newRightTarget) - Math.abs(right_drive.getCurrentPosition()) <= (1/4)*(newRightTarget)) {

                    //left_drive.setPower(Math.abs((1/10)*speed));
                    //right_drive.setPower(Math.abs((1/10)*speed));
                    newLeftTarget = left_drive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                    newRightTarget = right_drive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

                    left_drive.setTargetPosition(newLeftTarget);
                    right_drive.setTargetPosition(newRightTarget);

                    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    left_drive.setPower(Math.abs((1/10)*speed));
                    right_drive.setPower(Math.abs((1/10)*speed));
                }

                else {
                    //left_drive.setPower(Math.abs(speed));
                    //right_drive.setPower(Math.abs(speed));

                    left_drive.setTargetPosition(newLeftTarget);
                    right_drive.setTargetPosition(newRightTarget);

                    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    left_drive.setPower(Math.abs((1/10)*speed));
                    right_drive.setPower(Math.abs((1/10)*speed));

                }
            }
            //runtime.reset();
            //left_drive.setPower(Math.abs(speed));
            //right_drive.setPower(Math.abs(speed));



            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (left_drive.isBusy() && right_drive.isBusy()))
            {

                telemetry.addData("Path1","Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2","Running to %7d :%7d",
                        left_drive.getCurrentPosition(),
                        right_drive.getCurrentPosition());
                telemetry.update();
            }


            //Stop all motion
            left_drive.setPower(0);
            right_drive.setPower(0);

            //Turn off RUN_TO_POSITION
            left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // sleep(250); //optional pause after each move
        }
    }

    public void weSuck(double slowSpeed, double normalSpeed,  double leftInches, double rightInches, double timeoutS){
        int    newEndLeftTarget;
        int    newEndRightTarget;
        int    new1FourthLeftTarget;
        int    new1FourthRightTarget;
        int    new3FourthLeftTarget;
        int    new3FourthRightTarget;



        if(opModeIsActive())
        {
            newEndLeftTarget = left_drive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newEndRightTarget = right_drive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            new1FourthLeftTarget = left_drive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH) * (1/4);
            new1FourthRightTarget = right_drive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH) * (1/4);
            new3FourthLeftTarget = left_drive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH) * (3/4);
            new3FourthRightTarget = right_drive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH) * (3/4);


            left_drive.setTargetPosition(new1FourthLeftTarget);
            right_drive.setTargetPosition(new1FourthRightTarget);

            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            left_drive.setPower(Math.abs(slowSpeed));
            right_drive.setPower(Math.abs(slowSpeed));

            left_drive.setTargetPosition(new3FourthLeftTarget);
            right_drive.setTargetPosition(new3FourthRightTarget);

            left_drive.setPower(Math.abs(normalSpeed));
            right_drive.setPower(Math.abs(normalSpeed));

            left_drive.setTargetPosition(newEndLeftTarget);
            right_drive.setTargetPosition(newEndRightTarget);

            left_drive.setPower(Math.abs(slowSpeed));
            right_drive.setPower(Math.abs(slowSpeed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (left_drive.isBusy() && right_drive.isBusy()))
            {

                telemetry.addData("Target","Running to %7d :%7d", newEndLeftTarget, newEndRightTarget);
                telemetry.addData("Completion Rate","Running from %7d :%7d",
                        left_drive.getCurrentPosition(),
                        right_drive.getCurrentPosition());

                telemetry.update();


            }


            //Stop all motion
            left_drive.setPower(0);
            right_drive.setPower(0);

            //Turn off RUN_TO_POSITION
            left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // sleep(250); //optional pause after each move
        }
    }



    public void testDrive(double speed, double leftInches, double rightInches, double AccelerationInches, int Direction) {
        // Declares variables that are used for this method
        int NewLeftTarget;
        int NewRightTarget;
        int RightPosition;
        int LeftPosition;
        // Resets encoders to 0
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            // Calculates the needed encoder ticks by multiplying a pre-determined amount of CountsPerInches,
            // and the method input gets the actual distance travel in inches
            NewLeftTarget = left_drive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            NewRightTarget = right_drive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            // Gets the current position of the encoders at the beginning of the EncoderDrive method
            RightPosition = right_drive.getCurrentPosition();
            LeftPosition = left_drive.getCurrentPosition();
            // Gives the encoders the target.
            left_drive.setTargetPosition(NewLeftTarget);
            right_drive.setTargetPosition(NewRightTarget);

            // Turn On RUN_TO_POSITION
            left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();
            // This gets where the motor encoders will be at full position when it will be at full speed.
            double LeftEncoderPositionAtFullSpeed = ((AccelerationInches*(COUNTS_PER_INCH))+ LeftPosition);
            double RightEncoderPositionAtFullSpeed = ((AccelerationInches*(COUNTS_PER_INCH) + RightPosition));


            // This gets the absolute value of the encoder positions at full speed - the current speed, and while it's greater than 0, it will continues increasing the speed.
            // This allows the robot to accelerate over a set number of inches, which reduces wheel slippage and increases overall reliability
            while ((Math.abs(LeftEncoderPositionAtFullSpeed) - (Math.abs(left_drive.getCurrentPosition())) > 150) && (Math.abs(RightEncoderPositionAtFullSpeed)-(Math.abs(right_drive.getCurrentPosition()))) > 200 && opModeIsActive()) {
                // This allows the robot to accelerate over a set distance, rather than going full speed.  This reduces wheel slippage and increases reliability.
                left_drive.setPower(Range.clip(Math.abs(left_drive.getCurrentPosition()
                        / LeftEncoderPositionAtFullSpeed), .15*Direction, speed*Direction));
                right_drive.setPower(Range.clip(Math.abs(right_drive.getCurrentPosition()
                        / RightEncoderPositionAtFullSpeed), .15*Direction, speed*Direction));
                telemetry.addData("Accelerating", right_drive.getPower());
            }
            // Multiplies the speed desired by the direction, which has a value of either 1, or -1, and allows for backwards driving with the ramp up function
            left_drive.setPower((speed*Direction));
            right_drive.setPower((speed*Direction));
            // While encoders are not at position
            while (right_drive.isBusy() && left_drive.isBusy() && opModeIsActive()){
                // Display information for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", NewLeftTarget, NewRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", left_drive.getCurrentPosition(), right_drive.getCurrentPosition());

            }

            // Stops all motion
            // Set to run without encoder, so it's not necessary to declare this every time after the method is used
            left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Set power to 0
            left_drive.setPower(0);
            right_drive.setPower(0);

        }
    }


    public void copyPasteDrive(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup) {
        //initialise some variables for the subroutine
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
        newLeftTarget = left_drive.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
        newRightTarget = right_drive.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);

        // reset the timeout time and start motion.
        runtime.reset();

        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (Math.abs(left_drive.getCurrentPosition()) < newLeftTarget)  &&
                (Math.abs(right_drive.getCurrentPosition()) < newRightTarget)) {
            double rem = (Math.abs(left_drive.getCurrentPosition()) +
                    Math.abs(right_drive.getCurrentPosition()))/2;
            double NLspeed;
            double NRspeed;

            //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over
            //the amount of time you set to rampup
            double R = runtime.seconds();
            if (R < rampup) {
                double ramp = R / rampup;
                NLspeed = Lspeed * ramp;
                NRspeed = Rspeed * ramp;
            }

//if rampup time has passed, use set speed
            else {
                NLspeed = Lspeed;
                NRspeed = Rspeed;
            }

            //Pass the seed values to the motors

            left_drive.setPower(NLspeed);
            right_drive.setPower(NRspeed);
        }

        // Stop all motion;
        //Note: This is outside our while statement, this will only activate once the time, or distance has been met
        left_drive.setPower(0);
        right_drive.setPower(0);

        // show the driver how close they got to the last target
        telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
        telemetry.addData("Path2",  "Running at %7d :%7d", left_drive.getCurrentPosition(), right_drive.getCurrentPosition());
        telemetry.update();

        //setting resetC as a way to check the current encoder values easily
        double resetC = (Math.abs(left_drive.getCurrentPosition())
                + Math.abs(right_drive.getCurrentPosition()));

        //Get the motor encoder resets in motion
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //keep waiting while the reset is running
        while (Math.abs(resetC) > 0){
            left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            resetC =  (Math.abs(left_drive.getCurrentPosition())
                    + Math.abs(right_drive.getCurrentPosition()));
            //waitOneFullHardwareCycle();
        }
        // switch the motors back to RUN_USING_ENCODER mode
        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//give the encoders a chance to switch modes.
//waitOneFullHardwareCycle();
    }

    public void Pepega(double Lspeed, double Rspeed, double Inches,  double timeoutS, double rampup,
                       int leftDirection, int rightDirection) {
        //initialise some variables for the subroutine
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
        newLeftTarget = left_drive.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
        newRightTarget = right_drive.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);

        // reset the timeout time and start motion.
        runtime.reset();

        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (Math.abs(left_drive.getCurrentPosition()) < newLeftTarget)  &&
                (Math.abs(right_drive.getCurrentPosition()) < newRightTarget)) {
            double rem = (Math.abs(left_drive.getCurrentPosition()) +
                    Math.abs(right_drive.getCurrentPosition()))/2;
            double NLspeed;
            double NRspeed;

            //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over
            //the amount of time you set to rampup
            double R = runtime.seconds();
            if (R < rampup) {
                double ramp = R / rampup;
                NLspeed = Lspeed * ramp;
                NRspeed = Rspeed * ramp;
            }

//if rampup time has passed, use set speed
            else {
                NLspeed = Lspeed;
                NRspeed = Rspeed;
            }

            //Pass the seed values to the motors

            left_drive.setPower(NLspeed * leftDirection);
            right_drive.setPower(NRspeed * rightDirection);
        }

        // Stop all motion;
        //Note: This is outside our while statement, this will only activate once the time, or distance has been met
        left_drive.setPower(0);
        right_drive.setPower(0);

        // show the driver how close they got to the last target
        telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
        telemetry.addData("Path2",  "Running at %7d :%7d", left_drive.getCurrentPosition(), right_drive.getCurrentPosition());
        telemetry.update();

        //setting resetC as a way to check the current encoder values easily
        double resetC = (Math.abs(left_drive.getCurrentPosition())
                + Math.abs(right_drive.getCurrentPosition()));

        //Get the motor encoder resets in motion
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //keep waiting while the reset is running
        while (Math.abs(resetC) > 0){
            left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            resetC =  (Math.abs(left_drive.getCurrentPosition())
                    + Math.abs(right_drive.getCurrentPosition()));
            //waitOneFullHardwareCycle();
        }
        // switch the motors back to RUN_USING_ENCODER mode
        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//give the encoders a chance to switch modes.
//waitOneFullHardwareCycle();
    }

    public void SodiumChloride(double Lspeed, double Rspeed, double Inches,  double timeoutS, double rampup,
                               int leftDirection, int rightDirection) {
        //initialise some variables for the subroutine
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
        newLeftTarget = left_drive.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
        newRightTarget = right_drive.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);

        // reset the timeout time and start motion.
        runtime.reset();

        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (Math.abs(left_drive.getCurrentPosition()) < newLeftTarget)  &&
                (Math.abs(right_drive.getCurrentPosition()) < newRightTarget))
        {
            double rem = (Math.abs(left_drive.getCurrentPosition()) +
                    Math.abs(right_drive.getCurrentPosition()))/2;
            double NLspeed;
            double NRspeed;

            //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over
            //the amount of time you set to rampup
            double R = runtime.seconds();
            if (R < rampup) {
                double ramp = R / rampup;
                NLspeed = Lspeed * ramp;
                NRspeed = Rspeed * ramp;
            }

//if rampup time has passed, use set speed
            else  {
                NLspeed = Lspeed;
                NRspeed = Rspeed;
            }



            //Pass the seed values to the motors

            left_drive.setPower(NLspeed * leftDirection);
            right_drive.setPower(NRspeed * rightDirection);

            while(left_drive.isBusy() && right_drive.isBusy())
            {
                if(left_drive.getCurrentPosition() > (3/4) * newLeftTarget) // if the current position is 3/4 of way there...
                {                                                           // if it is not jump to line 712

                    while(left_drive.getCurrentPosition() > (3/4) * newLeftTarget) // then it will do this
                    {

                        //NLspeed -= 0.05;  // reduce both NEW speeds by 0.05
                        //NRspeed -= 0.05;  // have to test this out so commented out

                        left_drive.setPower((NLspeed -= 0.05) * leftDirection); // lowers left and right speed by 0.05 per sec
                        right_drive.setPower((NRspeed -= 0.05) * rightDirection);

                        if(left_drive.getPower() < 0.4 && right_drive.getPower() < 0.4) // while the power is reducing ...
                        {    // if it goes below 4/10 of max power, then it will change and stay at 4/10 of max power
                            NLspeed = 0.4;
                            NRspeed = 0.4;
                            left_drive.setPower(NLspeed * leftDirection);
                            right_drive.setPower(NRspeed * rightDirection);
                        }

                    }

                }

                else
                {
                    left_drive.setPower(NLspeed * leftDirection);
                    right_drive.setPower(NRspeed * rightDirection);
                }
            }
        }

        // Stop all motion;
        //Note: This is outside our while statement, this will only activate once the time, or distance has been met
        left_drive.setPower(0);
        right_drive.setPower(0);

        // show the driver how close they got to the last target
        telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
        telemetry.addData("Path2",  "Running at %7d :%7d", left_drive.getCurrentPosition(), right_drive.getCurrentPosition());
        telemetry.update();

        //setting resetC as a way to check the current encoder values easily
        double resetC = (Math.abs(left_drive.getCurrentPosition())
                + Math.abs(right_drive.getCurrentPosition()));

        //Get the motor encoder resets in motion
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //keep waiting while the reset is running
        while (Math.abs(resetC) > 0){
            left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            resetC =  (Math.abs(left_drive.getCurrentPosition())
                    + Math.abs(right_drive.getCurrentPosition()));
            //waitOneFullHardwareCycle();
        }
        // switch the motors back to RUN_USING_ENCODER mode
        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//give the encoders a chance to switch modes.
//waitOneFullHardwareCycle();
    }
}