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
package org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.Active;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.road_runner.OpModes.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.road_runner.master.drive.SampleMecanumDriveBase;
import com.acmerobotics.dashboard.canvas.Spline;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.LineSegment;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.QuinticSplineSegment;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.PathTrajectorySegment;
import com.acmerobotics.roadrunner.trajectory.PointTurn;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;

import java.util.Arrays;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.jetbrains.annotations.NotNull;


@Disabled
@Autonomous
public class PathOpModeTest extends LinearOpMode {

    // Declare drive and motors

    private SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

    private static DcMotor actuator;
    private static DcMotor pulley;
    private static DcMotor outtakePulley;
    private static CRServo intake;
    private static Servo   outtake;
    private static Servo   intakeRotate;
    private static GoldAlignDetector detector;
    private static DistanceSensor distanceSensor;

    // Declare variables

    private static String GoldPosition;

    private static final double intakePosition          = 0;
    private static final double retractPosition         = 0.5;
    private static final double transitionPosition      = 1;
    private static final double outtakeReadyPosition    = 0;
    private static final double outtakeMovementPosition = 0.15;
    private static final double outtakeDump             = 0.8;



    public void runOpMode()  {

        // Mapping the motors and drive class
        actuator       = hardwareMap.get(DcMotor.class, "actuator");
        pulley         = hardwareMap.get(DcMotor.class, "pulley");
        outtakePulley  = hardwareMap.get(DcMotor.class, "outtakePulley");
        intake         = hardwareMap.get(CRServo.class, "intake");
        outtake        = hardwareMap.get(Servo.class, "outtake");
        intakeRotate   = hardwareMap.get(Servo.class, "intakeRotate");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");


        // Make the paths, trajectories, splines, set up DogeCV, etc.

        String GoldPosition; // Used to check where the gold position is


        // Roadrunner set up

        Trajectory checkLeftSample = drive.trajectoryBuilder()
                .forward(2)
                .splineTo(new Pose2d(10,45,90))
                .build();

        Trajectory toDepot = drive.trajectoryBuilder()
                .splineTo(new Pose2d(-35,60, 180))
                .build();

        Trajectory toLeftSample = drive.trajectoryBuilder()
                .splineTo(new Pose2d(20,38,0))
                .turnTo(45)
                .build();

        Trajectory toMiddleSample = drive.trajectoryBuilder()
                .splineTo(new Pose2d(26,24,-45))
                .turnTo(45)
                .build();

        Trajectory toRightSample = drive.trajectoryBuilder()
                .splineTo(new Pose2d(36,14,-45))
                .turnTo(45)
                .build();



        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 500; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment


        detector.enable(); // Start the detector!


        waitForStart();

        if (isStopRequested()) return;

        // Type code that runs during the OpMode here

        unlatch();

        drive.followTrajectory(checkLeftSample);

        if(detector.getAligned())
        {
            GoldPosition = "l";
        }

        else {
            GoldPosition = "r";
        }

        drive.followTrajectory(toDepot);



        switch(GoldPosition)
        {
            case "l":
                drive.followTrajectory(toLeftSample);

                break;

            case "r":
                drive.followTrajectory(toRightSample);

                break;

            default:
                drive.followTrajectory(toMiddleSample);

                break;


        }

    }

    private void unlatch() {

        while(distanceSensor.getDistance(DistanceUnit.CM) > 2)
        {
            actuator.setPower(1);

            if(detector.getAligned())
            {
                GoldPosition = "m";
            }
        }

        actuator.setPower(0);

    }

    private void cycle(double extensionTime)
    {
        intakeP();
        outtake.setPosition(outtakeReadyPosition);
        sleep(200);
        intake.setPower(1);
        pulley.setPower(1);
        sleep(Math.round(extensionTime));
        pulley.setPower(0);

        transition();

        retract();

        intake.setPower(-1);
        sleep(250);
        outtake.setPosition(outtakeMovementPosition);

        upAndDump();
    }

    private void upAndDump()
    {
        outtakePulley.setPower(1);
        sleep(500);
        outtake.setPosition(outtakeDump);
        sleep(250);
    }

    private void dropOffMarker()
    {
        extendPulleyFull();

        intakeP();
        intake.setPower(-1);
        sleep(200);
        intake.setPower(0);

        retract();
    }

    private void intakeP()
    {
        intakeRotate.setPosition(intakePosition);
        sleep(200);
    }

    private void retract()
    {
        pulley.setPower(-1);
        intakeRotate.setPosition(retractPosition);
        sleep(650);
    }

    private void transition()
    {
        intakeRotate.setPosition(transitionPosition);
        sleep(200);
    }

    private void extendPulleyFull()
    {
        pulley.setPower(1);
        sleep(650);
        pulley.setPower(0);
    }
}