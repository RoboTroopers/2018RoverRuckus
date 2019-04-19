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


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.path.LineSegment;
import com.acmerobotics.roadrunner.path.QuinticSplineSegment;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.path.heading.WiggleInterpolator;
import com.acmerobotics.roadrunner.trajectory.PathTrajectorySegment;
import com.acmerobotics.roadrunner.trajectory.PointTurn;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.roadrunner.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.roadrunner.drive.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.roadrunner.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.roadrunner.drive.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.roadrunner.util.AssetsTrajectoryLoader;
import org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.roadrunner.util.DashboardUtil;

import java.util.Arrays;

import static java.lang.Math.PI;
import static java.lang.Math.round;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Autonomous(name = "Ugandan Crater", group = "ZZZ")
public class Auto_Crater extends LinearOpMode {

    private DcMotor actuator;
    private DcMotor pulley;
    private DcMotor outtakePulley;
    private Servo   outtake;
    private Servo   intakeRotate;
    private Servo   pepeJAM;
    private CRServo intake;
    private GoldAlignDetector detector;
    private DigitalChannel limitSwitch;


    private static String GoldPosition = "";
    private static final double PI = 355.0/113.0;
    private static final double dumpPos = 0.603;

    public static double oof(double degrees)
    {
        return Math.toRadians(degrees);
    }


    @Override
    public void runOpMode() {

        outtakePulley = hardwareMap.get(DcMotor.class, "outtakePulley");
        outtake       = hardwareMap.get(Servo.class, "outtake");
        actuator      = hardwareMap.get(DcMotor.class, "actuator");
        intake        = hardwareMap.get(CRServo.class, "intake");
        pulley        = hardwareMap.get(DcMotor.class, "pulley");
        intakeRotate  = hardwareMap.get(Servo.class, "intakeRotate");
        pepeJAM        = hardwareMap.get(Servo.class, "pepeJAM");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 500; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 300; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment


        detector.enable(); // Start the detector!




        FtcDashboard dashboard = FtcDashboard.getInstance();
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);


        // Declare trajectories

        Trajectory exitLZ = new TrajectoryBuilder(new Pose2d(-12, -12, oof(315)), DriveConstants.BASE_CONSTRAINTS)
                //.lineTo(new Vector2d(-8,-17))
                .forward(3)
                .strafeRight(28)
                .splineTo(new Pose2d(-10,-45, oof(280)))
                .strafeRight(5)
                .turnTo(oof(270))
                .build();

        Trajectory depot = new TrajectoryBuilder(new Pose2d(-10,-50, oof(270)), DriveConstants.BASE_CONSTRAINTS)
                .turnTo(oof(345.5))
                .strafeRight(22)
                .forward(10)
                .strafeLeft(4)
                .waitFor(0.5)
                .build();

        Trajectory toLeft = new TrajectoryBuilder(new Pose2d(0,-68,0), DriveConstants.BASE_CONSTRAINTS)
                .reverse()
                .splineTo(new Pose2d(-24,-50))
                .turnTo(oof(175))
                .reverse()
                .strafeLeft(4)
                .forward(18)
                .build();

        Trajectory toMiddle = new TrajectoryBuilder(new Pose2d(0,-68,0), DriveConstants.BASE_CONSTRAINTS)
                .back(18)
                .turnTo(oof(317))
                .strafeLeft(12)
                .back(38)
                .turnTo(oof(240))
                .forward(14)
                .build();

        Trajectory toRight = new TrajectoryBuilder(new Pose2d(0,-68,0), DriveConstants.BASE_CONSTRAINTS)
                .back(18)
                .turnTo(oof(321))
                .strafeLeft(10)
                .back(46)
                .turnTo(oof(240))
                .forward(14)
                .build();


        /**
         * DEPOT TRAJECTORIES STARTS HERE
         */

        Trajectory toLeftDepot = new TrajectoryBuilder(new Pose2d(12,-12, oof(45)), DriveConstants.BASE_CONSTRAINTS)
                .forward(3)
                .strafeRight(12)
                .build();

        Trajectory toRightMarker = new TrajectoryBuilder((drive.getPoseEstimate()), DriveConstants.BASE_CONSTRAINTS)
                .back(24)
                .turnTo(oof(315))
                .forward(18)
                .build();

        Trajectory toMiddleMarker = new TrajectoryBuilder(new Pose2d(12,-12, oof(45)), DriveConstants.BASE_CONSTRAINTS)
                .forward(3)
                .strafeRight(12)
                .back(5)
                .turnTo(oof(315))
                .forward(18)
                .build();

        Trajectory middlePark = new TrajectoryBuilder((drive.getPoseEstimate()), DriveConstants.BASE_CONSTRAINTS)
                .back(18)
                .turnTo(oof(45))
                .forward(36)
                .turnTo(oof(90))
                .forward(20)
                .build();


        waitForStart();

        //if (isStopRequested()) return;

        int POSVAR = 0;

        if(limitSwitch.getState())
        {
            POSVAR = 1;
        }

        if(!limitSwitch.getState())
        {
            POSVAR = 2;
        }

        switch (POSVAR)
        {
            case 1:

                unhang();

                sleep(500);

                //actuator.setPower(-0.5);
                //sleep(200);

                if(detector.getAligned())
                {
                    GoldPosition = "M";
                }


                drive.followTrajectory(exitLZ);


                while (!isStopRequested() && drive.isFollowingTrajectory()) {
                    Pose2d currentPose = drive.getPoseEstimate();

                    TelemetryPacket packet = new TelemetryPacket();
                    Canvas fieldOverlay = packet.fieldOverlay();

                    packet.put("x", currentPose.getX());
                    packet.put("y", currentPose.getY());
                    packet.put("heading", currentPose.getHeading() * (180/ PI));

                    fieldOverlay.setStrokeWidth(4);
                    fieldOverlay.setStroke("green");
                    DashboardUtil.drawSampledTrajectory(fieldOverlay, exitLZ);

                    fieldOverlay.setFill("blue");
                    fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                    dashboard.sendTelemetryPacket(packet);

                    drive.update();
                }

                if(detector.getAligned() && !GoldPosition.equals("M"))
                {
                    GoldPosition = "L";
                }

                telemetry.addData("Gold Position", GoldPosition);
                telemetry.update();

                drive.followTrajectory(depot);

                while (!isStopRequested() && drive.isFollowingTrajectory()) {
                    Pose2d currentPose = drive.getPoseEstimate();

                    TelemetryPacket packet = new TelemetryPacket();
                    Canvas fieldOverlay = packet.fieldOverlay();

                    packet.put("x", currentPose.getX());
                    packet.put("y", currentPose.getY());
                    packet.put("heading", currentPose.getHeading() * (180/ PI));

                    fieldOverlay.setStrokeWidth(4);
                    fieldOverlay.setStroke("green");
                    DashboardUtil.drawSampledTrajectory(fieldOverlay, depot);

                    fieldOverlay.setFill("blue");
                    fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                    dashboard.sendTelemetryPacket(packet);

                    drive.update();
                }

                pulley.setPower(1);
                sleep(1250);

                intakeRotate.setPosition(dumpPos);
                intake.setPower(1);
                pulley.setPower(0.25);
                sleep(1250);

                pulley.setPower(-1);
                intake.setPower(0);
                intakeRotate.setPosition(0);
                sleep(750);



                switch (GoldPosition)
                {
                    case "L":

                        drive.followTrajectory(toLeft);

                        while (!isStopRequested() && drive.isFollowingTrajectory()) {
                            Pose2d currentPose = drive.getPoseEstimate();

                            TelemetryPacket packet = new TelemetryPacket();
                            Canvas fieldOverlay = packet.fieldOverlay();

                            packet.put("x", currentPose.getX());
                            packet.put("y", currentPose.getY());
                            packet.put("heading", currentPose.getHeading() * (180/ PI));

                            fieldOverlay.setStrokeWidth(4);
                            fieldOverlay.setStroke("green");
                            DashboardUtil.drawSampledTrajectory(fieldOverlay, toLeft);

                            fieldOverlay.setFill("blue");
                            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                            dashboard.sendTelemetryPacket(packet);

                            drive.update();
                        }

                        pulley.setPower(1);
                        sleep(2000);
                        pulley.setPower(0);
                        intakeRotate.setPosition(dumpPos);
                        sleep(200);


                        break;

                    case "M":

                        drive.followTrajectory(toMiddle);

                        while (!isStopRequested() && drive.isFollowingTrajectory()) {
                            Pose2d currentPose = drive.getPoseEstimate();

                            TelemetryPacket packet = new TelemetryPacket();
                            Canvas fieldOverlay = packet.fieldOverlay();

                            packet.put("x", currentPose.getX());
                            packet.put("y", currentPose.getY());
                            packet.put("heading", currentPose.getHeading() * (180/ PI));

                            fieldOverlay.setStrokeWidth(4);
                            fieldOverlay.setStroke("green");
                            DashboardUtil.drawSampledTrajectory(fieldOverlay, toMiddle);

                            fieldOverlay.setFill("blue");
                            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                            dashboard.sendTelemetryPacket(packet);

                            drive.update();
                        }

                        pulley.setPower(1);
                        sleep(2000);
                        pulley.setPower(0);
                        intakeRotate.setPosition(dumpPos);
                        sleep(200);

                        break;

                    case "":

                        drive.followTrajectory(toRight);

                        while (!isStopRequested() && drive.isFollowingTrajectory()) {
                            Pose2d currentPose = drive.getPoseEstimate();

                            TelemetryPacket packet = new TelemetryPacket();
                            Canvas fieldOverlay = packet.fieldOverlay();

                            packet.put("x", currentPose.getX());
                            packet.put("y", currentPose.getY());
                            packet.put("heading", currentPose.getHeading() * (180/ PI));

                            fieldOverlay.setStrokeWidth(4);
                            fieldOverlay.setStroke("green");
                            DashboardUtil.drawSampledTrajectory(fieldOverlay, toRight);

                            fieldOverlay.setFill("blue");
                            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                            dashboard.sendTelemetryPacket(packet);

                            drive.update();
                        }

                        pulley.setPower(1);
                        sleep(1000);
                        pulley.setPower(0);
                        intakeRotate.setPosition(dumpPos);
                        sleep(200);

                        break;

                }

                break;

            case 2:

                if(detector.getAligned())
                {
                    GoldPosition = "M";
                    unhang();
                    drive.followTrajectory(toMiddleMarker);

                    requestOpModeStop();
                }

                unhang();

                drive.followTrajectory(toLeft);




        }

    }

    public void unhang() {
        pepeJAM.setPosition(0.2);
        sleep(1500);
    }

    public void park() {
        pulley.setPower(1);
        sleep(1000);
        pulley.setPower(0);
        intakeRotate.setPosition(dumpPos);
        sleep(200);
    }

    public void claim() {
        pulley.setPower(1);
        sleep(1250);

        intakeRotate.setPosition(dumpPos);
        intake.setPower(1);
        pulley.setPower(0.25);
        sleep(1250);

        pulley.setPower(-1);
        intake.setPower(0);
        intakeRotate.setPosition(0);
        sleep(750);
    }

}