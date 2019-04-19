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

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.roadrunner.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.roadrunner.drive.SampleMecanumDriveREVOptimized;

public class Trajectories {

    /*SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

    public Trajectory exitLZ = new TrajectoryBuilder(new Pose2d(-12, -12, Auto_Crater.oof(315)), DriveConstants.BASE_CONSTRAINTS)
            //.lineTo(new Vector2d(-8,-17))
            .forward(3)
            .strafeRight(26)
            .splineTo(new Pose2d(-10,-45, Auto_Crater.oof(270)))
            .strafeRight(5)
            .turnTo(Auto_Crater.oof(270))
            .build();

    Trajectory depot = new TrajectoryBuilder(new Pose2d(-10,-50, Auto_Crater.oof(270)), DriveConstants.BASE_CONSTRAINTS)
            .turnTo(Auto_Crater.oof(345.5))
            .strafeRight(18)
            .forward(10)
            .strafeLeft(4)
            .waitFor(0.5)
            .build();

    Trajectory toLeft = new TrajectoryBuilder(new Pose2d(0,-64,0), DriveConstants.BASE_CONSTRAINTS)
            .reverse()
            .splineTo(new Pose2d(-24,-50))
            .turnTo(Auto_Crater.oof(175))
            .reverse()
            .strafeLeft(4)
            .forward(18)
            .build();

    Trajectory toMiddle = new TrajectoryBuilder(new Pose2d(0,-64,0), DriveConstants.BASE_CONSTRAINTS)
            .back(18)
            .turnTo(Auto_Crater.oof(320))
            .strafeLeft(8)
            .back(32)
            .turnTo(Auto_Crater.oof(240))
            .forward(14)
            .build();

    Trajectory toRight = new TrajectoryBuilder(new Pose2d(0,-64,0), DriveConstants.BASE_CONSTRAINTS)
            .back(18)
            .turnTo(Auto_Crater.oof(321))
            .strafeLeft(10)
            .back(46)
            .turnTo(Auto_Crater.oof(240))
            .forward(14)
            .build();

    Trajectory toLeftDepot = new TrajectoryBuilder(new Pose2d(12,-12, Auto_Crater.oof(45)), DriveConstants.BASE_CONSTRAINTS)
            .forward(3)
            .strafeRight(12)
            .build();

    Trajectory toRightMarker = new TrajectoryBuilder((drive.getPoseEstimate()), DriveConstants.BASE_CONSTRAINTS)
            .back(24)
            .turnTo(Auto_Crater.oof(315))
            .forward(18)
            .build();

    Trajectory toMiddleMarker = new TrajectoryBuilder(new Pose2d(12,-12, Auto_Crater.oof(45)), DriveConstants.BASE_CONSTRAINTS)
            .forward(3)
            .strafeRight(12)
            .back(5)
            .turnTo(Auto_Crater.oof(315))
            .forward(18)
            .build();

    Trajectory middlePark = new TrajectoryBuilder((drive.getPoseEstimate()), DriveConstants.BASE_CONSTRAINTS)
            .back(18)
            .turnTo(Auto_Crater.oof(45))
            .forward(36)
            .turnTo(Auto_Crater.oof(90))
            .forward(20)
            .build();*/
}
