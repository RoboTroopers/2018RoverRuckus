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
package org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.road_runner.classes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.drive.Kinematics;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.road_runner.master.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Worlds_Code.Autonomous.road_runner.master.drive.SampleMecanumDriveREV;

/*
 * Similar to the deprecated TrackWidthCalibrationOpMode, this routine attempts to automagically
 * determine the drive track width. The basic idea is to use a motion profile to rotate the robot
 * a certain circumferential distance and compare it to the angle swept out (as measured by the
 * IMU). For robustness, this procedure is repeated a few times, and the final track width is
 * averaged over those runs.
 */
@Config
@Autonomous
public class NewTrackWidthCalibrationOpMode extends LinearOpMode {
    public static int CIRCUMFERENTIAL_DISTANCE = 500;
    public static int NUM_TRIALS = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        // it's important that the IMU/gyro/heading sensor is not part of the localization
        drive.setLocalizer(new MecanumDrive.MecanumLocalizer(drive, false));

        telemetry.log().add("Press play to begin the track width calibration routine");
        telemetry.log().add("Make sure your robot has enough clearance to turn smoothly");
        telemetry.log().add("Additionally, set the drive's track width to 1");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.log().clear();
        telemetry.log().add("Running...");
        telemetry.update();

        MovingStatistics trackWidthStats = new MovingStatistics(NUM_TRIALS);
        NanoClock clock = NanoClock.system();
        for (int i = 0; i < NUM_TRIALS; i++) {
            drive.setPoseEstimate(new Pose2d());

            double headingAccumulator = 0;
            double lastHeading = drive.getExternalHeading();
            MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(0, 0, 0, 0),
                    new MotionState(CIRCUMFERENTIAL_DISTANCE, 0, 0, 0),
                    DriveConstants.BASE_CONSTRAINTS.maximumVelocity,
                    DriveConstants.BASE_CONSTRAINTS.maximumAcceleration
            );

            double startTime = clock.seconds();
            while (!isStopRequested()) {
                double elapsedTime = clock.seconds() - startTime;
                if (elapsedTime > profile.duration()) {
                    drive.setVelocity(new Pose2d(0, 0, 0));
                    break;
                }

                double heading = drive.getExternalHeading();
                // accumulator is an unwrapped version of the heading
                headingAccumulator += Angle.norm(heading - lastHeading);
                lastHeading = heading;

                MotionState state = profile.get(elapsedTime);
                drive.setVelocity(new Pose2d(0, 0,
                        Kinematics.calculateMotorFeedforward(
                                state.getV(),
                                state.getA(),
                                DriveConstants.kV,
                                DriveConstants.kA,
                                DriveConstants.kStatic
                        )
                ));

                drive.updatePoseEstimate();
            }

            double trackWidth = drive.getPoseEstimate().getHeading() / headingAccumulator;
            trackWidthStats.add(trackWidth);

            sleep(1000);
        }

        telemetry.log().clear();
        telemetry.log().add("Calibration complete");
        telemetry.log().add(Misc.formatInvariant("Effective track width = %.2f (SE = %.3f)",
                trackWidthStats.getMean(), trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }
}
