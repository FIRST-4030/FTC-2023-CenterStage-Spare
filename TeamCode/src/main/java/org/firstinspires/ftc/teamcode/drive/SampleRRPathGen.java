package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * Op mode for a sample path created in RRPAthGen
 */
@Config
@Autonomous(group = "drive")
public class SampleRRPathGen extends LinearOpMode {

    int xVal = 30;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /*TrajectorySequence demo = drive.trajectorySequenceBuilder(new Pose2d(23.44, -23.44, Math.toRadians(90.00)))
                .lineTo(new Vector2d(24.07, 23.86),
                        SampleMecanumDrive.getVelocityConstraint(30,30,14),
                        SampleMecanumDrive.getAccelerationConstraint(30)) */
        TrajectorySequence demo = drive.trajectorySequenceBuilder(new Pose2d(-36.89, -60.02, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-36.47, -13.35, Math.toRadians(0.00)))
                .lineTo(new Vector2d(33.95, -12.30))
                .lineToConstantHeading(new Vector2d(34.58, -35.00))
                .build();
        waitForStart();
        drive.setPoseEstimate(demo.start());

        drive.followTrajectorySequence(demo);

    }
}