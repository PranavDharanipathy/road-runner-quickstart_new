package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "AdvancedMotionTest")
public class AdvancedMotionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        TrajectoryActionBuilder advancedMotionTest = drive.actionBuilder(startPose)
                .lineToXLinearHeading(48, Math.toRadians(90))
                .lineToXLinearHeading(0, Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 0, Math.toRadians(180)), Math.toRadians(0),
                null,
                new ProfileAccelConstraint(-15.0, 15.0))
                .waitSeconds(1.25)
                .turnTo(90)
                .strafeToLinearHeading(new Vector2d(0,50), Math.toRadians(0));

        waitForStart();

        Actions.runBlocking(advancedMotionTest.build());
    }
}
