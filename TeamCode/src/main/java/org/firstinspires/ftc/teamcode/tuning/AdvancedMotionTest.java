package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
                .splineToLinearHeading(new Pose2d(60, 0, Math.toRadians(180)), Math.toRadians(0),
                new TranslationalVelConstraint(50),
                new ProfileAccelConstraint(-20.0, 20.0))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0));

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Actions.runBlocking(advancedMotionTest.build());
        }
    }
}
