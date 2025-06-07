package org.firstinspires.ftc.teamcode;


// Non-RR imports
import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import static org.firstinspires.ftc.teamcode.OurColorSensor.DetectedColor.BLUE;
import static org.firstinspires.ftc.teamcode.OurColorSensor.DetectedColor.OTHER;
import static org.firstinspires.ftc.teamcode.OurColorSensor.DetectedColor.RED;
import static org.firstinspires.ftc.teamcode.OurColorSensor.DetectedColor.YELLOW;

//RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BLUE_Feb_22_SampleAuton", group = "Areas")
public class BLUE_SampleAuton extends LinearOpMode {

    private Servo claw_servo;
    private Servo claw_pivot_servo;
    private Servo left_arm;
    private Servo right_arm;
    private Servo left_intake_chamber_servo;
    private Servo right_intake_chamber_servo;
    private Servo left_extendo_servo;
    private Servo right_extendo_servo;
    private UnboundedMotor left_arm_motor;
    private UnboundedMotor right_arm_motor;
    private DcMotor intake;
    private static int SLIDES_UP_POSITION = 760;
    private final static int SLIDES_DOWN_POSITION = -12;
    private final static long[] TIME_BEFORE_TRANSFER_INITIATED_FOR_SPIKE = {400, 1300, 1300};
    private final static long[] TIME_BEFORE_SCORE_INITIATED_FOR_SPIKE = {1050, 1650, 1400};

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime universalTimer = new ElapsedTime();
    private double intakingInSubmersibleMaxTime = 2500;

    private double SUBMERISBLE_TANGENT = 0;

    private OurColorSensor.DetectedColor ALLIANCE_COLOR = BLUE;
    private OurColorSensor.DetectedColor OPPOSITE_ALLIANCE_COLOR = RED;

    private static volatile boolean searchForSampleInSubmersibleTrigger = false;
    private static volatile boolean sampleFromSubmerisbleIntaked = false;
    private static volatile boolean INTAKE_TYPE = true;
    private static volatile boolean SIXTH = false;

    public void intakeTimeAdjustment() { while (!searchForSampleInSubmersibleTrigger); }

    public void intake(double maxTime, ElapsedTime timer) {
        claw_servo.setPosition(0.2);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        left_arm_motor.setPosition(SLIDES_DOWN_POSITION);
        right_arm_motor.setPosition(SLIDES_DOWN_POSITION);
        left_arm.setPosition(0.0653);
        right_arm.setPosition(0.0653);
        claw_pivot_servo.setPosition(1);
        left_extendo_servo.setPosition(0.1325);
        right_extendo_servo.setPosition(0.1325);
        left_intake_chamber_servo.setPosition(0.81325);
        right_intake_chamber_servo.setPosition(0.81325);
        sleep(400);
        intake.setPower(1);
        left_extendo_servo.setPosition(0.3);
        right_extendo_servo.setPosition(0.3);

        claw_pivot_servo.setPosition(0.7);
        waitUntilIntaked(maxTime, timer);

        intake.setPower(0.5);
        claw_pivot_servo.setPosition(0.568);
        if (INTAKE_TYPE) {
            left_intake_chamber_servo.setPosition(0.075);
            right_intake_chamber_servo.setPosition(0.075);
            sleep(180);
            left_extendo_servo.setPosition(0.0);
            right_extendo_servo.setPosition(0.0);
            sleep(500);
        }
        else {
            left_extendo_servo.setPosition(0.34);
            right_extendo_servo.setPosition(0.34);
        }
    }

    public void outtake(long outtakeTime, long armMoveTime){
        intake.setPower(-0.825);
        sleep(outtakeTime);
        intake.setPower(0);
        claw_servo.setPosition(0.35);
        left_arm.setPosition(0.2);
        right_arm.setPosition(0.2);
        sleep(300);
        left_arm_motor.setPosition(SLIDES_UP_POSITION);
        right_arm_motor.setPosition(SLIDES_UP_POSITION);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        sleep(200);
        claw_pivot_servo.setPosition(0.7);
        sleep(armMoveTime);
        left_arm.setPosition(0.485);
        right_arm.setPosition(0.485);
        sleep(100);
        claw_pivot_servo.setPosition(0.875);
    }

    public void waitUntilIntaked(double maxTime, ElapsedTime timer) {

        searchForSampleInSubmersibleTrigger = false;
        timer.reset();

        while (timer.milliseconds() <= maxTime && (ourColorSensor.detectColor() == OPPOSITE_ALLIANCE_COLOR || ourColorSensor.detectColor() == OTHER)) {
            telemetry.clearAll();
            telemetry.addData("intaking", timer.milliseconds());
            telemetry.addData("detected color", ourColorSensor.detectColor());
            telemetry.update();
        }
        searchForSampleInSubmersibleTrigger = true;
    }

    Thread thread2 = new Thread(() -> {
        try{
            claw_servo.setPosition(0.375);
            claw_pivot_servo.setPosition(0.875);
            left_arm.setPosition(0.5);
            right_arm.setPosition(0.5);
            left_arm_motor.setPosition(SLIDES_UP_POSITION);
            right_arm_motor.setPosition(SLIDES_UP_POSITION);
            left_arm_motor.setPower(1);
            right_arm_motor.setPower(1);
            sleep(100);
            left_arm.setPosition(0.485);
            right_arm.setPosition(0.485);
            claw_pivot_servo.setPosition(0.875);
            sleep(1250);
            claw_servo.setPosition(0.2);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    });
    Thread thread5 = new Thread(() -> {
        try{
            intake(1000, timer);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    });
    Thread thread6 = new Thread(() -> {
        try {

            outtake(350,500);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    });
    Thread thread7 = new Thread(() -> {
        try{
            intake(1000, timer);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    });
    Thread thread8 = new Thread(() -> {
        try {
            outtake(350,750);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    });

    Thread thread9 = new Thread(() -> {
        try {
            searchForSampleInSubmersibleTrigger = false;
            intake.setPower(-1);
            left_extendo_servo.setPosition(0.175);
            right_extendo_servo.setPosition(0.175);
            sleep(1500);
            left_extendo_servo.setPosition(0.316);
            right_extendo_servo.setPosition(0.316);
            sleep(650);
            left_intake_chamber_servo.setPosition(0.81325);
            right_intake_chamber_servo.setPosition(0.81325);
            sleep(100);
            intake.setPower(1);
            sleep(20);
            left_extendo_servo.setPosition(0.36);
            right_extendo_servo.setPosition(0.36);
            intake(1500, timer);
            left_intake_chamber_servo.setPosition(0.425);
            right_intake_chamber_servo.setPosition(0.425);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    });

    Thread thread9b = new Thread(() -> {
        try {
            searchForSampleInSubmersibleTrigger = false;
            intake.setPower(-1);
            left_extendo_servo.setPosition(0.175);
            right_extendo_servo.setPosition(0.175);
            sleep(1650);
            left_extendo_servo.setPosition(0.316);
            right_extendo_servo.setPosition(0.316);
            sleep(650);
            left_intake_chamber_servo.setPosition(0.81325);
            right_intake_chamber_servo.setPosition(0.81325);
            sleep(100);
            intake.setPower(1);
            sleep(20);
            left_extendo_servo.setPosition(0.36);
            right_extendo_servo.setPosition(0.36);
            intake(1500, timer);
            left_intake_chamber_servo.setPosition(0.425);
            right_intake_chamber_servo.setPosition(0.425);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    });

    Thread thread10a = new Thread(() -> {
        try {
            intake(1350, timer);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    });

    Thread thread10b = new Thread(() -> {
        try {
            intake(1350, timer);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    });

    Thread thread10c = new Thread(() -> {
        try {
            intake(1350, timer);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    });

    Thread thread12 = new Thread(() -> {
        try {
            sleep(750);
            outtake(380,550);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    });

    private OurColorSensor ourColorSensor = new OurColorSensor();

    public void park() {
        left_arm.setPosition(0.3);
        right_arm.setPosition(0.3);
        claw_pivot_servo.setPosition(0.5);
        sleep(350);
        left_arm.setPosition(0.6);
        right_arm.setPosition(0.6);
        claw_pivot_servo.setPosition(0.875);
        while(opModeIsActive()) {}
    }

    public void checkIntakedSample(OurColorSensor ourColorSensor) {
        if (ourColorSensor.detectColor() == YELLOW || ourColorSensor.detectColor() == ALLIANCE_COLOR) {
            left_intake_chamber_servo.setPosition(0.075);
            right_intake_chamber_servo.setPosition(0.075);
            sampleFromSubmerisbleIntaked = true;
        }
        else if (ourColorSensor.detectColor() == OPPOSITE_ALLIANCE_COLOR) {
            intake.setPower(-1);
            sleep(250);
        }
    }

    @Override
    public void runOpMode() {
        left_arm = hardwareMap.get(Servo.class, "left_arm");
        right_arm = hardwareMap.get(Servo.class, "right_arm");
        left_intake_chamber_servo = hardwareMap.get(Servo.class, "left_intake_servo");
        right_intake_chamber_servo = hardwareMap.get(Servo.class, "right_intake_servo");
        left_intake_chamber_servo.setDirection(REVERSE);
        right_intake_chamber_servo.setDirection(REVERSE);
        left_extendo_servo = hardwareMap.get(Servo.class, "left_extendo_servo");
        right_extendo_servo = hardwareMap.get(Servo.class, "right_extendo_servo");
        left_extendo_servo.setDirection(FORWARD);
        right_extendo_servo.setDirection(FORWARD);

        left_arm_motor = new UnboundedMotor(hardwareMap, "left_arm_motor");
        right_arm_motor = new UnboundedMotor(hardwareMap, "right_arm_motor");
        right_arm_motor.setDirection(UnboundedMotor.Direction.REVERSE);
        left_arm_motor.setMaxCurrent(8_500);
        right_arm_motor.setMaxCurrent(8_500);
        left_arm_motor.setMode(UnboundedMotor.RunMode.RUN_USING_ENCODER);
        right_arm_motor.setMode(UnboundedMotor.RunMode.RUN_USING_ENCODER);
        left_arm_motor.start(new UnboundedMotor.ScheduleWithFixedDelay(15));
        right_arm_motor.start(new UnboundedMotor.ScheduleWithFixedDelay(15));

        claw_servo = hardwareMap.get(Servo.class, "claw_servo");
        claw_servo.setDirection(FORWARD);
        claw_pivot_servo = hardwareMap.get(Servo.class, "claw_pivot_servo");
        claw_pivot_servo.setDirection(FORWARD);
        intake = hardwareMap.get(DcMotor.class, "intake");
        ourColorSensor.initialize(hardwareMap);
        Pose2d initialPose = new Pose2d(-33, -60, Math.toRadians(0));
        Pose2d thirdSamplePose = new Pose2d(-53, -48, Math.toRadians(63));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder firstSample = drive.actionBuilder(initialPose)

                .splineToLinearHeading(new Pose2d(-53, -48, Math.toRadians(64)), Math.toRadians(0),
                        new TranslationalVelConstraint(50),
                        new ProfileAccelConstraint(-35,25))
                ;

        TrajectoryActionBuilder secondSample = drive.actionBuilder(thirdSamplePose)
                .turnTo(Math.toRadians(88))
                ;

        TrajectoryActionBuilder backToBucketSecondSample = drive.actionBuilder(new Pose2d(-53, -48, Math.toRadians(85)))
                .turnTo(Math.toRadians(63))
                ;

        TrajectoryActionBuilder thirdSample = drive.actionBuilder(thirdSamplePose)
                .turnTo(Math.toRadians(106.25))

                ;

        TrajectoryActionBuilder backToThirdSample = drive.actionBuilder(new Pose2d(-53, -48, Math.toRadians(106.25)))
                .turnTo(Math.toRadians(63))
                ;

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-53, -48, Math.toRadians(63)))

                .splineToLinearHeading(new Pose2d(-16, -6, Math.toRadians(180)), Math.toRadians(0),
                        new TranslationalVelConstraint(50),
                        new ProfileAccelConstraint(-35,25))
                ;

        TrajectoryActionBuilder toFifthSample = drive.actionBuilder(new Pose2d(-53, -48, Math.toRadians(63)))
                .splineToLinearHeading(new Pose2d(-18,4, Math.toRadians(0)), Math.toRadians(70),
                        new TranslationalVelConstraint(50),
                        new ProfileAccelConstraint(-35, 25))
                .splineToConstantHeading(new Vector2d(-7.5,4), Math.toRadians(0),
                        new TranslationalVelConstraint(50),
                        new ProfileAccelConstraint(-35, 25))
                ;

        Action backToBucketForFifthSample;

        Action fifthSampleScorePossibleCorrection;

        Action adjustForFifthSample = drive.actionBuilder(initialPose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-4,-4, Math.toRadians(25)), Math.toRadians(0))
                .build();

        Action unAdjustForFifthSample = drive.actionBuilder(initialPose)
                .setTangent(0)
                .turnTo(Math.toRadians(-25))
                .build();


        claw_servo.setPosition(0.35);

        left_arm.setPosition(0.1);
        right_arm.setPosition(0.1);

        claw_pivot_servo.setPosition(0.5);

        left_extendo_servo.setPosition(0.0);
        right_extendo_servo.setPosition(0.0);
        left_intake_chamber_servo.setPosition(0.075);
        right_intake_chamber_servo.setPosition(0.075);


        waitForStart();
        if (isStopRequested()) return;

        universalTimer.reset();

        thread2.start();
        telemetry.addLine("1st sample");
        telemetry.update();
        Actions.runBlocking(firstSample.build());///
        claw_servo.setPosition(0.2);
        intake(1000, timer);
        sleep(50);
        if (ourColorSensor.detectColor() != OTHER) {
            sleep(350);
            outtake(300,500);
            sleep(TIME_BEFORE_SCORE_INITIATED_FOR_SPIKE[0]);
        }
        else {
            left_extendo_servo.setPosition(0.2);
            right_extendo_servo.setPosition(0.2);
        }
        claw_servo.setPosition(0.2);

        telemetry.addLine("2nd sample");
        telemetry.update();
        thread5.start();
        Actions.runBlocking(secondSample.build());///
        /* prevents thread from being cut short if sample is missed */ try { thread2.join(); } catch (Exception e) {}
        intakeTimeAdjustment();
        sleep(TIME_BEFORE_TRANSFER_INITIATED_FOR_SPIKE[1]);
        if (ourColorSensor.detectColor() != OurColorSensor.DetectedColor.OTHER) {
            thread6.start();
            Actions.runBlocking(backToBucketSecondSample.build());
            sleep(TIME_BEFORE_SCORE_INITIATED_FOR_SPIKE[1]);
            try {
                thread6.join();
            } catch (Exception e) {}
        }
        else {
            left_extendo_servo.setPosition(0.2);
            right_extendo_servo.setPosition(0.2);
            Actions.runBlocking(backToBucketSecondSample.build());
        }
        claw_servo.setPosition(0.2);

        sleep(50);
        telemetry.addLine("3rd sample");
        telemetry.update();
        thread7.start();
        Actions.runBlocking(thirdSample.build());///
        /* prevents thread from being cut short if sample is missed */ try { thread5.join(); } catch (Exception e) {}
        intakeTimeAdjustment();
        sleep(TIME_BEFORE_TRANSFER_INITIATED_FOR_SPIKE[2]);
        if (ourColorSensor.detectColor() != OTHER) {
            thread8.start();
            Actions.runBlocking(backToThirdSample.build());
            sleep(TIME_BEFORE_SCORE_INITIATED_FOR_SPIKE[2]);

            sleep(300);
            claw_servo.setPosition(0.2);

            sleep(100);
        }
        else {
            left_extendo_servo.setPosition(0.2);
            right_extendo_servo.setPosition(0.2);
            Actions.runBlocking(backToThirdSample.build());

            sleep(100);
            claw_servo.setPosition(0.2);

            sleep(50);
        }
        /* prevents thread from being cut short if sample is missed */ try { thread7.join(); } catch (Exception e) {}

        left_arm.setPosition(0.09);
        right_arm.setPosition(0.09);
        left_arm_motor.setPosition(SLIDES_DOWN_POSITION);
        right_arm_motor.setPosition(SLIDES_DOWN_POSITION);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        left_intake_chamber_servo.setPosition(0.425);
        right_intake_chamber_servo.setPosition(0.425);
        claw_pivot_servo.setPosition(0.7);
        try {
            thread8.join();
        } catch (Exception e) {}
        INTAKE_TYPE = false;
        thread9.start();
        Actions.runBlocking(toFifthSample.build());
        intakeTimeAdjustment();
        if (ourColorSensor.detectColor() == ALLIANCE_COLOR || ourColorSensor.detectColor() == YELLOW) {

            SIXTH = true;

            try {
                thread9.join();
            } catch (Exception e) {}
        }
        else {
            if (ourColorSensor.detectColor() == OPPOSITE_ALLIANCE_COLOR) {
                intake.setPower(-1);
                sleep(250);
            }

            intake.setPower(1);

            thread10a.start();
            SUBMERISBLE_TANGENT = Math.toRadians(-25);
            Actions.runBlocking(adjustForFifthSample);
            intakeTimeAdjustment();
            try { thread10a.join(); } catch (Exception e) {}
            try { thread9.join(); } catch (Exception e) {}
            checkIntakedSample(ourColorSensor);

            if (!sampleFromSubmerisbleIntaked) {
                thread10b.start();
                SUBMERISBLE_TANGENT = Math.toRadians(0);
                Actions.runBlocking(unAdjustForFifthSample);
                intakeTimeAdjustment();
                try { thread10b.join(); } catch (Exception e) {}
                checkIntakedSample(ourColorSensor);
            }

            if (!sampleFromSubmerisbleIntaked) {
                thread10c.start();
                SUBMERISBLE_TANGENT = Math.toRadians(25);
                Actions.runBlocking(unAdjustForFifthSample);
                intakeTimeAdjustment();
                try { thread10c.join(); } catch (Exception e) {}
                checkIntakedSample(ourColorSensor);
            }

            if (!sampleFromSubmerisbleIntaked) {
                left_arm.setPosition(0.25);
                right_arm.setPosition(0.25);

                left_arm_motor.kill();
                right_arm_motor.kill();

                while (opModeIsActive()) {}
            }
            SIXTH = true;

            if (SUBMERISBLE_TANGENT == 0) {
                fifthSampleScorePossibleCorrection = drive.actionBuilder(initialPose)
                        .splineToConstantHeading(new Vector2d(4,4), Math.toRadians(0))
                        .build();
            }
            else {
                fifthSampleScorePossibleCorrection = drive.actionBuilder(initialPose)
                        .splineToLinearHeading(new Pose2d(4,4, SUBMERISBLE_TANGENT), Math.toRadians(0))
                        .build();
            }
            drive.localizer.setPose(initialPose);
            Actions.runBlocking(fifthSampleScorePossibleCorrection);
        }

        left_extendo_servo.setPosition(0.2);
        right_extendo_servo.setPosition(0.2);
        sleep(200);
        left_intake_chamber_servo.setPosition(0.075);
        right_intake_chamber_servo.setPosition(0.075);
        sleep(350);
        left_extendo_servo.setPosition(0.0);
        right_extendo_servo.setPosition(0.0);

        claw_servo.setPosition(0.2);
        claw_pivot_servo.setPosition(0.55);
        SLIDES_UP_POSITION = 725;

        thread12.start();
        backToBucketForFifthSample = drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineTo(new Vector2d(-76, -115), Math.toRadians(-115))
                .build();
        drive.localizer.setPose(initialPose);
        Actions.runBlocking(backToBucketForFifthSample);
        claw_servo.setPosition(0.2);

        left_extendo_servo.setPosition(0.0);
        right_extendo_servo.setPosition(0.0);

        try {
            thread12.join();
        } catch (Exception e) {}

        sleep(200);

        left_arm.setPosition(0.09);
        right_arm.setPosition(0.09);
        left_arm_motor.setPosition(SLIDES_DOWN_POSITION);
        right_arm_motor.setPosition(SLIDES_DOWN_POSITION);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        left_intake_chamber_servo.setPosition(0.425);
        right_intake_chamber_servo.setPosition(0.425);
        claw_pivot_servo.setPosition(0.7);

        //if (SIXTH) pastFive(drive); //won't destroy fuse by putting too much work on the processor

        sleep(750);

        left_arm_motor.kill();
        right_arm_motor.kill();

    }

    private void pastFive(MecanumDrive drive) {

        TrajectoryActionBuilder toSixthSample = drive.actionBuilder(new Pose2d(-53, -48, Math.toRadians(-15)))
                .splineTo(new Vector2d(-18,-6), Math.toRadians(0),
                        new TranslationalVelConstraint(50),
                        new ProfileAccelConstraint(-35, 25))
                ;

        thread9b.start();
        Actions.runBlocking(toSixthSample.build());
        intakeTimeAdjustment();
        sleep(400);
        try {
            thread9b.join();
        } catch (Exception e) {}

        claw_servo.setPosition(0.2);
        claw_pivot_servo.setPosition(0.55);
    }

}
