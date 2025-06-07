package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Areas_Feb22_SpecimenAuton", group = "Areas")
public class Areas_Feb22_SpecimenAuton extends LinearOpMode {

    private Servo claw_servo;
    private Servo claw_pivot_servo;
    private Servo left_arm;
    private Servo right_arm;
    private DcMotor left_arm_motor;
    private DcMotor right_arm_motor;
    private DcMotor intake;
    private ElapsedTime timer = new ElapsedTime();

    private Servo left_intake_chamber_servo;
    private Servo right_intake_chamber_servo;
    private Servo left_extendo_servo;
    private Servo right_extendo_servo;

    private final static int SLIDES_UP_POSITION = 815;
    private final static int SLIDES_DOWN_POSITION = -38;
    private static int SLIDES_SCORE_SPEC_POSITION = 315;
    private final static int SLIDES_PICK_SPEC_POSITION = 220;

    //                                             #1  #2  #3  #4  #5
    private final static double[] CYCLE_FORWARD = {14, 14, 14, 16, 17};
    private final static double[] CYCLE_BACKWARD = {-22.5, -23, -23.5, -23.5, -24};

    public void slidesToPose(int targetPosition) {
        left_arm_motor.setTargetPosition(targetPosition);
        right_arm_motor.setTargetPosition(targetPosition);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    ///THREADS
    Thread armSetupAndIntakeFirstSample = new Thread(() -> {
        try {
            //arm goes to pick position
            sleep(350);
            claw_pivot_servo.setPosition(0.875);
            slidesToPose(SLIDES_PICK_SPEC_POSITION);
            sleep(200);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);

            sleep(200);
            extendoExtend(0.1325);
            sleep(300);
            intake.setPower(1);
            sleep(275);
            extendoExtend(0.25);
            sleep(2500);
            extendoExtend(0.36);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    });

    Thread pickSpecs = new Thread(() -> {
        try  {
            //sleep(1275);
            ///1st cycle
            claw_pivot_servo.setPosition(0.87);
            claw_servo.setPosition(0.35);
            slidesToPose(SLIDES_SCORE_SPEC_POSITION-5);
            sleep(200);
            claw_pivot_servo.setPosition(1);
            sleep(390);
            left_arm.setPosition(0.3);
            right_arm.setPosition(0.3);
            sleep(10);
            left_arm.setPosition(0.125);
            right_arm.setPosition(0.125);
            sleep(400);
            claw_pivot_servo.setPosition(0.775);
            sleep(775);
            claw_servo.setPosition(0.1875);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
            slidesToPose(SLIDES_PICK_SPEC_POSITION);
            claw_pivot_servo.setPosition(0.87);

            ///2nd cycle
            sleep(2050);
            claw_pivot_servo.setPosition(0.87);
            claw_servo.setPosition(0.35);
            slidesToPose(SLIDES_SCORE_SPEC_POSITION+5);
            sleep(200);
            claw_pivot_servo.setPosition(1);
            sleep(340);
            left_arm.setPosition(0.3);
            right_arm.setPosition(0.3);
            sleep(10);
            left_arm.setPosition(0.125);
            right_arm.setPosition(0.125);
            sleep(400);
            claw_pivot_servo.setPosition(0.775);
            sleep(1000);
            claw_servo.setPosition(0.1875);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
            slidesToPose(SLIDES_PICK_SPEC_POSITION);
            claw_pivot_servo.setPosition(0.87);

            ///3rd cycle
            sleep(2025);
            claw_pivot_servo.setPosition(0.87);
            claw_servo.setPosition(0.35);
            slidesToPose(SLIDES_SCORE_SPEC_POSITION);
            sleep(150);
            claw_pivot_servo.setPosition(1);
            sleep(515);
            left_arm.setPosition(0.3);
            right_arm.setPosition(0.3);
            sleep(10);
            left_arm.setPosition(0.125);
            right_arm.setPosition(0.125);
            sleep(400);
            claw_pivot_servo.setPosition(0.775);
            sleep(1000);
            claw_servo.setPosition(0.1875);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
            slidesToPose(SLIDES_PICK_SPEC_POSITION);
            claw_pivot_servo.setPosition(0.87);

            ///4th cycle
            sleep(1930);
            claw_pivot_servo.setPosition(0.87);
            claw_servo.setPosition(0.35);
            slidesToPose(SLIDES_SCORE_SPEC_POSITION+2);
            sleep(150);
            claw_pivot_servo.setPosition(1);
            sleep(615);
            left_arm.setPosition(0.3);
            right_arm.setPosition(0.3);
            sleep(10);
            left_arm.setPosition(0.125);
            right_arm.setPosition(0.125);
            sleep(400);
            claw_pivot_servo.setPosition(0.775);
            sleep(1000);
            claw_servo.setPosition(0.1875);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
            slidesToPose(SLIDES_PICK_SPEC_POSITION);
            claw_pivot_servo.setPosition(0.87);

            ///5th cycle
            sleep(1950);
            claw_pivot_servo.setPosition(0.87);
            claw_servo.setPosition(0.35);
            slidesToPose(SLIDES_SCORE_SPEC_POSITION + 2);
            sleep(200);
            claw_pivot_servo.setPosition(1);
            sleep(500);
            left_arm.setPosition(0.3);
            right_arm.setPosition(0.3);
            sleep(10);
            left_arm.setPosition(0.125);
            right_arm.setPosition(0.125);
            sleep(300);
            claw_pivot_servo.setPosition(0.775);
            sleep(1100);
            claw_servo.setPosition(0.1875);
            left_arm.setPosition(0.55);
            right_arm.setPosition(0.55);
            slidesToPose(SLIDES_DOWN_POSITION);
            claw_pivot_servo.setPosition(0.5);

        } catch (Exception e) {
            //Thread.currentThread().interrupt();
        }
    });


    @Override
    public void runOpMode() {

        //initialization
        left_arm_motor = hardwareMap.get(DcMotor.class, "left_arm_motor");
        right_arm_motor = hardwareMap.get(DcMotor.class, "right_arm_motor");
        right_arm_motor.setDirection(DcMotor.Direction.REVERSE);
        left_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_arm = hardwareMap.get(Servo.class, "left_arm");
        right_arm = hardwareMap.get(Servo.class, "right_arm");
        left_arm.setDirection(Servo.Direction.FORWARD);
        right_arm.setDirection(Servo.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");

        claw_servo = hardwareMap.get(Servo.class, "claw_servo");
        claw_servo.setDirection(Servo.Direction.FORWARD);
        claw_pivot_servo = hardwareMap.get(Servo.class, "claw_pivot_servo");

        left_extendo_servo = hardwareMap.get(Servo.class, "left_extendo_servo");
        right_extendo_servo = hardwareMap.get(Servo.class, "right_extendo_servo");

        left_intake_chamber_servo = hardwareMap.get(Servo.class, "left_intake_servo");
        right_intake_chamber_servo = hardwareMap.get(Servo.class, "right_intake_servo");
        left_intake_chamber_servo.setDirection(REVERSE);
        right_intake_chamber_servo.setDirection(REVERSE);


        Pose2d initialPose = new Pose2d(0,0, Math.toRadians(0));
        Pose2d cyclePose = new Pose2d(-22,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        //pose initialization
        extendoRetract(0);

        left_arm.setPosition(0.125);
        right_arm.setPosition(0.125);
        claw_servo.setPosition(0.1875);
        claw_pivot_servo.setPosition(0.5);


        waitForStart();
        if (isStopRequested()) return;


        drive.localizer.setPose(initialPose);
        armSetupAndIntakeFirstSample.start();
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .splineToLinearHeading(new Pose2d(25,-6, Math.toRadians(-60)), Math.toRadians(0.0))
                        .build()
        );
        sleep(180);

        /**Sample 1**/ giveSampleAndReturn(drive, initialPose,3.5, Math.toRadians(1), -75, true);

        sleep(475);

        /**Sample 2**/ giveSampleAndReturn(drive, initialPose,5, Math.toRadians(11.85), -75, false);

        extendoExtend(0.36);
        sleep(575);
        /**Sample 3**/ giveSampleAndReturnThirdTime(drive, initialPose,0, 105, -72);

        drive.localizer.setPose(initialPose);
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .setTangent(Math.toRadians(-2.5))
                        .lineToXConstantHeading(-27, new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))
                        .build()
        );

        drive.localizer.setPose(cyclePose);
        pickSpecs.start();
        Actions.runBlocking(
                drive.actionBuilder(cyclePose)
                        .setTangent(Math.toRadians(44.5))
                        .lineToXConstantHeading(CYCLE_FORWARD[0], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))
                        .lineToXConstantHeading(CYCLE_BACKWARD[0], new TranslationalVelConstraint(100), new ProfileAccelConstraint(-50,50))

                        .setTangent(Math.toRadians(45))
                        .lineToXConstantHeading(CYCLE_FORWARD[1], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))
                        .lineToXConstantHeading(CYCLE_BACKWARD[1], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))

                        .setTangent(Math.toRadians(43.85))
                        .lineToXConstantHeading(CYCLE_FORWARD[2], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))
                        .setTangent(Math.toRadians(43))
                        .lineToXConstantHeading(CYCLE_BACKWARD[2], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))

                        .setTangent(Math.toRadians(43.4))
                        .lineToXConstantHeading(CYCLE_FORWARD[3], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))
                        .setTangent(Math.toRadians(43))
                        .lineToXConstantHeading(CYCLE_BACKWARD[3], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))

                        .setTangent(Math.toRadians(43.4))
                        .lineToXConstantHeading(CYCLE_FORWARD[4], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))
                        .setTangent(Math.toRadians(44))
                        .lineToXConstantHeading(CYCLE_BACKWARD[4], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))
                        .build()
        );

        try {
            pickSpecs.join();
        } catch (Exception e) {}

        left_arm.setPosition(0.55);
        right_arm.setPosition(0.55);
        sleep(200);
    }

    public void giveSampleAndReturn(MecanumDrive drive, Pose2d initialPose, double forwardAmt, double angOffset, double ang, boolean extendWait) {
        extendoExtend(0.13);
        drive.localizer.setPose(initialPose);
        asynchExtendoExtend.start();
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .splineToLinearHeading(new Pose2d(-0.01 + forwardAmt,-0.01 - forwardAmt, Math.toRadians(ang)), Math.toRadians(0.0))
                        .build());
        try {
            asynchExtendoExtend.join();
        } catch (Exception e) {}
        intake.setPower(-1);
        if (extendWait) {
            sleep(175);
        }
        else {
            sleep(215);
        }
        intake.setPower(1);
        extendoExtend(0.2);
        drive.localizer.setPose(initialPose);
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .splineToLinearHeading(new Pose2d(-0.01 + forwardAmt,0.01, (Math.toRadians(-(ang - 5)) + angOffset)), Math.toRadians(0.0))
                        .build());
        drive.localizer.setPose(initialPose);
    }

    Thread asynchExtendoExtend = new Thread(() -> {
        try {
            sleep(400);
            extendoExtend(0.36);
            intake.setPower(-0.185);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    });

    Thread asynchExtendoExtendForThird = new Thread(() -> {
        try {
            sleep(425);
            extendoExtend(0.36);
            intake.setPower(-0.235);
        } catch (Exception e) {
            Thread.currentThread().interrupt();
        }
    });

    public void giveSampleAndReturnThirdTime(MecanumDrive drive, Pose2d initialPose, double forwardAmt, double turnAng, double ang) {
        extendoExtend(0.135);
        drive.localizer.setPose(initialPose);
        asynchExtendoExtendForThird.start();
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .splineToLinearHeading(new Pose2d(-0.01 + forwardAmt,-0.01, Math.toRadians(ang)), Math.toRadians(0.0))
                        .build());
        try {
            asynchExtendoExtendForThird.join();
        } catch (Exception e) {}
        intake.setPower(-1);
        sleep(250);
        intake.setPower(0);
        extendoExtend(0.1325);
        asynchExtendoRetract.start();
        drive.localizer.setPose(initialPose);
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .splineToLinearHeading(new Pose2d(-10,0, Math.toRadians(turnAng)), Math.toRadians(0))
                        .build());
        drive.localizer.setPose(initialPose);
    }

    Thread asynchExtendoRetract = new Thread(() -> {
        try {
            extendoRetract(500);
        } catch (Exception e) {}
    });


    public void extendoExtend(double extendoPos) {
        left_extendo_servo.setPosition(extendoPos);
        right_extendo_servo.setPosition(extendoPos);
        left_intake_chamber_servo.setPosition(0.81325);
        right_intake_chamber_servo.setPosition(0.81325);
    }

    public void extendoRetract(long waitTime) {
        left_intake_chamber_servo.setPosition(0.075);
        right_intake_chamber_servo.setPosition(0.075);
        if (waitTime > 0) sleep(waitTime);
        left_extendo_servo.setPosition(0);
        right_extendo_servo.setPosition(0);
    }

}