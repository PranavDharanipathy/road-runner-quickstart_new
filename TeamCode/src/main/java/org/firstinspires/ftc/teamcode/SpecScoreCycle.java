package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "test")
public class SpecScoreCycle extends LinearOpMode {

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
    private static int SLIDES_SCORE_SPEC_POSITION = 370;
    private final static int SLIDES_PICK_SPEC_POSITION = 200;

    //                                             #1    #2  #3  #4  #5
    private final static double[] CYCLE_FORWARD = {13.5, 14, 14, 15, 14.5};
    private final static double[] CYCLE_BACKWARD = {-23, -23, -23.5, -23.5, -24};

    public void slidesToPose(int targetPosition) {
        left_arm_motor.setTargetPosition(targetPosition);
        right_arm_motor.setTargetPosition(targetPosition);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    ///THREADS
    Thread pickSpecs = new Thread(() -> {
        try  {
            //sleep(1275);
            ///1st cycle
            claw_pivot_servo.setPosition(0.87);
            claw_servo.setPosition(0.35);
            slidesToPose(SLIDES_SCORE_SPEC_POSITION-4);
            sleep(200);
            claw_pivot_servo.setPosition(1);
            left_arm.setPosition(0.3);
            right_arm.setPosition(0.3);
            sleep(25);
            left_arm.setPosition(0.1285);
            right_arm.setPosition(0.1285);
            claw_pivot_servo.setPosition(0.775);
            sleep(1650);
            claw_servo.setPosition(0.1875);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
            slidesToPose(SLIDES_PICK_SPEC_POSITION);
            claw_pivot_servo.setPosition(0.87);

            ///2nd cycle
            sleep(1700);
            claw_pivot_servo.setPosition(0.87);
            claw_servo.setPosition(0.35);
            slidesToPose(SLIDES_SCORE_SPEC_POSITION);
            sleep(200);
            claw_pivot_servo.setPosition(1);
            left_arm.setPosition(0.3);
            right_arm.setPosition(0.3);
            sleep(25);
            left_arm.setPosition(0.125);
            right_arm.setPosition(0.125);
            claw_pivot_servo.setPosition(0.775);
            sleep(1700);
            claw_servo.setPosition(0.1875);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
            slidesToPose(SLIDES_PICK_SPEC_POSITION);
            claw_pivot_servo.setPosition(0.87);

            ///3rd cycle
            sleep(1720);
            claw_pivot_servo.setPosition(0.87);
            claw_servo.setPosition(0.35);
            slidesToPose(SLIDES_SCORE_SPEC_POSITION);
            sleep(200);
            claw_pivot_servo.setPosition(1);
            left_arm.setPosition(0.3);
            right_arm.setPosition(0.3);
            sleep(25);
            left_arm.setPosition(0.125);
            right_arm.setPosition(0.125);
            claw_pivot_servo.setPosition(0.775);
            sleep(1800);
            claw_servo.setPosition(0.1875);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
            slidesToPose(SLIDES_PICK_SPEC_POSITION);
            claw_pivot_servo.setPosition(0.87);

            ///4th cycle
            sleep(1730);
            claw_pivot_servo.setPosition(0.87);
            claw_servo.setPosition(0.35);
            slidesToPose(SLIDES_SCORE_SPEC_POSITION + 5);
            sleep(200);
            claw_pivot_servo.setPosition(1);
            left_arm.setPosition(0.3);
            right_arm.setPosition(0.3);
            sleep(25);
            left_arm.setPosition(0.125);
            right_arm.setPosition(0.125);
            claw_pivot_servo.setPosition(0.775);
            sleep(1750);
            claw_servo.setPosition(0.1875);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
            slidesToPose(SLIDES_PICK_SPEC_POSITION);
            claw_pivot_servo.setPosition(0.87);

            ///5th cycle
            sleep(1650);
            claw_pivot_servo.setPosition(0.87);
            claw_servo.setPosition(0.35);
            slidesToPose(SLIDES_SCORE_SPEC_POSITION + 11);
            sleep(200);
            claw_pivot_servo.setPosition(1);
            left_arm.setPosition(0.3);
            right_arm.setPosition(0.3);
            sleep(25);
            left_arm.setPosition(0.125);
            right_arm.setPosition(0.125);
            claw_pivot_servo.setPosition(0.775);
            sleep(1800);
            claw_servo.setPosition(0.1875);
            left_arm.setPosition(0.1175);
            right_arm.setPosition(0.1175);
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


        Pose2d cyclePose = new Pose2d(-22,0, Math.toRadians(0));
        Pose2d nextPose = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, cyclePose);


        //pose initialization
        extendoRetract(0);

        left_arm.setPosition(0.2);
        right_arm.setPosition(0.2);
        sleep(1500);
        claw_servo.setPosition(0.1875);
        claw_pivot_servo.setPosition(0.87);
        slidesToPose(SLIDES_SCORE_SPEC_POSITION);
        sleep(500);
        left_arm.setPosition(0.82);
        right_arm.setPosition(0.82);


        waitForStart();
        if (isStopRequested()) return;


        drive.localizer.setPose(cyclePose);
        pickSpecs.start();
        Actions.runBlocking(
                drive.actionBuilder(cyclePose)
                        .setTangent(Math.toRadians(43.5))
                        .lineToXConstantHeading(CYCLE_FORWARD[0], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))
                        .lineToXConstantHeading(CYCLE_BACKWARD[0], new TranslationalVelConstraint(100), new ProfileAccelConstraint(-50,50))

                        .setTangent(Math.toRadians(44))
                        .lineToXConstantHeading(CYCLE_FORWARD[1], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))
                        .lineToXConstantHeading(CYCLE_BACKWARD[1], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))

                        .setTangent(Math.toRadians(43.85))
                        .lineToXConstantHeading(CYCLE_FORWARD[2], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))
                        .lineToXConstantHeading(CYCLE_BACKWARD[2], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))

                        .setTangent(Math.toRadians(43.375))
                        .lineToXConstantHeading(CYCLE_FORWARD[3], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))
                        .setTangent(Math.toRadians(39))
                        .lineToXConstantHeading(CYCLE_BACKWARD[3], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))

                        .setTangent(Math.toRadians(43.375))
                        .lineToXConstantHeading(CYCLE_FORWARD[4], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))
                        .setTangent(Math.toRadians(40.5))
                        .lineToXConstantHeading(CYCLE_BACKWARD[4], new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50,50))
                        .build()
        );


        try {
            pickSpecs.join();
        } catch (Exception e) {}
    }

    public void extendoRetract(long waitTime) {
        left_intake_chamber_servo.setPosition(0.075);
        right_intake_chamber_servo.setPosition(0.075);
        if (waitTime > 0) sleep(waitTime);
        left_extendo_servo.setPosition(0);
        right_extendo_servo.setPosition(0);
    }

}