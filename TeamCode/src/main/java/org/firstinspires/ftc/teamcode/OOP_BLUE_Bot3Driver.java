package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DriveCode.*;

@TeleOp (name = "OOP_BLUE_Bot3Driver")
public class OOP_BLUE_Bot3Driver extends OpMode {

    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;
    private DcMotor left_arm_motor;
    private DcMotor right_arm_motor;
    private Servo claw_servo;
    private Servo claw_pivot_servo;

    private Servo left_arm;
    private Servo right_arm;

    private DcMotor intake;
    private Servo left_intake_chamber_servo;
    private Servo right_intake_chamber_servo;
    private Servo left_extendo_servo;
    private Servo right_extendo_servo;

    private OurColorSensor ourColorSensor = new OurColorSensor();

    private Drive mecanumDrivetrain = new Drive();
    private Extendo extendo = new Extendo();
    private Intake intakeAndSampleModes = new Intake(OurColorSensor.DetectedColor.BLUE);
    private SpecimenScoring specimenScoring = new SpecimenScoring();
    private SampleScoring sampleScoring = new SampleScoring();
    private BackgroundActionProcessing backgroundActionProcessing = new BackgroundActionProcessing();


    @Override
    public void init() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
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

        claw_servo = hardwareMap.get(Servo.class, "claw_servo");
        claw_servo.setDirection(Servo.Direction.FORWARD);
        claw_pivot_servo = hardwareMap.get(Servo.class, "claw_pivot_servo");

        intake = hardwareMap.get(DcMotor.class, "intake");

        left_intake_chamber_servo = hardwareMap.get(Servo.class, "left_intake_servo");
        right_intake_chamber_servo = hardwareMap.get(Servo.class, "right_intake_servo");

        left_extendo_servo = hardwareMap.get(Servo.class, "left_extendo_servo");
        right_extendo_servo = hardwareMap.get(Servo.class, "right_extendo_servo");
        left_extendo_servo.setDirection(Servo.Direction.FORWARD);
        right_extendo_servo.setDirection(Servo.Direction.FORWARD);

        left_intake_chamber_servo.setDirection(REVERSE);
        right_intake_chamber_servo.setDirection(REVERSE);

        right_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_intake_chamber_servo.setDirection(Servo.Direction.REVERSE);
        right_intake_chamber_servo.setDirection(Servo.Direction.REVERSE);

        ourColorSensor.initialize(hardwareMap);
    }

    @Override
    public void start() {

        PostAutonomousRobotReset robot = new PostAutonomousRobotReset(
                claw_servo, claw_pivot_servo,
                left_arm, right_arm,
                left_extendo_servo, right_extendo_servo,
                left_intake_chamber_servo, right_intake_chamber_servo,
                left_arm_motor, right_arm_motor
        );

        robot.runFailSafe();
    }

    @Override
    public void loop() {


        intakeAndSampleModes.runInstance(intake, ourColorSensor, gamepad1, gamepad2);
        extendo.runInstance(left_extendo_servo, right_extendo_servo, left_intake_chamber_servo, right_intake_chamber_servo, gamepad2);
        mecanumDrivetrain.runInstance(left_front, right_front, left_back, right_back, gamepad1);
        specimenScoring.runInstance(left_arm_motor, right_arm_motor, claw_servo, claw_pivot_servo, left_arm, right_arm, gamepad1);
        sampleScoring.runInstance(left_arm_motor, right_arm_motor, claw_servo, claw_pivot_servo, left_arm, right_arm, gamepad2);
        backgroundActionProcessing.handle(left_arm_motor, right_arm_motor, left_arm, right_arm);
    }
}
