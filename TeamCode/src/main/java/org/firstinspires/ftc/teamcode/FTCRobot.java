package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DriveCode.Drive;
import org.firstinspires.ftc.teamcode.DriveCode.Extendo;
import org.firstinspires.ftc.teamcode.DriveCode.Intake;
import org.firstinspires.ftc.teamcode.DriveCode.SampleScoring;
import org.firstinspires.ftc.teamcode.DriveCode.SpecimenScoring;

public class FTCRobot {
    public DcMotor left_front;
    public DcMotor left_back;
    public DcMotor right_front;
    public DcMotor right_back;
    public DcMotor left_arm_motor;
    public DcMotor right_arm_motor;
    public Servo claw_servo;
    public Servo claw_pivot_servo;

    public Servo left_arm;
    public Servo right_arm;

    public DcMotor intake;
    public Servo left_intake_chamber_servo;
    public Servo right_intake_chamber_servo;
    public Servo left_extendo_servo;
    public Servo right_extendo_servo;

    public OurColorSensor ourColorSensor = new OurColorSensor();
    public Drive mecanumDrivetrain = new Drive();
    public Extendo extendo = new Extendo();

    private Intake intakeAndSampleModes = new Intake(OurColorSensor.DetectedColor.BLUE);
    private SpecimenScoring specimenScoring = new SpecimenScoring();
    private SampleScoring sampleScoring = new SampleScoring();

    public void init(HardwareMap hardwareMap) {
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
    public void runInstance(Gamepad gamepad1, Gamepad gamepad2){
        intakeAndSampleModes.runInstance(intake, ourColorSensor, gamepad1, gamepad2);
        extendo.runInstance(left_extendo_servo, right_extendo_servo, left_intake_chamber_servo, right_intake_chamber_servo, gamepad2);
        mecanumDrivetrain.runInstance(left_front, right_front, left_back, right_back, gamepad1);
        specimenScoring.runInstance(left_arm_motor, right_arm_motor, claw_servo, claw_pivot_servo, left_arm, right_arm, gamepad1);
        sampleScoring.runInstance(left_arm_motor, right_arm_motor, claw_servo, claw_pivot_servo, left_arm, right_arm, gamepad2);

    }
    public void runFailSafe() {

        ///arm moved out of the way
        claw_servo.setPosition(0.2);
        claw_pivot_servo.setPosition(0.568);
        left_arm.setPosition(0.071);
        right_arm.setPosition(0.071);

        sleep(300);

        ///extend extendo
        left_extendo_servo.setPosition(0.34);
        right_extendo_servo.setPosition(0.34);
        left_intake_chamber_servo.setPosition(0.81325);
        right_intake_chamber_servo.setPosition(0.81325);

        ///slides moved up
        left_arm_motor.setTargetPosition(1000);
        right_arm_motor.setTargetPosition(1000);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(800);

        ///retract extendo
        left_extendo_servo.setPosition(0);
        right_extendo_servo.setPosition(0);
        left_intake_chamber_servo.setPosition(0.075);
        right_intake_chamber_servo.setPosition(0.075);

        ///slides moved down
        left_arm_motor.setTargetPosition(-1000);
        right_arm_motor.setTargetPosition(-1000);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(500);
        left_arm.setPosition(0.071);
        right_arm.setPosition(0.071);
        sleep(750);

        ///slides stopped and reset
        left_arm_motor.setPower(0);
        right_arm_motor.setPower(0);
        //slides zero set
        left_arm_motor.setTargetPosition(0);
        right_arm_motor.setTargetPosition(0);
        left_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_arm_motor.setTargetPosition(0);
        right_arm_motor.setTargetPosition(0);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(100);

        //slides reset
        left_arm_motor.setTargetPosition(0);
        right_arm_motor.setTargetPosition(0);
        left_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

