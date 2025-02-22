package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp (name = "Spec Pick")
public class specpick extends LinearOpMode {

    private DcMotor left_arm_motor;
    private DcMotor right_arm_motor;
    private Servo claw_pivot_servo;
    private Servo claw_servo;

    private Servo left_arm;
    private Servo right_arm;

    public static double ARM = 0.82;
    public static double PIVOT = 0.875;
    public static double CLAW = 0.35;
    public static double SLIDES = 170;

    @Override
    public void runOpMode() {

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

        waitForStart();

        while (opModeIsActive()) {
            left_arm.setPosition(ARM);
            right_arm.setPosition(ARM);

            left_arm_motor.setTargetPosition((int) SLIDES);
            right_arm_motor.setTargetPosition((int) SLIDES);
            left_arm_motor.setPower(1);
            right_arm_motor.setPower(1);
            left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            claw_pivot_servo.setPosition(PIVOT);

            claw_servo.setPosition(CLAW);
        }
    }
}
