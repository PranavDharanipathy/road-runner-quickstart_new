package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp (name = "TwoDriverMecanumDrive")
public class TwoDriverMecanumDrive extends OpMode {

    private DcMotor left_front, right_front, left_back, right_back;

    private Servo left_extendo_servo;
    private Servo right_extendo_servo;

    public static double SPEED_MULTIPLIER = 1;

    @Override
    public void init() {

        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

        right_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_extendo_servo = hardwareMap.get(Servo.class, "left_extendo_servo");
        right_extendo_servo = hardwareMap.get(Servo.class, "right_extendo_servo");
        left_extendo_servo.setDirection(Servo.Direction.FORWARD);
        right_extendo_servo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void start() {
        left_extendo_servo.setPosition(0);
        right_extendo_servo.setPosition(0);
    }

    @Override
    public void loop() {

        left_front.setPower(SPEED_MULTIPLIER * (gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));
        left_back.setPower(SPEED_MULTIPLIER * (gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x));
        right_front.setPower(SPEED_MULTIPLIER * (gamepad2.left_stick_y + gamepad2.right_stick_x + gamepad2.left_stick_x));
        right_back.setPower(SPEED_MULTIPLIER * (gamepad2.left_stick_y + gamepad2.right_stick_x - gamepad2.left_stick_x));

        telemetry.addData("left_front", motorDir(SPEED_MULTIPLIER * (gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x)));
        telemetry.addData("left_back", motorDir(SPEED_MULTIPLIER * (gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x)));
        telemetry.addData("right_front", motorDir(SPEED_MULTIPLIER * (gamepad2.left_stick_y + gamepad2.right_stick_x + gamepad2.left_stick_x)));
        telemetry.addData("right_back", motorDir(SPEED_MULTIPLIER * (gamepad2.left_stick_y + gamepad2.right_stick_x - gamepad2.left_stick_x)));
        telemetry.update();
    }

    public String motorDir(double power) {

        String motorDirection;

        if (power > 0) {
            motorDirection = "forward";
        }
        else if (power < 0) {
            motorDirection = "backward";
        }
        else {
            motorDirection = "inactive";
        }

        return motorDirection;
    }
}
