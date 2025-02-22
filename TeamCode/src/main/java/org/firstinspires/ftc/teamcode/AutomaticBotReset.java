package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AutomaticBotReset extends LinearOpMode {

    public void runOpMode() {}

    public void startWhileBlocking(DcMotor left_arm_motor, DcMotor right_arm_motor) {

        left_arm_motor.setTargetPosition(-1000);
        right_arm_motor.setTargetPosition(-1000);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(2000);

        left_arm_motor.setPower(0);
        right_arm_motor.setPower(0);
        left_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
