package org.firstinspires.ftc.teamcode.DriveCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {

    public static boolean clawTransfer = true;

    public static double TRANSFER_PIVOT = 0.568;
    private static double ARM_AFTER_TRANSFER = 0.11;
    public static double TRANSFER_POSITION = 0.071;

    public void runInstance(DcMotor intake, Servo claw_pivot_servo, Servo claw_servo, Servo left_arm, Servo right_arm) {

    //transfer sample to claw
        if (PresetCommands.transferToClawStages == 1) {
        Intake.allowIntaking = false;
        Extendo.allowExtendo = false;
        claw_pivot_servo.setPosition(TRANSFER_PIVOT);
        left_arm.setPosition(TRANSFER_POSITION);
        right_arm.setPosition(TRANSFER_POSITION);
            PresetCommands.transferToClawStages++;
    }
        if (PresetCommands.transferToClawStages == 2 && PresetCommands.transferToClawTimer.milliseconds() >= 400) {
        intake.setPower(-0.75);
            PresetCommands.transferToClawStages++;
    }
        if (PresetCommands.transferToClawStages == 3 && PresetCommands.transferToClawTimer.milliseconds() >= (700)) {
        claw_servo.setPosition(PresetCommands.CLAW_CLOSED);
            PresetCommands.transferToClawStages++;
    }
        if (PresetCommands.transferToClawStages == 4 && PresetCommands.transferToClawTimer.milliseconds() >= (735)) {
            PresetCommands.transferToClawStages = 0;
        left_arm.setPosition(ARM_AFTER_TRANSFER);
        right_arm.setPosition(ARM_AFTER_TRANSFER);
    }
        if (PresetCommands.transferToClawStages == 0) {
            Intake.allowIntaking = true;
            Extendo.allowExtendo = true;
        }

    }
}
