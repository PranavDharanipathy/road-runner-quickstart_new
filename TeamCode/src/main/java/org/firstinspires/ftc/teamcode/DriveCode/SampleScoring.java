package org.firstinspires.ftc.teamcode.DriveCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class SampleScoring {

    private boolean prev_gamepad2lb = false;
    private boolean cur_gamepad2lb = false;
    private boolean prev_gamepad2rb = false;
    private boolean cur_gamepad2rb = false;

    public void runInstance(DcMotor left_arm_motor, DcMotor right_arm_motor, Servo claw_servo, Servo claw_pivot_servo, Servo left_arm, Servo right_arm, Gamepad gamepad2) {

        prev_gamepad2lb = cur_gamepad2lb;
        cur_gamepad2lb = gamepad2.left_bumper;

        prev_gamepad2rb = cur_gamepad2rb;
        cur_gamepad2rb = gamepad2.right_bumper;

        if (cur_gamepad2rb && !prev_gamepad2rb) {
            PresetCommands.scoreSamplePose();
        }
        else if (cur_gamepad2lb && !prev_gamepad2lb) {
            PresetCommands.pickSamplePose();
        }

        scoreSamplePose(left_arm_motor, right_arm_motor, claw_servo, claw_pivot_servo, left_arm, right_arm);
        pickSamplePose(left_arm_motor, right_arm_motor, claw_servo, claw_pivot_servo, left_arm, right_arm);
    }

    private void scoreSamplePose(DcMotor left_arm_motor, DcMotor right_arm_motor, Servo claw_servo, Servo claw_pivot_servo, Servo left_arm, Servo right_arm) {

        //score sample
        if (PresetCommands.scoreSampleStages == 1 && PresetCommands.scoreSampleTimer.milliseconds() >= 1) {
            claw_pivot_servo.setPosition(0.92);
            claw_servo.setPosition(PresetCommands.CLAW_CLOSED);
            PresetCommands.scoreSampleStages++;
        }
        if (PresetCommands.scoreSampleStages == 2 && PresetCommands.scoreSampleTimer.milliseconds() >= 300) {
            left_arm.setPosition(0.3);
            right_arm.setPosition(0.3);
            PresetCommands.scoreSampleStages++;
        }
        if (PresetCommands.scoreSampleStages == 3 && PresetCommands.scoreSampleTimer.milliseconds() >= 625) {
            BackgroundActionProcessing.slidesToPose(PresetCommands.SLIDES_UP_POSITION, left_arm_motor, right_arm_motor);
            PresetCommands.scoreSampleStages++;
        }
        if (PresetCommands.scoreSampleStages == 4 && PresetCommands.scoreSampleTimer.milliseconds() >= 1075) {
            left_arm.setPosition(0.485);
            right_arm.setPosition(0.485);
            PresetCommands.scoreSampleStages++;
        }
        if (PresetCommands.scoreSampleStages == 5 && PresetCommands.scoreSampleTimer.milliseconds() >= 1275) {
            claw_pivot_servo.setPosition(0.875);
        }
    }

    private void pickSamplePose(DcMotor left_arm_motor, DcMotor right_arm_motor, Servo claw_servo, Servo claw_pivot_servo, Servo left_arm, Servo right_arm) {

        //pick sample
        if (PresetCommands.pickSampleStages == 1 && PresetCommands.pickSampleTimer.milliseconds() >= 1) {
            claw_pivot_servo.setPosition(0.865); //goes a bit down before releasing to prevent the sample from getting flung out
            claw_servo.setPosition(PresetCommands.CLAW_OPEN);
            PresetCommands.pickSampleStages++;
        }
        if (PresetCommands.pickSampleStages == 2 && PresetCommands.pickSampleTimer.milliseconds() >= 300) {
            left_arm.setPosition(Transfer.TRANSFER_POSITION);
            right_arm.setPosition(Transfer.TRANSFER_POSITION);
            PresetCommands.pickSampleStages++;
        }
        if (PresetCommands.pickSampleStages == 3 && PresetCommands.pickSampleTimer.milliseconds() >= 630) {
            claw_pivot_servo.setPosition(Transfer.TRANSFER_PIVOT);
            BackgroundActionProcessing.slidesToPose(PresetCommands.SLIDES_DOWN_POSITION, left_arm_motor, right_arm_motor);
            if (PresetCommands.pickSampleTimer.milliseconds() >= 1000) {
                PresetCommands.pickSampleStages = 4; // increased at final step to prevent the slides from trying to go down after task has been completed
            }
        }
    }

    public static void returnToPickSamplePose(DcMotor left_arm_motor, DcMotor right_arm_motor, Servo claw_servo, Servo claw_pivot_servo, Servo left_arm, Servo right_arm) {

        PresetCommands.transferToClawStages = 0;
        PresetCommands.scoreSampleStages = 0;
        PresetCommands.pickSpecimenAfterScoringStages = 0;
        PresetCommands.scoreSpecimenStages = 0;
        PresetCommands.pickSpecimenStages = 0;
        PresetCommands.pickSampleStages = 0;
        PresetCommands.returnToPickSamplePoseStages = 1;

        left_arm.setPosition(Transfer.TRANSFER_POSITION);
        right_arm.setPosition(Transfer.TRANSFER_POSITION);
        claw_servo.setPosition(PresetCommands.CLAW_OPEN);
        claw_pivot_servo.setPosition(Transfer.TRANSFER_PIVOT);
        BackgroundActionProcessing.slidesToPose(PresetCommands.SLIDES_DOWN_POSITION, left_arm_motor, right_arm_motor);
    }

}