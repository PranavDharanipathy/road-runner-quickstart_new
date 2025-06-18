package org.firstinspires.ftc.teamcode.DriveCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class SpecimenScoring {

    private boolean prev_gamepad1lb = false;
    private boolean cur_gamepad1lb = false;
    private boolean prev_gamepad1lt = false;
    private boolean cur_gamepad1lt = false;
    private boolean prev_gamepad1rb = false;
    private boolean cur_gamepad1rb = false;
    private boolean prev_gamepad1rt = false;
    private boolean cur_gamepad1rt = false;
    private boolean prev_gamepad1dpad_up = false;
    private boolean cur_gamepad1dpad_up = false;
    private boolean prev_gamepad1dpad_down = false;
    private boolean cur_gamepad1dpad_down = false;

    public void runInstance(DcMotor left_arm_motor, DcMotor right_arm_motor, Servo claw_servo, Servo claw_pivot_servo, Servo left_arm, Servo right_arm, Gamepad gamepad1) {

        prev_gamepad1lb = cur_gamepad1lb;
        cur_gamepad1lb = gamepad1.left_bumper;

        prev_gamepad1lt = cur_gamepad1lt;
        cur_gamepad1lt = gamepad1.left_trigger > 0.078;

        prev_gamepad1rb = cur_gamepad1rb;
        cur_gamepad1rb = gamepad1.right_bumper;

        prev_gamepad1rt = cur_gamepad1rt;
        cur_gamepad1rt = gamepad1.right_trigger > 0.078;

        if (cur_gamepad1lb && !prev_gamepad1lb) {
            PresetCommands.pickSpecimenPose();
        }
        else if (cur_gamepad1lt && !prev_gamepad1lt) {
            PresetCommands.scoreSpecimenPose();
        }
        else if (cur_gamepad1rb && !prev_gamepad1rb) {
            SampleScoring.returnToPickSamplePose(left_arm_motor, right_arm_motor, claw_servo, claw_pivot_servo, left_arm, right_arm);
        }
        else if (cur_gamepad1rt && !prev_gamepad1rt) {
            PresetCommands.pickSpecimenPoseAfterScoring();
        }

        debugSlideInaccuracies(left_arm_motor, gamepad1);

        scoreSpec(left_arm_motor, right_arm_motor, claw_servo, claw_pivot_servo, left_arm, right_arm);
        pickSpec(left_arm_motor, right_arm_motor, claw_servo, claw_pivot_servo, left_arm, right_arm);
        pickSpecAfterScoring(left_arm_motor, right_arm_motor, claw_servo, claw_pivot_servo, left_arm, right_arm);
    }

    private void debugSlideInaccuracies(DcMotor left_arm_motor, Gamepad gamepad1) {

        prev_gamepad1dpad_up = cur_gamepad1dpad_up;
        cur_gamepad1dpad_up = gamepad1.dpad_up;

        prev_gamepad1dpad_down = cur_gamepad1dpad_down;
        cur_gamepad1dpad_down = gamepad1.dpad_down;

        if (cur_gamepad1dpad_up && !prev_gamepad1dpad_up) {
            if (left_arm_motor.getTargetPosition() == PresetCommands.SLIDES_SCORE_SPEC_POSITION) {
                PresetCommands.SLIDES_SCORE_SPEC_POSITION+=25;
            }
            else if (left_arm_motor.getTargetPosition() == PresetCommands.SLIDES_PICK_SPEC_POSITION) {
                PresetCommands.SLIDES_PICK_SPEC_POSITION+=25;
            }
        }
        else if (cur_gamepad1dpad_down && !prev_gamepad1dpad_down) {
            if (left_arm_motor.getTargetPosition() == PresetCommands.SLIDES_SCORE_SPEC_POSITION) {
                PresetCommands.SLIDES_SCORE_SPEC_POSITION-=25;
            }
            else if (left_arm_motor.getTargetPosition() == PresetCommands.SLIDES_PICK_SPEC_POSITION) {
                PresetCommands.SLIDES_PICK_SPEC_POSITION-=25;
            }
        }
    }

    private void scoreSpec(DcMotor left_arm_motor, DcMotor right_arm_motor, Servo claw_servo, Servo claw_pivot_servo, Servo left_arm, Servo right_arm) {

        //score spec
        if (PresetCommands.scoreSpecimenStages == 1 && PresetCommands.scoreSpecimenTimer.milliseconds() >= 1) {
            claw_pivot_servo.setPosition(0.83);
            claw_servo.setPosition(PresetCommands.CLAW_CLOSED);
            PresetCommands.scoreSpecimenStages++;
        }
        if (PresetCommands.scoreSpecimenStages == 2 && PresetCommands.scoreSpecimenTimer.milliseconds() >= 50) {
            BackgroundActionProcessing.slidesToPose(PresetCommands.SLIDES_SCORE_SPEC_POSITION, left_arm_motor, right_arm_motor);
            PresetCommands.scoreSpecimenStages++;
        }
        if (PresetCommands.scoreSpecimenStages == 3 && PresetCommands.scoreSpecimenTimer.milliseconds() >= 250) {
            claw_pivot_servo.setPosition(1);
            PresetCommands.scoreSpecimenStages++;
        }
        if (PresetCommands.scoreSpecimenStages == 4 && PresetCommands.scoreSpecimenTimer.milliseconds() >= 600) {
            left_arm.setPosition(0.65);
            right_arm.setPosition(0.65);
            PresetCommands.scoreSpecimenStages++;
        }
        if (PresetCommands.scoreSpecimenStages == 5 && PresetCommands.scoreSpecimenTimer.milliseconds() >= 700) {
            left_arm.setPosition(0.1);
            right_arm.setPosition(0.1);
            PresetCommands.scoreSpecimenStages++;
        }
        if (PresetCommands.scoreSpecimenStages == 6 && PresetCommands.scoreSpecimenTimer.milliseconds() >= 715) {
            claw_pivot_servo.setPosition(0.8175);
        }
    }

    private void pickSpec(DcMotor left_arm_motor, DcMotor right_arm_motor, Servo claw_servo, Servo claw_pivot_servo, Servo left_arm, Servo right_arm) {

        //pick spec
        if (PresetCommands.pickSpecimenStages == 1 && PresetCommands.pickSpecimenTimer.milliseconds() >= 1) {
            claw_servo.setPosition(PresetCommands.CLAW_OPEN);
            PresetCommands.pickSpecimenStages++;
        }
        if (PresetCommands.pickSpecimenStages == 2 && PresetCommands.pickSpecimenTimer.milliseconds() >= 50) {
            BackgroundActionProcessing.slidesToPose(PresetCommands.SLIDES_PICK_SPEC_POSITION, left_arm_motor, right_arm_motor);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
            PresetCommands.pickSpecimenStages++;
        }
        if (PresetCommands.pickSpecimenStages == 3 && PresetCommands.pickSpecimenTimer.milliseconds() >= 250) {
            claw_pivot_servo.setPosition(0.83);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
        }
    }

    private void pickSpecAfterScoring(DcMotor left_arm_motor, DcMotor right_arm_motor, Servo claw_servo, Servo claw_pivot_servo, Servo left_arm, Servo right_arm) {

        //pick spec after scoring
        if (PresetCommands.pickSpecimenAfterScoringStages == 1 && PresetCommands.pickSpecimenAfterScoringTimer.milliseconds() >= 1) {
            claw_servo.setPosition(PresetCommands.CLAW_OPEN);
            PresetCommands.pickSpecimenAfterScoringStages++;
        }
        if (PresetCommands.pickSpecimenAfterScoringStages == 2 && PresetCommands.pickSpecimenAfterScoringTimer.milliseconds() >= 50) {
            claw_pivot_servo.setPosition(0.83);
            PresetCommands.pickSpecimenAfterScoringStages++;
        }
        if (PresetCommands.pickSpecimenAfterScoringStages == 3 && PresetCommands.pickSpecimenAfterScoringTimer.milliseconds() >= 250) {
            BackgroundActionProcessing.slidesToPose(PresetCommands.SLIDES_PICK_SPEC_POSITION, left_arm_motor, right_arm_motor);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
            PresetCommands.pickSpecimenAfterScoringStages++;
        }
        if (PresetCommands.pickSpecimenAfterScoringStages == 4 && PresetCommands.pickSpecimenAfterScoringTimer.milliseconds() >= 550) {
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
        }
    }

}
