package org.firstinspires.ftc.teamcode.DriveCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BackgroundActionProcessing {

    private ElapsedTime returnToPickSamplePoseMoveArmOutOfExtendoPath = new ElapsedTime();
    
    public void handle(DcMotor left_arm_motor, DcMotor right_arm_motor, Servo left_arm, Servo right_arm) {
        
        // release slides power
        if (left_arm_motor.getCurrentPosition() <= PresetCommands.SLIDES_MINIMUM_THRESHOLD && left_arm_motor.getTargetPosition() == PresetCommands.SLIDES_DOWN_POSITION) {
            left_arm_motor.setPower(0);
            right_arm_motor.setPower(0);
            left_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (left_arm_motor.getTargetPosition() != PresetCommands.SLIDES_PICK_SPEC_POSITION && left_arm_motor.getTargetPosition() != PresetCommands.SLIDES_DOWN_POSITION && left_arm_motor.getTargetPosition() != PresetCommands.SLIDES_UP_POSITION) {
            if (PresetCommands.SLIDES_SCORE_SPEC_POSITION != PresetCommands.prev_slidesScoreSpecPosition) {
                slidesToPose(PresetCommands.SLIDES_SCORE_SPEC_POSITION, left_arm_motor, right_arm_motor);
                PresetCommands.prev_slidesScoreSpecPosition = PresetCommands.SLIDES_SCORE_SPEC_POSITION;
            }
        }
        if (Extendo.extendoPos == 1 && PresetCommands.returnToPickSamplePoseStages != 2 && (PresetCommands.returnToPickSamplePoseStages == 1 || PresetCommands.pickSampleStages > 1 /*stages 2 & 3*/)) {
            if (returnToPickSamplePoseMoveArmOutOfExtendoPath.milliseconds() <= 250) {
                left_arm.setPosition(0.15);
                right_arm.setPosition(0.15);
            }
            else {
                left_arm.setPosition(0.1168);
                right_arm.setPosition(0.1168);
                PresetCommands.returnToPickSamplePoseStages = 2;
            }
        }
        else returnToPickSamplePoseMoveArmOutOfExtendoPath.reset();

    }

    public static void slidesToPose(int targetPosition, DcMotor left_arm_motor, DcMotor right_arm_motor) {
        left_arm_motor.setTargetPosition(targetPosition);
        right_arm_motor.setTargetPosition(targetPosition);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
}
