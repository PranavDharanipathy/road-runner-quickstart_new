package org.firstinspires.ftc.teamcode.DriveCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PresetCommands {

    public static int SLIDES_UP_POSITION = 740;
    public static int SLIDES_DOWN_POSITION = -38;
    public static int SLIDES_SCORE_SPEC_POSITION = 370;
    public static int prev_slidesScoreSpecPosition;
    public static int SLIDES_PICK_SPEC_POSITION = 210;
    public static int SLIDES_MINIMUM_THRESHOLD = 10;

    public static double CLAW_OPEN = 0.175;
    public static double CLAW_CLOSED = 0.35;

    public static ElapsedTime scoreSpecimenTimer = new ElapsedTime();
    public static ElapsedTime pickSpecimenTimer = new ElapsedTime();
    public static ElapsedTime pickSpecimenAfterScoringTimer = new ElapsedTime();
    public static ElapsedTime pickSampleTimer = new ElapsedTime();
    public static ElapsedTime scoreSampleTimer = new ElapsedTime();
    public static ElapsedTime transferToClawTimer = new ElapsedTime();
    public static int scoreSpecimenStages = 0;
    public static int pickSpecimenStages = 0;
    public static int pickSpecimenAfterScoringStages = 0;
    public static int scoreSampleStages = 0;
    public static int pickSampleStages = 0;
    public static int transferToClawStages = 0;
    public static int returnToPickSamplePoseStages = 0;

    /// TRANSFER
    public static void transferToClawPose() {

        transferToClawTimer.reset();
        scoreSampleStages = 0;
        pickSpecimenAfterScoringStages = 0;
        scoreSpecimenStages = 0;
        pickSpecimenStages = 0;
        pickSampleStages = 0;
        returnToPickSamplePoseStages = 0;
        transferToClawStages = 1;
    }

    /// SPEC
    public static void pickSpecimenPose() {

        pickSpecimenTimer.reset();
        pickSampleStages = 0;
        scoreSampleStages = 0;
        pickSpecimenAfterScoringStages = 0;
        scoreSpecimenStages = 0;
        transferToClawStages = 0;
        returnToPickSamplePoseStages = 0;
        pickSpecimenStages = 1;
    }

    public static void scoreSpecimenPose() {

        scoreSpecimenTimer.reset();
        pickSampleStages = 0;
        scoreSampleStages = 0;
        pickSpecimenAfterScoringStages = 0;
        pickSpecimenStages = 0;
        transferToClawStages = 0;
        returnToPickSamplePoseStages = 0;
        scoreSpecimenStages = 1;
    }

    public static void pickSpecimenPoseAfterScoring() {

        pickSpecimenAfterScoringTimer.reset();
        pickSampleStages = 0;
        scoreSampleStages = 0;
        scoreSpecimenStages = 0;
        pickSpecimenStages = 0;
        transferToClawStages = 0;
        returnToPickSamplePoseStages = 0;
        pickSpecimenAfterScoringStages = 1;
    }

    /// SAMPLE
    public static void pickSamplePose() {

        pickSampleTimer.reset();
        transferToClawStages = 0;
        scoreSampleStages = 0;
        pickSpecimenAfterScoringStages = 0;
        scoreSpecimenStages = 0;
        pickSpecimenStages = 0;
        returnToPickSamplePoseStages = 0;
        pickSampleStages = 1;
    }

    public static void scoreSamplePose() {

        scoreSampleTimer.reset();
        transferToClawStages = 0;
        pickSampleStages = 0;
        pickSpecimenAfterScoringStages = 0;
        pickSpecimenStages = 0;
        scoreSpecimenStages = 0;
        returnToPickSamplePoseStages = 0;
        scoreSampleStages = 1;
    }
}
