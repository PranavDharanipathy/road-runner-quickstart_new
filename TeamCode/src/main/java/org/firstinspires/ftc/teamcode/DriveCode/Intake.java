package org.firstinspires.ftc.teamcode.DriveCode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OurColorSensor;

public class Intake {

    private OurColorSensor.DetectedColor ALLIANCE_COLOR;
    private OurColorSensor.DetectedColor wantedColor = OurColorSensor.DetectedColor.OTHER;

    public static double outtakeSpeed = -0.6;

    private boolean canIntake = true;

    private boolean allowNeutral = true;
    public static boolean allowIntaking = true;

    private ElapsedTime automaticOuttakeTimer = new ElapsedTime();
    private boolean automaticOuttakeStateHolder = false;

    private ElapsedTime setIntakeModeRumbleTimer = new ElapsedTime();
    private int setIntakeModeRumbleDuration = 150;

    private boolean prev_gamepad2a = false;
    private boolean cur_gamepad2a = false;

    public Intake(OurColorSensor.DetectedColor allianceColor) { ALLIANCE_COLOR = allianceColor; }

    public void runInstance(DcMotor intake, OurColorSensor ourColorSensor, Gamepad gamepad1, Gamepad gamepad2) {

        //set mode
        if (gamepad1.x) {//alliance color
            wantedColor = ALLIANCE_COLOR;
            allowNeutral = false;
        }
        else if(gamepad1.b) {//neutral
            wantedColor = ALLIANCE_COLOR;
            allowNeutral = false;
        }
        else if(gamepad1.a) {//both
            wantedColor = ALLIANCE_COLOR;
            allowNeutral = true;
        }

        if (setIntakeModeRumbleTimer.milliseconds() >= setIntakeModeRumbleDuration && wantedColor == OurColorSensor.DetectedColor.OTHER) {
            gamepad1.rumble(setIntakeModeRumbleDuration);
            gamepad2.rumble(setIntakeModeRumbleDuration);
            setIntakeModeRumbleTimer.reset();
        }

        prev_gamepad2a = cur_gamepad2a;
        cur_gamepad2a = gamepad2.a;

        sampleTypeChoice(ourColorSensor);

        if (allowIntaking & wantedColor != OurColorSensor.DetectedColor.OTHER) {
            if (Transfer.clawTransfer && (ourColorSensor.getColor() == wantedColor || ourColorSensor.getColor() == OurColorSensor.DetectedColor.YELLOW && allowNeutral)) {
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
                Transfer.clawTransfer = false;
            }
            if (cur_gamepad2a && !prev_gamepad2a) {
                Extendo.toggle_extendo = false;
                PresetCommands.transferToClawPose();
            }
            if (gamepad2.right_trigger > 0.1 && canIntake) {
                intake.setPower(1); //intake
            } else if (!canIntake) {
                automaticOuttakeStateHolder = true; /*outtake*/ /** [[actual outtaking is done in withinLoopProcessing()]] **/
            } else if (gamepad2.left_trigger > 0.1) {
                intake.setPower(-1); //outtake
            } else {
                intake.setPower(0);
            }
        }

        if (automaticOuttakeStateHolder) {
            intake.setPower(outtakeSpeed);
            if (automaticOuttakeTimer.milliseconds() >= 1200) {
                automaticOuttakeStateHolder = false;
                intake.setPower(0);
            }
        }
        else automaticOuttakeTimer.reset();
    }

    private void sampleTypeChoice(OurColorSensor ourColorSensor) {

        if (ourColorSensor.getColor() != wantedColor) {

            if (ourColorSensor.getColor() == OurColorSensor.DetectedColor.OTHER) {
                canIntake = true;
            }
            else if (allowNeutral && ourColorSensor.getColor() == OurColorSensor.DetectedColor.YELLOW) {
                canIntake = true;
            }
            else canIntake = false;
        }
        else if (ourColorSensor.getColor() == wantedColor) {
            canIntake = true;
        }
        else canIntake = true;
    }

}
