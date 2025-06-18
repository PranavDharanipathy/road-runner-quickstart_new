package org.firstinspires.ftc.teamcode.DriveCode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Extendo {

    public static boolean allowExtendo = true;
    private boolean cur_extendo = false;
    private boolean prev_extendo = false;
    public static boolean toggle_extendo = false;
    public static boolean shootThroughBotToggle = false;
    public static double extendoPos = 1; //in = 1 | out = 0

    private boolean prev_gamepad2x = false;
    private boolean cur_gamepad2x = false;

    public static double SHOOT_THROUGH_BOT_CHAMBER_POSITION = 0.275;

    public void runInstance(Servo left_extendo_servo, Servo right_extendo_servo, Servo left_intake_chamber_servo, Servo right_intake_chamber_servo, Gamepad gamepad2) {

        prev_extendo = cur_extendo;
        cur_extendo = gamepad2.dpad_up;

        prev_gamepad2x = cur_gamepad2x;
        cur_gamepad2x = gamepad2.x;

        if (allowExtendo) {
            if (gamepad2.dpad_down) {
                toggle_extendo = false;
                /** IN **/
                left_extendo_servo.setPosition(0);
                right_extendo_servo.setPosition(0);
                left_intake_chamber_servo.setPosition(0.075);
                right_intake_chamber_servo.setPosition(0.075);

                extendoPos = 1;
                shootThroughBotToggle = false;
            } else if (cur_extendo && !prev_extendo) {
                if (toggle_extendo) {
                    /** OUT **/
                    Transfer.clawTransfer = true;
                    left_extendo_servo.setPosition(0.34);
                    right_extendo_servo.setPosition(0.34);
                    left_intake_chamber_servo.setPosition(0.81325);
                    right_intake_chamber_servo.setPosition(0.81325);

                    extendoPos = 0;
                    shootThroughBotToggle = false;
                    Intake.outtakeSpeed = -0.475;
                } else {
                    /** PICK FROM SUBMERSIBLE **/
                    left_extendo_servo.setPosition(0.34);
                    right_extendo_servo.setPosition(0.34);
                    left_intake_chamber_servo.setPosition(0.425);
                    right_intake_chamber_servo.setPosition(0.425);

                    extendoPos = 0;
                    shootThroughBotToggle = false;
                    Intake.outtakeSpeed = -0.8;
                }
                toggle_extendo = !toggle_extendo;
            }
            if (cur_gamepad2x && !prev_gamepad2x) {
                if (!shootThroughBotToggle) {
                    left_intake_chamber_servo.setPosition(SHOOT_THROUGH_BOT_CHAMBER_POSITION);
                    right_intake_chamber_servo.setPosition(SHOOT_THROUGH_BOT_CHAMBER_POSITION);
                    Intake.outtakeSpeed = -0.7;
                }
                else {
                    left_intake_chamber_servo.setPosition(0.075);
                    right_intake_chamber_servo.setPosition(0.075);
                }
                shootThroughBotToggle = !shootThroughBotToggle;
            }
        }
    }

}
