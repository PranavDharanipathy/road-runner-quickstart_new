package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.FTCRobot;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DriveCode.*;

@TeleOp (name = "OOP_BLUE_Bot3Driver")
public class OOP_BLUE_Bot3Driver extends OpMode {
    private FTCRobot ftcRobot =new FTCRobot();;

      private BackgroundActionProcessing backgroundActionProcessing = new BackgroundActionProcessing();


    @Override
    public void init() {
        ftcRobot.init(hardwareMap);
    }

    @Override
    public void start() {

        ftcRobot.runFailSafe();
    }

    @Override
    public void loop() {
        ftcRobot.runInstance(gamepad1, gamepad2);

        backgroundActionProcessing.handle(ftcRobot.left_arm_motor,ftcRobot.right_arm_motor, ftcRobot.left_arm, ftcRobot.right_arm);
    }
}
