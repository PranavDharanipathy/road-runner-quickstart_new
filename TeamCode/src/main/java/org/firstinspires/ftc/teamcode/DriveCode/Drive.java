package org.firstinspires.ftc.teamcode.DriveCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Drive {

    private double driveLeftX_debugger;
    private double driveLeftY_debugger;
    private double driveRightX_debugger;
    private final static double joyStickMargin = 0.004;

    public void runInstance(DcMotor left_front, DcMotor right_front, DcMotor left_back, DcMotor right_back, Gamepad gamepad1) {

        if (Math.abs(gamepad1.left_stick_x) >= joyStickMargin) {
            driveLeftX_debugger = gamepad1.left_stick_x;
        } else driveLeftX_debugger = 0;

        if (Math.abs(gamepad1.left_stick_y) >= joyStickMargin) {
            driveLeftY_debugger = gamepad1.left_stick_y;
        } else driveLeftY_debugger = 0;

        if (Math.abs(gamepad1.right_stick_x) >= joyStickMargin) {
            driveRightX_debugger = gamepad1.right_stick_x;
        } else driveRightX_debugger = 0;

        left_front.setPower(driveLeftY_debugger - driveRightX_debugger - driveLeftX_debugger);
        left_back.setPower(driveLeftY_debugger - driveRightX_debugger + driveLeftX_debugger);
        right_front.setPower(driveLeftY_debugger + driveRightX_debugger + driveLeftX_debugger);
        right_back.setPower(driveLeftY_debugger + driveRightX_debugger - driveLeftX_debugger);
    }
}
