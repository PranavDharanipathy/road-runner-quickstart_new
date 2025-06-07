package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.UnboundedMotor.ScheduleAtFixedRate;
import org.firstinspires.ftc.teamcode.UnboundedMotor.ScheduleWithFixedDelay;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class UnboundedMotorExample extends OpMode {

    private UnboundedMotor motor;

    @Override
    public void init() {
        motor = new UnboundedMotor(hardwareMap, "motor");
        motor.setMaxCurrent(5500);
        motor.setMode(UnboundedMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void start() {
        motor.start(new ScheduleWithFixedDelay(20));
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        motor.kill();
    }
}
