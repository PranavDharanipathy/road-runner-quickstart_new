package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class PostAutonomousRobotReset {

    Servo claw, pivot, lfArm, rtArm, lfExtendo, rtExtendo, lfChamber, rtChamber;
    DcMotor lfSlides, rtSlides;

    public PostAutonomousRobotReset(Servo claw, Servo pivot, Servo lfArm, Servo rtArm, Servo lfExtendo, Servo rtExtendo, Servo lfChamber, Servo rtChamber, DcMotor lfSlides, DcMotor rtSlides) {
        this.claw = claw;
        this.pivot = pivot;
        this.lfArm = lfArm;
        this.rtArm = rtArm;
        this.lfExtendo = lfExtendo;
        this.rtExtendo = rtExtendo;
        this.lfChamber = lfChamber;
        this.rtChamber = rtChamber;
        this.lfSlides = lfSlides;
        this.rtSlides = rtSlides;
    }

    private void armFix(Servo lA, Servo rA) {
//        lA.setPosition(0.0);
//        rA.setPosition(0.0);
//        sleep(1);
//       for(double i = 0.01; i >= 0.071;) {
//            lA.setPosition(i);
//            rA.setPosition(i);
//            i = i + 0.01;
//        }
        lA.setPosition(0.071);
        rA.setPosition(0.071);
    }

    public void runFailSafe() {

        ///arm moved out of the way
        claw.setPosition(0.2);
        pivot.setPosition(0.568);
        armFix(lfArm, rtArm);

        sleep(300);

        ///entend extendo
        lfExtendo.setPosition(0.34);
        rtExtendo.setPosition(0.34);
        lfChamber.setPosition(0.81325);
        rtChamber.setPosition(0.81325);

        ///slides moved up
        lfSlides.setTargetPosition(1000);
        rtSlides.setTargetPosition(1000);
        lfSlides.setPower(1);
        rtSlides.setPower(1);
        lfSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rtSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(800);

        ///retract extendo
        lfExtendo.setPosition(0);
        rtExtendo.setPosition(0);
        lfChamber.setPosition(0.075);
        rtChamber.setPosition(0.075);

        ///slides moved down
        lfSlides.setTargetPosition(-1000);
        rtSlides.setTargetPosition(-1000);
        lfSlides.setPower(1);
        rtSlides.setPower(1);
        lfSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rtSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(500);
        lfArm.setPosition(0.071);
        rtArm.setPosition(0.071);
        sleep(750);

        ///slides stopped and reset
        lfSlides.setPower(0);
        rtSlides.setPower(0);
        //slides zero set
        lfSlides.setTargetPosition(0);
        rtSlides.setTargetPosition(0);
        lfSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rtSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfSlides.setTargetPosition(0);
        rtSlides.setTargetPosition(0);
        lfSlides.setPower(1);
        rtSlides.setPower(1);
        lfSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rtSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(100);

        //slides reset
        lfSlides.setTargetPosition(0);
        rtSlides.setTargetPosition(0);
        lfSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rtSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
