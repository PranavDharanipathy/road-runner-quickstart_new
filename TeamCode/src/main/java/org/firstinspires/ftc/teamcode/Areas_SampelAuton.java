package org.firstinspires.ftc.teamcode;

//RR-specific imports

// Non-RR imports
import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import kotlin.jvm.internal.TypeParameterReference;

@Autonomous(name = "Areas_Feb_22_SampleAuton", group = "Areas")
public class Areas_SampelAuton extends LinearOpMode{
    private Servo claw_servo;
    private Servo claw_pivot_servo;
    private Servo left_arm;
    private Servo right_arm;
    private Servo left_intake_chamber_servo;
    private Servo right_intake_chamber_servo;
    private Servo left_extendo_servo;
    private Servo right_extendo_servo;
    private DcMotor left_arm_motor;
    private DcMotor right_arm_motor;
    private DcMotor intake;
    private final static int SLIDES_UP_POSITION = 695;
    private final static int SLIDES_DOWN_POSITION = -12;
    private final static int SLIDES_MINIMUM_THRESHOLD = 35;
    private ElapsedTime clawPivotTimer = new ElapsedTime();
    private ElapsedTime clawTimer = new ElapsedTime();
    private ElapsedTime armTimer = new ElapsedTime();
    private ElapsedTime armMotorTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime chamberTimer = new ElapsedTime();
    private ElapsedTime extendoTimer = new ElapsedTime();



    public class Arm {
        private Servo left_arm;
        private Servo right_arm;

        public Arm(HardwareMap hardwareMap) {
            left_arm = hardwareMap.get(Servo.class, "left_arm");
            right_arm = hardwareMap.get(Servo.class, "right_arm");
            left_arm.setDirection(FORWARD);
            right_arm.setDirection(FORWARD);
        }

        public class MoveArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                left_arm.setPosition(0);
                right_arm.setPosition(0);
                return false;
            }
        }
        public Action moveArmDown() {
            return new Arm.MoveArmDown();
        }
        public class InitARM implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                left_arm.setPosition(0.2);
                right_arm.setPosition(0.2);
                return false;
            }
        }
        public Action initArm() {
            return new Arm.InitARM();
        }

        public class SecondScoreSampleStage implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
//                left_arm.setDirection(REVERSE);
//                right_arm.setDirection(REVERSE);
                left_arm.setPosition(0.5);
                right_arm.setPosition(0.5);
                return false;
            }
        }
        public Action secondScoreSampleStage() {
            return new Arm.SecondScoreSampleStage();
        }
        public class FourthScoreSampleStage implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                left_arm.setPosition(0.485);
                right_arm.setPosition(0.485);
                if (armTimer.milliseconds() < 500){
                    return true;
                }
                armTimer.reset();
                return false;
            }
        }
        public Action fourthScoreSampleStage() {
            return new Arm.FourthScoreSampleStage();
        }
    }

    public class Chamber_Servos{
        private Servo left_intake_chamber_servo;
        private Servo right_intake_chamber_servo ;

        public Chamber_Servos(HardwareMap hardwareMap) {
            left_intake_chamber_servo = hardwareMap.get(Servo.class, "left_intake_servo");
            right_intake_chamber_servo = hardwareMap.get(Servo.class, "right_intake_servo");
            left_intake_chamber_servo.setDirection(REVERSE);
            right_intake_chamber_servo.setDirection(REVERSE);
        }
        public class ChamberExtendoRetract implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                left_intake_chamber_servo.setPosition(0.075);
                right_intake_chamber_servo.setPosition(0.075);
                return false;
            }
        }
        public Action chamberExtendoRetract() {
            return new Chamber_Servos.ChamberExtendoRetract();
        }
        public class ChamberExtendoExtend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                left_intake_chamber_servo.setPosition(0.65);
                right_intake_chamber_servo.setPosition(0.65);
                return false;
            }
        }
        public Action chamberExtendoExtend() {
            return new Chamber_Servos.ChamberExtendoExtend();
        }
    }
    public class Extendo_Servos{
        private Servo left_extendo_servo;
        private Servo right_extendo_servo;

        public Extendo_Servos(HardwareMap hardwareMap) {
            left_extendo_servo = hardwareMap.get(Servo.class, "left_extendo_servo");
            right_extendo_servo = hardwareMap.get(Servo.class, "right_extendo_servo");
            left_extendo_servo.setDirection(FORWARD);
            right_extendo_servo.setDirection(FORWARD);
        }
        public class ExtendoRetract implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                left_extendo_servo.setPosition(0.0);
                right_extendo_servo.setPosition(0.0);
                return false;
            }
        }
        public Action extendoRetract() {
            return new Extendo_Servos.ExtendoRetract();
        }
        public class ExtendoExtend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                left_extendo_servo.setPosition(0.3);
                right_extendo_servo.setPosition(0.3);
                if (extendoTimer.milliseconds() < 250){
                    return true;
                }
                extendoTimer.reset();
                return false;
            }
        }
        public Action extendoExtend() {
            return new Extendo_Servos.ExtendoExtend();
        }
    }
    public class Arm_Motors{
        private DcMotor left_arm_motor;
        private DcMotor right_arm_motor;

        public Arm_Motors(HardwareMap hardwareMap) {
            left_arm_motor = hardwareMap.get(DcMotor.class, "left_arm_motor");
            right_arm_motor = hardwareMap.get(DcMotor.class, "right_arm_motor");
            left_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_arm_motor.setDirection(DcMotor.Direction.REVERSE);
        }

        public class ThirdScoreSampleStage implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                left_arm_motor.setTargetPosition(SLIDES_UP_POSITION);
                right_arm_motor.setTargetPosition(SLIDES_UP_POSITION);
                left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left_arm_motor.setPower(1);
                right_arm_motor.setPower(1);
                if (armMotorTimer.milliseconds() < 1500){
                    return true;
                }
                armMotorTimer.reset();
                return false;
            }
        }
        public Action thirdScoreSampleStage() {
            return new Arm_Motors.ThirdScoreSampleStage();
        }
        public class MoveSlidesDown implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                left_arm_motor.setTargetPosition(SLIDES_DOWN_POSITION);
                right_arm_motor.setTargetPosition(SLIDES_DOWN_POSITION);
                left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left_arm_motor.setPower(1);
                right_arm_motor.setPower(1);
                if (armMotorTimer.milliseconds() < 800){
                    return true;
                }
                armMotorTimer.reset();
                return false;
            }
        }
        public Action moveSlidesDown() {
            return new Arm_Motors.MoveSlidesDown();
        }
    }
    public class Claw_Servos{
        private Servo claw_servo;

        public Claw_Servos(HardwareMap hardwareMap) {
            claw_servo = hardwareMap.get(Servo.class, "claw_servo");
            claw_servo.setDirection(FORWARD);
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw_servo.setPosition(0.375);
                return false;
            }
        }
        public Action closeClaw() {
            return new Claw_Servos.CloseClaw();
        }
        public class FirstScoreSampleStage implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw_servo.setPosition(0.5);
                if (clawTimer.milliseconds() < 250){
                    return true;
                }
                clawTimer.reset();
                return false;
            }
        }
        public Action firstScoreSampleStage() {
            return new Claw_Servos.FirstScoreSampleStage();
        }
        public class SixthScoreSampleStage implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw_servo.setPosition(0.0);
                if (clawTimer.milliseconds() < 250){
                    return true;
                }
                clawTimer.reset();
                return false;
            }
        }
        public Action sixthScoreSampleStage() {
            return new Claw_Servos.SixthScoreSampleStage();
        }

    }
    public class Claw_Pivot_Servos{
        private Servo claw_pivot_servo;

        public Claw_Pivot_Servos(HardwareMap hardwareMap) {
            claw_pivot_servo = hardwareMap.get(Servo.class, "claw_pivot_servo");
            claw_pivot_servo.setDirection(FORWARD);
        }
        public class SetClawStraight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw_pivot_servo.setPosition(0.5);
                return false;
            }
        }
        public Action setClawStraight() {
            return new Claw_Pivot_Servos.SetClawStraight();
        }
        public class HalfFirstScoreSampleStage implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw_pivot_servo.setPosition(0.875);
                if (clawPivotTimer.milliseconds() < 250){
                    return true;
                }
                clawPivotTimer.reset();
                return false;
            }
        }
        public Action halfFirstScoreSampleStage() {
            return new Claw_Pivot_Servos.HalfFirstScoreSampleStage();
        }
        public class FifthScoreSampleStage implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw_pivot_servo.setPosition(0.875);
                if (clawPivotTimer.milliseconds() < 250){
                    return true;
                }
                clawPivotTimer.reset();
                return false;
            }
        }
        public Action fifthScoreSampleStage() {
            return new Claw_Pivot_Servos.FifthScoreSampleStage();
        }

    }
    public class Intake{
        private DcMotor intake;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "intake");
        }
        public class IntakeRun implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(1);
                if (intakeTimer.milliseconds() < 1000){
                    return true;
                }
                intake.setPower(0);
                intakeTimer.reset();
                return false;
            }
        }
        public Action intakeRun() {
            return new Intake.IntakeRun();
        }
    }

    public void intake(){
        left_arm_motor.setTargetPosition(SLIDES_DOWN_POSITION);
        right_arm_motor.setTargetPosition(SLIDES_DOWN_POSITION);
        left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        left_arm.setPosition(0.11);
        right_arm.setPosition(0.11);
        left_extendo_servo.setPosition(0.1325);
        right_extendo_servo.setPosition(0.1325);
        sleep(500);
        left_intake_chamber_servo.setPosition(0.81325);
        right_intake_chamber_servo.setPosition(0.81325);
        sleep(500);
        intake.setPower(1);
        left_extendo_servo.setPosition(0.265);
        right_extendo_servo.setPosition(0.265);
        sleep(1200);
        intake.setPower(0);
        claw_pivot_servo.setPosition(0.5);
        left_intake_chamber_servo.setPosition(0.075);
        right_intake_chamber_servo.setPosition(0.075);
        sleep(400);
        left_extendo_servo.setPosition(0.0);
        right_extendo_servo.setPosition(0.0);
        sleep(850);
    }

    public void outtake(){
        intake.setPower(-0.85);
        sleep(250);
        intake.setPower(0);
        claw_servo.setPosition(0.418);
        left_arm.setPosition(0.2);
        right_arm.setPosition(0.2);
        sleep(800);
        left_arm_motor.setTargetPosition(SLIDES_UP_POSITION);
        right_arm_motor.setTargetPosition(SLIDES_UP_POSITION);
        left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        sleep(800);
        left_arm.setPosition(0.485);
        right_arm.setPosition(0.485);
        claw_pivot_servo.setPosition(0.875);
        sleep(650);
    }



    Thread thread2 = new Thread(() -> {
        try{
            claw_servo.setPosition(0.375);
            claw_pivot_servo.setPosition(0.875);
            left_arm.setPosition(0.5);
            right_arm.setPosition(0.5);
            left_arm_motor.setTargetPosition(SLIDES_UP_POSITION);
            right_arm_motor.setTargetPosition(SLIDES_UP_POSITION);
            left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_arm_motor.setPower(1);
            right_arm_motor.setPower(1);
            left_arm.setPosition(0.485);
            right_arm.setPosition(0.485);
            claw_pivot_servo.setPosition(0.875);
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
    });
    Thread thread3 = new Thread(() -> {
        try{
            intake();
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
    });
    Thread thread4 = new Thread(() -> {
        try{
            outtake();
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
    });
    Thread thread5 = new Thread(() -> {
        try{
            intake();
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
    });
    Thread thread6 = new Thread(() -> {
        try {

            outtake();
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
    });
    Thread thread7 = new Thread(() -> {
        try{
            intake();
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
    });
    Thread thread8 = new Thread(() -> {
        try {
            outtake();
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
    });

    @Override
    public void runOpMode() throws InterruptedException{
        left_arm = hardwareMap.get(Servo.class, "left_arm");
        right_arm = hardwareMap.get(Servo.class, "right_arm");
        left_intake_chamber_servo = hardwareMap.get(Servo.class, "left_intake_servo");
        right_intake_chamber_servo = hardwareMap.get(Servo.class, "right_intake_servo");
        left_intake_chamber_servo.setDirection(REVERSE);
        right_intake_chamber_servo.setDirection(REVERSE);
        left_extendo_servo = hardwareMap.get(Servo.class, "left_extendo_servo");
        right_extendo_servo = hardwareMap.get(Servo.class, "right_extendo_servo");
        left_extendo_servo.setDirection(FORWARD);
        right_extendo_servo.setDirection(FORWARD);
        left_arm_motor = hardwareMap.get(DcMotor.class, "left_arm_motor");
        right_arm_motor = hardwareMap.get(DcMotor.class, "right_arm_motor");
        left_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_arm_motor.setDirection(DcMotor.Direction.REVERSE);
        claw_servo = hardwareMap.get(Servo.class, "claw_servo");
        claw_servo.setDirection(FORWARD);
        claw_pivot_servo = hardwareMap.get(Servo.class, "claw_pivot_servo");
        claw_pivot_servo.setDirection(FORWARD);
        intake = hardwareMap.get(DcMotor.class, "intake");
        Pose2d initialPose = new Pose2d(-33, -60, Math.toRadians(0));
        Pose2d thirdSamplePose = new Pose2d(-53, -48, Math.toRadians(63));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        Extendo_Servos extendo_servos = new Extendo_Servos(hardwareMap);
        Chamber_Servos chamber_servos = new Chamber_Servos(hardwareMap);
        Arm_Motors arm_motors = new Arm_Motors(hardwareMap);
        Claw_Servos claw_servos = new Claw_Servos(hardwareMap);
        Claw_Pivot_Servos claw_pivot_servos = new Claw_Pivot_Servos(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        TrajectoryActionBuilder firstSample = drive.actionBuilder(initialPose)


                .splineToLinearHeading(new Pose2d(-53, -48, Math.toRadians(64)), Math.toRadians(0),
                        new TranslationalVelConstraint(50),
                        new ProfileAccelConstraint(-35,25))


                ;
        TrajectoryActionBuilder thirdSample = drive.actionBuilder(thirdSamplePose)
                .turnTo(Math.toRadians(87))
                ;

        TrajectoryActionBuilder backToBucketThirdSample = drive.actionBuilder(new Pose2d(-53, -48, Math.toRadians(85)))
                .turnTo(Math.toRadians(63))
                ;

        TrajectoryActionBuilder fourthSample = drive.actionBuilder(thirdSamplePose)
                .turnTo(Math.toRadians(107))

                ;

        TrajectoryActionBuilder backToFourthSample = drive.actionBuilder(new Pose2d(-53, -48, Math.toRadians(107)))
                .turnTo(Math.toRadians(63))
                ;

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-53, -48, Math.toRadians(63)))

                .splineToLinearHeading(new Pose2d(-16, -6, Math.toRadians(180)), Math.toRadians(0),
                        new TranslationalVelConstraint(50),
                        new ProfileAccelConstraint(-35,25))
                ;




        Actions.runBlocking(arm.initArm());
        Actions.runBlocking(claw_servos.closeClaw());
        Actions.runBlocking(claw_pivot_servos.setClawStraight());
        Actions.runBlocking(extendo_servos.extendoRetract());
        Actions.runBlocking(chamber_servos.chamberExtendoRetract());

        waitForStart();

        thread2.start();
        Actions.runBlocking(firstSample.build());
        sleep(850);
        claw_servo.setPosition(0.17);
        try {
            thread2.join();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        thread3.start();
        sleep(2600);
        try {
            thread3.join();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        thread4.start();
        sleep(2600);
        try {
            thread4.join();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        claw_servo.setPosition(0.17);
        sleep(200);
        thread5.start();
        Actions.runBlocking(thirdSample.build());
        sleep(2600);
        try{
            thread5.join();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        thread6.start();
        Actions.runBlocking(backToBucketThirdSample.build());
        sleep(2800);
        try{
            thread6.join();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        claw_servo.setPosition(0.17);
        sleep(200);
        thread7.start();
        Actions.runBlocking(fourthSample.build());
        sleep(2600);
        try{
            thread7.join();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        thread8.start();
        Actions.runBlocking(backToFourthSample.build());
        sleep(2400);
        try{
            thread8.join();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        claw_servo.setPosition(0.17);
        sleep(200);
        left_arm_motor.setTargetPosition(SLIDES_DOWN_POSITION);
        right_arm_motor.setTargetPosition(SLIDES_DOWN_POSITION);
        left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        left_arm.setPosition(0.3);
        right_arm.setPosition(0.3);
        Actions.runBlocking(park.build());
        left_arm.setPosition(0.6);
        right_arm.setPosition(0.6);


        sleep(4000);

    }
}