package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "DRIVE", group = "Areas") //The one being used
public class DRIVE extends OpMode {

    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;
    private DcMotor left_arm_motor;
    private DcMotor right_arm_motor;
    private Servo claw_servo;
    private Servo claw_pivot_servo;

    private Servo left_arm;
    private Servo right_arm;

    private DcMotor intake;
    private Servo left_intake_chamber_servo;
    private Servo right_intake_chamber_servo;
    private Servo left_extendo_servo;
    private Servo right_extendo_servo;

    private boolean cur_extendo = false;
    private boolean prev_extendo = false;
    private boolean toggle_extendo = false;
    private boolean shootThroughBotToggle = false;

    private double driveLeftX_debugger;
    private double driveLeftY_debugger;
    private double driveRightX_debugger;
    private double joyStickMargin = 0.004;
    private final static int SLIDES_UP_POSITION = 740;
    private final static int SLIDES_DOWN_POSITION = -38;
    private static int SLIDES_SCORE_SPEC_POSITION = 280;
    private static int prev_slidesScoreSpecPosition;
    private static int SLIDES_PICK_SPEC_POSITION = 200;
    private final static int SLIDES_MINIMUM_THRESHOLD = 10;
    private ElapsedTime scoreSpecimenTimer = new ElapsedTime();
    private ElapsedTime pickSpecimenTimer = new ElapsedTime();
    private ElapsedTime pickSpecimenAfterScoringTimer = new ElapsedTime();
    private ElapsedTime pickSampleTimer = new ElapsedTime();
    private ElapsedTime scoreSampleTimer = new ElapsedTime();
    private ElapsedTime transferToClawTimer = new ElapsedTime();
    private ElapsedTime returnToPickSamplePoseMoveArmOutOfExtendoPath = new ElapsedTime();
    private int scoreSpecimenStages = 0;
    private int pickSpecimenStages = 0;
    private int pickSpecimenAfterScoringStages = 0;
    private int scoreSampleStages = 0;
    private int pickSampleStages = 0;
    private int transferToClawStages = 0;
    private int returnToPickSamplePoseStage = 0;
    private boolean allowIntaking = true;
    private boolean allowExtendo = true;
    private boolean prev_gamepad1lb = false;
    private boolean cur_gamepad1lb = false;
    private boolean prev_gamepad1lt = false;
    private boolean cur_gamepad1lt = false;
    private boolean prev_gamepad1rb = false;
    private boolean cur_gamepad1rb = false;
    private boolean prev_gamepad1rt = false;
    private boolean cur_gamepad1rt = false;
    private boolean prev_gamepad2lb = false;
    private boolean cur_gamepad2lb = false;
    private boolean prev_gamepad2rb = false;
    private boolean cur_gamepad2rb = false;
    private boolean prev_gamepad2a = false;
    private boolean cur_gamepad2a = false;
    private boolean prev_gamepad1dpad_up = false;
    private boolean cur_gamepad1dpad_up = false;
    private boolean prev_gamepad1dpad_down = false;
    private boolean cur_gamepad1dpad_down = false;
    private boolean prev_gamepad2x = false;
    private boolean cur_gamepad2x = false;
    private OurColorSensor ourColorSensor = new OurColorSensor();
    private OurColorSensor.DetectedColor wantedColor = OurColorSensor.DetectedColor.OTHER;
    private OurColorSensor.DetectedColor detectedColor;
    private double outtakeSpeed = -0.6;
    private ElapsedTime automaticOuttakeTimer = new ElapsedTime();
    private boolean automaticOuttakeStateHolder = false;
    private double extendoPos = 1; //in = 1 | out = 0
    private boolean CanIntake = true;
    private boolean AllowNeutral = true;
    private boolean clawTransfer = true;
    private ElapsedTime setIntakeModeRumbleTimer = new ElapsedTime();
    private int setIntakeModeRumbleDuration = 150;

    private static double CLAW_OPEN = 0.175;
    private static double CLAW_CLOSED = 0.35;
    private static double TRANSFER_PIVOT = 0.568;
    private static double ARM_AFTER_TRANSFER = 0.11;

    private final static double TRANSFER_POSITION = 0.071;

    @Override
    public void init() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        left_arm_motor = hardwareMap.get(DcMotor.class, "left_arm_motor");
        right_arm_motor = hardwareMap.get(DcMotor.class, "right_arm_motor");
        right_arm_motor.setDirection(DcMotor.Direction.REVERSE);
        left_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_arm = hardwareMap.get(Servo.class, "left_arm");
        right_arm = hardwareMap.get(Servo.class, "right_arm");
        left_arm.setDirection(Servo.Direction.FORWARD);
        right_arm.setDirection(Servo.Direction.FORWARD);

        claw_servo = hardwareMap.get(Servo.class, "claw_servo");
        claw_servo.setDirection(Servo.Direction.FORWARD);
        claw_pivot_servo = hardwareMap.get(Servo.class, "claw_pivot_servo");

        intake = hardwareMap.get(DcMotor.class, "intake");

        left_intake_chamber_servo = hardwareMap.get(Servo.class, "left_intake_servo");
        right_intake_chamber_servo = hardwareMap.get(Servo.class, "right_intake_servo");

        left_extendo_servo = hardwareMap.get(Servo.class, "left_extendo_servo");
        right_extendo_servo = hardwareMap.get(Servo.class, "right_extendo_servo");
        left_extendo_servo.setDirection(Servo.Direction.FORWARD);
        right_extendo_servo.setDirection(Servo.Direction.FORWARD);

        left_intake_chamber_servo.setDirection(REVERSE);
        right_intake_chamber_servo.setDirection(REVERSE);

        right_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_intake_chamber_servo.setDirection(Servo.Direction.REVERSE);
        right_intake_chamber_servo.setDirection(Servo.Direction.REVERSE);

        ourColorSensor.initialize(hardwareMap);
    }

    @Override
    public void start() {

        PostAutonomousRobotReset robot = new PostAutonomousRobotReset(
                claw_servo, claw_pivot_servo,
                left_arm, right_arm,
                left_extendo_servo, right_extendo_servo,
                left_intake_chamber_servo, right_intake_chamber_servo,
                left_arm_motor, right_arm_motor
        );

        robot.runFailSafe();

//        left_arm.setPosition(TRANSFER_POSITION);
//        right_arm.setPosition(TRANSFER_POSITION);
//        claw_servo.setPosition(0.22);
//        claw_pivot_servo.setPosition(0.535);
//        slidesToPose(SLIDES_DOWN_POSITION);
//        extendoRetract();

        scoreSpecimenTimer.reset();
        pickSpecimenTimer.reset();
        pickSpecimenAfterScoringTimer.reset();
        scoreSampleTimer.reset();
        pickSampleTimer.reset();
        setIntakeModeRumbleTimer.reset();
        returnToPickSamplePoseMoveArmOutOfExtendoPath.reset();
        automaticOuttakeTimer.reset();
    }

    public void choice() {
        if (ourColorSensor.getColor() != wantedColor) {
            if (ourColorSensor.getColor() == OurColorSensor.DetectedColor.OTHER) {
                CanIntake = true;
            }
            else if (AllowNeutral == true && ourColorSensor.getColor() == OurColorSensor.DetectedColor.YELLOW) {
                CanIntake = true;
            }
            else{
                CanIntake = false;
            }
        }
        else if (ourColorSensor.getColor() == wantedColor) {
            CanIntake = true;
        }
        else {
            CanIntake = true;
        }
    }

    public void modes() {
        //set mode
        if (gamepad1.x) {//alliance color
            wantedColor = OurColorSensor.DetectedColor.RED;
            AllowNeutral = false;
        }
        else if(gamepad1.b) {//neutral
            wantedColor = OurColorSensor.DetectedColor.YELLOW;
            AllowNeutral = false;
        }
        else if(gamepad1.a) {//both
            wantedColor = OurColorSensor.DetectedColor.RED;
            AllowNeutral = true;
        }

        if (setIntakeModeRumbleTimer.milliseconds() >= setIntakeModeRumbleDuration && wantedColor == OurColorSensor.DetectedColor.OTHER) {
            gamepad1.rumble(setIntakeModeRumbleDuration);
            gamepad2.rumble(setIntakeModeRumbleDuration);
            setIntakeModeRumbleTimer.reset();
        }

        prev_gamepad2a = cur_gamepad2a;
        cur_gamepad2a = gamepad2.a;
        choice();
        if (allowIntaking & wantedColor != OurColorSensor.DetectedColor.OTHER) {
            if (clawTransfer && (ourColorSensor.getColor() == wantedColor || ourColorSensor.getColor() == OurColorSensor.DetectedColor.YELLOW && AllowNeutral)) {
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
                clawTransfer = false;
            }
            if (cur_gamepad2a && !prev_gamepad2a) {
                toggle_extendo = false;
                transferToClawPose();
            }
            if (gamepad2.right_trigger > 0.1 && CanIntake) {
                intake.setPower(1); //intake
            } else if (!CanIntake) {
                automaticOuttakeStateHolder = true; /*outtake*/ /** [[actual outtaking is done in withinLoopProcessing()]] **/
            } else if (gamepad2.left_trigger > 0.1) {
                intake.setPower(-1); //outtake
            } else {
                intake.setPower(0);
            }
        }


        detectedColor = ourColorSensor.detectColor();
        if (detectedColor == OurColorSensor.DetectedColor.BLUE) {
            telemetry.addData("Action", "Blue Sample");
        } else if (detectedColor == OurColorSensor.DetectedColor.RED) {
            telemetry.addData("Action", "Red Sample");
        } else if (detectedColor == OurColorSensor.DetectedColor.YELLOW) {
            telemetry.addData("Action", "Neutral Sample");
        } else {
            telemetry.addData("Action", "No Action");
        }

        telemetry.addData("Mode", wantedColor);
        telemetry.addData("Both", AllowNeutral);

        telemetry.addData("Detected Color", detectedColor);
        telemetry.addData("Power", intake.getPower());
        telemetry.addData("Can Intake", CanIntake);
    }

    public void drivetrain() {

        if (Math.abs(gamepad1.left_stick_x) >= joyStickMargin) {
            driveLeftX_debugger = gamepad1.left_stick_x;
        } else {
            driveLeftX_debugger = 0;
        }

        if (Math.abs(gamepad1.left_stick_y) >= joyStickMargin) {
            driveLeftY_debugger = gamepad1.left_stick_y;
        } else {
            driveLeftY_debugger = 0;
        }

        if (Math.abs(gamepad1.right_stick_x) >= joyStickMargin) {
            driveRightX_debugger = gamepad1.right_stick_x;
        } else {
            driveRightX_debugger = 0;
        }

        left_front.setPower(driveLeftY_debugger - driveRightX_debugger - driveLeftX_debugger);
        left_back.setPower(driveLeftY_debugger - driveRightX_debugger + driveLeftX_debugger);
        right_front.setPower(driveLeftY_debugger + driveRightX_debugger + driveLeftX_debugger);
        right_back.setPower(driveLeftY_debugger + driveRightX_debugger - driveLeftX_debugger);
    }

    public void extendo() {

        prev_extendo = cur_extendo;
        cur_extendo = gamepad2.dpad_up;

        prev_gamepad2x = cur_gamepad2x;
        cur_gamepad2x = gamepad2.x;

        if (allowExtendo) {
            if (gamepad2.dpad_down) {
                toggle_extendo = false;
                extendoRetract();
            } else if (cur_extendo && !prev_extendo) {
                if (toggle_extendo) {
                    /** OUT **/
                    extendoExtend();
                    outtakeSpeed = -0.475;
                } else {
                    /** IN **/
                    pickFromSubmersible();
                    outtakeSpeed = -0.8;
                }
                toggle_extendo = !toggle_extendo;
            }
            if (cur_gamepad2x && !prev_gamepad2x) {
                if (!shootThroughBotToggle) {
                    shootThroughBot();
                    outtakeSpeed = -0.7;
                }
                else {
                    left_intake_chamber_servo.setPosition(0.075);
                    right_intake_chamber_servo.setPosition(0.075);
                }
                shootThroughBotToggle = !shootThroughBotToggle;
            }
        }
    }

    public void debugSlideInaccuracies() {

        prev_gamepad1dpad_up = cur_gamepad1dpad_up;
        cur_gamepad1dpad_up = gamepad1.dpad_up;

        prev_gamepad1dpad_down = cur_gamepad1dpad_down;
        cur_gamepad1dpad_down = gamepad1.dpad_down;

        if (cur_gamepad1dpad_up && !prev_gamepad1dpad_up) {
            if (left_arm_motor.getTargetPosition() == SLIDES_SCORE_SPEC_POSITION) {
                SLIDES_SCORE_SPEC_POSITION+=25;
            }
            else if (left_arm_motor.getTargetPosition() == SLIDES_PICK_SPEC_POSITION) {
                SLIDES_PICK_SPEC_POSITION+=25;
            }
        }
        else if (cur_gamepad1dpad_down && !prev_gamepad1dpad_down) {
            if (left_arm_motor.getTargetPosition() == SLIDES_SCORE_SPEC_POSITION) {
                SLIDES_SCORE_SPEC_POSITION-=25;
            }
            else if (left_arm_motor.getTargetPosition() == SLIDES_PICK_SPEC_POSITION) {
                SLIDES_PICK_SPEC_POSITION-=25;
            }
        }
    }

    public void specimenUsingClaw() {

        //on-click for toggle
        prev_gamepad1lb = cur_gamepad1lb;
        cur_gamepad1lb = gamepad1.left_bumper;

        prev_gamepad1lt = cur_gamepad1lt;
        cur_gamepad1lt = gamepad1.left_trigger > 0.078;

        prev_gamepad1rb = cur_gamepad1rb;
        cur_gamepad1rb = gamepad1.right_bumper;

        prev_gamepad1rt = cur_gamepad1rt;
        cur_gamepad1rt = gamepad1.right_trigger > 0.078;

        if (cur_gamepad1lb && !prev_gamepad1lb) {
            pickSpecimenPose();
        }
        else if (cur_gamepad1lt && !prev_gamepad1lt) {
            scoreSpecimenPose();
        }
        else if (cur_gamepad1rb && !prev_gamepad1rb) {
            returnToPickSamplePose();
        }
        else if (cur_gamepad1rt && !prev_gamepad1rt) {
            pickSpecimenPoseAfterScoring();
        }
        debugSlideInaccuracies();
    }

    public void sample() {

        prev_gamepad2lb = cur_gamepad2lb;
        cur_gamepad2lb = gamepad2.left_bumper;

        prev_gamepad2rb = cur_gamepad2rb;
        cur_gamepad2rb = gamepad2.right_bumper;

        if (cur_gamepad2rb && !prev_gamepad2rb) {
            scoreSamplePose();
        }
        else if (cur_gamepad2lb && !prev_gamepad2lb) {
            pickSamplePose();
        }
    }

    @Override
    public void loop() {

        modes();
        extendo();
        specimenUsingClaw();
        sample();
        withinLoopProcessing();
        drivetrain();

        telemetry.addData("Mode", wantedColor);
        telemetry.addData("Both", AllowNeutral);
        telemetry.addData("Detected Color", detectedColor);
        telemetry.addData("Hue Detected", ourColorSensor.getHue());
        telemetry.addData("Can Intake", CanIntake);
        telemetry.addLine(" ");
        telemetry.addData("Power", intake.getPower());
        telemetry.addData("pickSampleStages", pickSampleStages);
        telemetry.addData("scoreSampleStages", scoreSampleStages);
        telemetry.addData("pickSpecimenStages", pickSpecimenStages);
        telemetry.addData("scoreSpecimenStages", scoreSpecimenStages);
        telemetry.addData("pickSpecimenAfterScoringStages", pickSpecimenAfterScoringStages);
        telemetry.addLine(" ");
        telemetry.addData("left_slides", left_arm_motor.getCurrentPosition());
        telemetry.addData("right_slides", right_arm_motor.getCurrentPosition());
        telemetry.addData("left_slides power", left_arm_motor.getPower());
        telemetry.addData("right_slides power", right_arm_motor.getPower());
        telemetry.addLine(" ");
        telemetry.addData("left_arm servo", left_arm.getPosition());
        telemetry.addData("right_arm servo", right_arm.getPosition());

        telemetry.update();
    }

    public void pickSamplePose() {
        pickSampleTimer.reset();
        scoreSampleStages = 0;
        pickSpecimenAfterScoringStages = 0;
        scoreSpecimenStages = 0;
        pickSpecimenStages = 0;
        returnToPickSamplePoseStage = 0;
        pickSampleStages = 1;
    }

    public void returnToPickSamplePose() {
        scoreSampleStages = 0;
        pickSpecimenAfterScoringStages = 0;
        scoreSpecimenStages = 0;
        pickSpecimenStages = 0;
        pickSampleStages = 0;
        returnToPickSamplePoseStage = 1;
        left_arm.setPosition(TRANSFER_POSITION);
        right_arm.setPosition(TRANSFER_POSITION);
        claw_servo.setPosition(CLAW_OPEN);
        claw_pivot_servo.setPosition(TRANSFER_PIVOT);
        slidesToPose(SLIDES_DOWN_POSITION);
    }

    public void transferToClawPose() {
        transferToClawTimer.reset();
        scoreSampleStages = 0;
        pickSpecimenAfterScoringStages = 0;
        scoreSpecimenStages = 0;
        pickSpecimenStages = 0;
        pickSampleStages = 0;
        returnToPickSamplePoseStage = 0;
        transferToClawStages = 1;
    }

    public void pickSpecimenPose() {
        pickSpecimenTimer.reset();
        pickSampleStages = 0;
        scoreSampleStages = 0;
        pickSpecimenAfterScoringStages = 0;
        scoreSpecimenStages = 0;
        transferToClawStages = 0;
        returnToPickSamplePoseStage = 0;
        pickSpecimenStages = 1;
    }

    public void pickSpecimenPoseAfterScoring() {
        pickSpecimenAfterScoringTimer.reset();
        pickSampleStages = 0;
        scoreSampleStages = 0;
        scoreSpecimenStages = 0;
        pickSpecimenStages = 0;
        transferToClawStages = 0;
        returnToPickSamplePoseStage = 0;
        pickSpecimenAfterScoringStages = 1;
    }

    public void scoreSpecimenPose() {
        scoreSpecimenTimer.reset();
        pickSampleStages = 0;
        scoreSampleStages = 0;
        pickSpecimenAfterScoringStages = 0;
        pickSpecimenStages = 0;
        transferToClawStages = 0;
        returnToPickSamplePoseStage = 0;
        scoreSpecimenStages = 1;
    }

    public void scoreSamplePose() {
        scoreSampleTimer.reset();
        pickSampleStages = 0;
        pickSpecimenAfterScoringStages = 0;
        pickSpecimenStages = 0;
        scoreSpecimenStages = 0;
        returnToPickSamplePoseStage = 0;
        scoreSampleStages = 1;
    }

    public void extendoExtend() {
        clawTransfer = true;
        left_extendo_servo.setPosition(0.3);
        right_extendo_servo.setPosition(0.3);
        left_intake_chamber_servo.setPosition(0.81325);
        right_intake_chamber_servo.setPosition(0.81325);
        extendoPos = 0;

        shootThroughBotToggle = false;
    }

    public void pickFromSubmersible() {
        left_extendo_servo.setPosition(0.3);
        right_extendo_servo.setPosition(0.3);
        left_intake_chamber_servo.setPosition(0.425);
        right_intake_chamber_servo.setPosition(0.425);
        extendoPos = 0;

        shootThroughBotToggle = false;
    }

    public void extendoRetract() {
        left_extendo_servo.setPosition(0);
        right_extendo_servo.setPosition(0);
        left_intake_chamber_servo.setPosition(0.075);
        right_intake_chamber_servo.setPosition(0.075);
        extendoPos = 1;

        shootThroughBotToggle = false;
    }

    public void shootThroughBot() {
        left_intake_chamber_servo.setPosition(0.25);
        right_intake_chamber_servo.setPosition(0.25);
    }

    public void slidesToPose(int targetPosition) {
        left_arm_motor.setTargetPosition(targetPosition);
        right_arm_motor.setTargetPosition(targetPosition);
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);
        left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void withinLoopProcessing() {
        // release slides power
        if (left_arm_motor.getCurrentPosition() <= SLIDES_MINIMUM_THRESHOLD && left_arm_motor.getTargetPosition() == SLIDES_DOWN_POSITION) {
            left_arm_motor.setPower(0);
            right_arm_motor.setPower(0);
            left_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (left_arm_motor.getTargetPosition() != SLIDES_PICK_SPEC_POSITION && left_arm_motor.getTargetPosition() != SLIDES_DOWN_POSITION && left_arm_motor.getTargetPosition() != SLIDES_UP_POSITION) {
            if (SLIDES_SCORE_SPEC_POSITION != prev_slidesScoreSpecPosition) { //to prevent running as much computationally intense actions to make battery life last longer.
                slidesToPose(SLIDES_SCORE_SPEC_POSITION);
                prev_slidesScoreSpecPosition = SLIDES_SCORE_SPEC_POSITION;
            }
        }
        if (extendoPos == 1 && returnToPickSamplePoseStage != 2 && (returnToPickSamplePoseStage == 1 || pickSampleStages > 1 /*stages 2 & 3*/)) {
            if (returnToPickSamplePoseMoveArmOutOfExtendoPath.milliseconds() <= 250) {
                left_arm.setPosition(0.15);
                right_arm.setPosition(0.15);
            }
            else {
                left_arm.setPosition(0.1168);
                right_arm.setPosition(0.1168);
                returnToPickSamplePoseStage = 2;
            }
        }
        else returnToPickSamplePoseMoveArmOutOfExtendoPath.reset();

        if (automaticOuttakeStateHolder) {
            intake.setPower(outtakeSpeed);
            if (automaticOuttakeTimer.milliseconds() >= 1200) {
                automaticOuttakeStateHolder = false;
                intake.setPower(0);
            }
        }
        else automaticOuttakeTimer.reset();

        //score spec
        if (scoreSpecimenStages == 1 && scoreSpecimenTimer.milliseconds() >= 1) {
            claw_pivot_servo.setPosition(0.83);
            claw_servo.setPosition(CLAW_CLOSED);
            scoreSpecimenStages++;
        }
        if (scoreSpecimenStages == 2 && scoreSpecimenTimer.milliseconds() >= 50) {
            slidesToPose(SLIDES_SCORE_SPEC_POSITION);
            scoreSpecimenStages++;
        }
        if (scoreSpecimenStages == 3 && scoreSpecimenTimer.milliseconds() >= 250) {
            claw_pivot_servo.setPosition(1);
            scoreSpecimenStages++;
        }
        if (scoreSpecimenStages == 4 && scoreSpecimenTimer.milliseconds() >= 600) {
            left_arm.setPosition(0.65);
            right_arm.setPosition(0.65);
            scoreSpecimenStages++;
        }
        if (scoreSpecimenStages == 5 && scoreSpecimenTimer.milliseconds() >= 700) {
            left_arm.setPosition(0.1);
            right_arm.setPosition(0.1);
            scoreSpecimenStages++;
        }
        if (scoreSpecimenStages == 6 && scoreSpecimenTimer.milliseconds() >= 715) {
            claw_pivot_servo.setPosition(0.775);
        }
        /*---*/
        //pick spec
        if (pickSpecimenStages == 1 && pickSpecimenTimer.milliseconds() >= 1) {
            claw_servo.setPosition(CLAW_OPEN);
            pickSpecimenStages++;
        }
        if (pickSpecimenStages == 2 && pickSpecimenTimer.milliseconds() >= 50) {
            slidesToPose(SLIDES_PICK_SPEC_POSITION);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
            pickSpecimenStages++;
        }
        if (pickSpecimenStages == 3 && pickSpecimenTimer.milliseconds() >= 250) {
            claw_pivot_servo.setPosition(0.83);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
        }
        /*---*/
        //pick spec after scoring
        if (pickSpecimenAfterScoringStages == 1 && pickSpecimenAfterScoringTimer.milliseconds() >= 1) {
            claw_servo.setPosition(CLAW_OPEN);
            pickSpecimenAfterScoringStages++;
        }
        if (pickSpecimenAfterScoringStages == 2 && pickSpecimenAfterScoringTimer.milliseconds() >= 50) {
            claw_pivot_servo.setPosition(0.83);
            pickSpecimenAfterScoringStages++;
        }
        if (pickSpecimenAfterScoringStages == 3 && pickSpecimenAfterScoringTimer.milliseconds() >= 250) {
            slidesToPose(SLIDES_PICK_SPEC_POSITION);
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
            pickSpecimenAfterScoringStages++;
        }
        if (pickSpecimenAfterScoringStages == 4 && pickSpecimenAfterScoringTimer.milliseconds() >= 550) {
            left_arm.setPosition(0.82);
            right_arm.setPosition(0.82);
        }

        /*---*/
        //score sample
        if (scoreSampleStages == 1 && scoreSampleTimer.milliseconds() >= 1) {
            claw_pivot_servo.setPosition(0.92);
            claw_servo.setPosition(CLAW_CLOSED);
            scoreSampleStages++;
        }
        if (scoreSampleStages == 2 && scoreSampleTimer.milliseconds() >= 300) {
            left_arm.setPosition(0.3);
            right_arm.setPosition(0.3);
            scoreSampleStages++;
        }
        if (scoreSampleStages == 3 && scoreSampleTimer.milliseconds() >= 625) {
            slidesToPose(SLIDES_UP_POSITION);
            scoreSampleStages++;
        }
        if (scoreSampleStages == 4 && scoreSampleTimer.milliseconds() >= 1075) {
            left_arm.setPosition(0.485);
            right_arm.setPosition(0.485);
            scoreSampleStages++;
        }
        if (scoreSampleStages == 5 && scoreSampleTimer.milliseconds() >= 1275) {
            claw_pivot_servo.setPosition(0.875);
        }
        /*---*/
        //pick sample
        if (pickSampleStages == 1 && pickSampleTimer.milliseconds() >= 1) {
            claw_pivot_servo.setPosition(0.865); //goes a bit down before releasing to prevent the sample from getting flung out
            claw_servo.setPosition(CLAW_OPEN);
            pickSampleStages++;
        }
        if (pickSampleStages == 2 && pickSampleTimer.milliseconds() >= 300) {
            left_arm.setPosition(TRANSFER_POSITION);
            right_arm.setPosition(TRANSFER_POSITION);
            pickSampleStages++;
        }
        if (pickSampleStages == 3 && pickSampleTimer.milliseconds() >= 630) {
            claw_pivot_servo.setPosition(TRANSFER_PIVOT);
            slidesToPose(SLIDES_DOWN_POSITION);
            if (pickSampleTimer.milliseconds() >= 1000) {
                pickSampleStages = 4; // increased at final step to prevent the slides from trying to go down again and again
            }
        }
        /*---*/
        //transfer sample to claw
        if (transferToClawStages == 1) {
            allowIntaking = false;
            allowExtendo = false;
            claw_pivot_servo.setPosition(TRANSFER_PIVOT);
            left_arm.setPosition(TRANSFER_POSITION);
            right_arm.setPosition(TRANSFER_POSITION);
            transferToClawStages++;
        }
        if (transferToClawStages == 2 && transferToClawTimer.milliseconds() >= 400) {
            intake.setPower(-0.75);
            transferToClawStages++;
        }
        if (transferToClawStages == 3 && transferToClawTimer.milliseconds() >= (700)) {
            claw_servo.setPosition(CLAW_CLOSED);
            transferToClawStages++;
        }
        if (transferToClawStages == 4 && transferToClawTimer.milliseconds() >= (735)) {
            transferToClawStages = 0;
            left_arm.setPosition(ARM_AFTER_TRANSFER);
            right_arm.setPosition(ARM_AFTER_TRANSFER);
        }
        if (transferToClawStages == 0) {
            allowIntaking = true;
            allowExtendo = true;
        }
        /*---*/
    }

}