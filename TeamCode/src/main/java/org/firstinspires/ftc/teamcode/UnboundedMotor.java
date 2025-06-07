package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


@SuppressLint("DiscouragedApi")
@SuppressWarnings({"unused", "cast"})
public class UnboundedMotor {

    private boolean isStarted = false;
    public class MotorAlreadyStartedException extends RejectedExecutionException {

        public MotorAlreadyStartedException(String message) {
            super(message);
        }
    }

    private String deviceName;

    private static Integer SCHEDULE_RATE;
    private static String SCHEDULE_TYPE = "UNKNOWN";
    private static int SCHEDULE_BIAS;
    private ArrayList<Double> timeBetweenTicks = new ArrayList<>();
    private ArrayList<Double> timeWaited = new ArrayList<>();

    private ExecutorService asyncStarter;
    private volatile boolean process = true;
    private Future<?> runner;

    private double MAX_MILLIAMPS;

    private double power = 0;

    private int targetPosition = 0;

    public enum Direction { FORWARD, REVERSE }

    public enum RunMode { RUN_WITHOUT_ENCODER, RUN_USING_ENCODER, STOP_AND_RESET_ENCODER }

    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;

    private Direction direction = Direction.FORWARD;

    private DcMotorEx motor;

    private ScheduledExecutorService executor;

    public abstract static class ScheduleType {}

    public static class ScheduleAtFixedRate extends ScheduleType {

        public ScheduleAtFixedRate(Integer rate) {
            isIllegalArgumentGivenToScheduler(rate);
            SCHEDULE_TYPE = "FIXED_RATE";
            SCHEDULE_RATE = rate;
        }
    }

    public static class ScheduleWithFixedDelay extends ScheduleType {

        public ScheduleWithFixedDelay(Integer rate) {
            isIllegalArgumentGivenToScheduler(rate);
            SCHEDULE_TYPE = "FIXED_DELAY";
            SCHEDULE_RATE = rate;
        }
    }

    public static class AutomaticSmartScheduling extends ScheduleType {

        public AutomaticSmartScheduling(int positiveBias) {
            isIllegalArgumentGivenToScheduler(positiveBias);
            SCHEDULE_TYPE = "AUTO";
            SCHEDULE_BIAS = positiveBias;
        }
    }

    public static void isIllegalArgumentGivenToScheduler(long scheduleRateArgument) {
        if (scheduleRateArgument < 1) throw new IllegalArgumentException("scheduleRateArgument cannot be less than zero!");
    }

    public UnboundedMotor(@NonNull HardwareMap hardwareMap, String deviceName, double MAX_MILLIAMPS, int tolerance) {
        this.MAX_MILLIAMPS = MAX_MILLIAMPS;

        this.deviceName = deviceName;
        motor = hardwareMap.get(DcMotorEx.class, deviceName);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setTargetPositionTolerance(tolerance);

        executor = Executors.newSingleThreadScheduledExecutor();
        asyncStarter = Executors.newSingleThreadExecutor();
    }

    public UnboundedMotor(@NonNull HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap, deviceName, 5100, 2);
    }

    private Double getSumOfArrayListItems(@NonNull ArrayList<Double> arrayList) {
        Double sum = 0.0;

        for (int i = 0; i < arrayList.size(); i++) {
            sum += arrayList.get(i);
        }

        return sum;
    }

    public void start(ScheduleType rate/** all actions done in constructor renders calling methods redundant **/ ) {
        if (!isStarted) {
            if (SCHEDULE_TYPE.equals("FIXED_DELAY")) executor.scheduleWithFixedDelay(this::update,0, SCHEDULE_RATE, TimeUnit.MILLISECONDS);
            else if (SCHEDULE_TYPE.equals("FIXED_RATE")) executor.scheduleAtFixedRate(this::update, 0, SCHEDULE_RATE, TimeUnit.MILLISECONDS);
            else {
                runner = asyncStarter.submit(() -> {
                    try {
                        timeBetweenTicks.add(20.0);
                        Double prev_timeBetweenTick = 20.0;
                        Double curr_timeBetweenTick = 0.0;
                        double averageOfTimeInBetweenTicks;
                        double averageOfTimeWaited;
                        ElapsedTime timer = new ElapsedTime();

                        while (process) {

                            update();
                            averageOfTimeInBetweenTicks = (getSumOfArrayListItems(timeBetweenTicks) / timeBetweenTicks.size());
                            averageOfTimeWaited = getSumOfArrayListItems(timeWaited) / timeWaited.size();
                            timeWaited.add(0.0 + averageOfTimeInBetweenTicks);
                            sleep((long) (averageOfTimeInBetweenTicks + (averageOfTimeWaited / averageOfTimeInBetweenTicks)));
                            sleep(SCHEDULE_BIAS);

                            curr_timeBetweenTick = (timer.milliseconds() - SCHEDULE_BIAS);
                            timeBetweenTicks.add(curr_timeBetweenTick);
                            timeBetweenTicks.set(0, 20.0 * (curr_timeBetweenTick / prev_timeBetweenTick));
                            prev_timeBetweenTick = curr_timeBetweenTick;
                            timer.reset();
                        }
                    }
                    catch (Exception e) {
                        Thread.currentThread().interrupt();
                    }
                });
            }
        }
        else throw new MotorAlreadyStartedException("Motor has already been started and cannot be started again!");
    }

    private void update() {

        if (direction == Direction.FORWARD) motor.setDirection(DcMotor.Direction.FORWARD);
        else motor.setDirection(DcMotor.Direction.REVERSE);

        if (getCurrent() > MAX_MILLIAMPS) power = 0;

        if (mode == RunMode.STOP_AND_RESET_ENCODER) {
            power = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mode = RunMode.RUN_USING_ENCODER;
        }

        if (mode == RunMode.RUN_USING_ENCODER) {
            motor.setTargetPosition(targetPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        motor.setPower(power);
    }

    private void endAsyncStarter() {
        process = false;
        if (runner != null) {
            runner.cancel(true);
        }
        asyncStarter.shutdown();
    }

    public void kill() {
        executor.shutdown();
        endAsyncStarter();
        motor.setPower(0);
    }

    public void setMaxCurrent(double MAX_MILLIAMPS) {
        this.MAX_MILLIAMPS = MAX_MILLIAMPS;
    }

    public void setTargetPositionTolerance(int tolerance) {
        motor.setTargetPositionTolerance(tolerance);
    }

    public void setPower(double power) { if (getCurrent() <= MAX_MILLIAMPS) this.power = Math.min(power, 1); }

    public void setPosition(int position) {
        if (mode == RunMode.RUN_WITHOUT_ENCODER) throw new IllegalArgumentException("Cannot set target position when motor not using encoder!");
        targetPosition = position;
    }

    public void setMode(RunMode mode) { this.mode = mode; }

    public void setDirection(Direction direction) {
        this.direction = direction;
        if (direction == Direction.FORWARD) motor.setDirection(DcMotor.Direction.FORWARD);
        else motor.setDirection(DcMotor.Direction.REVERSE);
    }

    public Direction getDirection() { return direction; }

    public RunMode getMode() { return mode; }

    public double getCurrent() { return motor.getCurrent(CurrentUnit.MILLIAMPS); }

    public int getTargetPositionTolerance() { return motor.getTargetPositionTolerance(); }

    public double getPower() { return motor.getPower(); }

    public double getCurrentPosition() { return motor.getCurrentPosition(); }

    public int getTargetPosition() { return targetPosition; }

}