package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "OurColorSensor")
public class OurColorSensor extends LinearOpMode {
    public void runOpMode() {}

    // Sensor and color-related fields
    private NormalizedColorSensor colorSensor;
    private final float[] hsvValues = new float[3];
    private View relativeLayout;

    // Enum for detected colors
    public enum DetectedColor {
        BLUE, YELLOW, RED, OTHER
    }

    private DetectedColor detectedColor = DetectedColor.OTHER;
    private volatile String color = "";

    public void initialize(HardwareMap hardwareMap) {
        // Initialize the color sensor
        initColorSensor(hardwareMap);

        telemetry.addData("Status", "Initialized");
    }

    public DetectedColor getColor() {
        // Detect the color
        detectedColor = detectColor();

        // Perform actions based on detected color
        if (detectedColor == DetectedColor.BLUE) {
            //telemetry.addData("Action", "Intake Blue Sample");
        } else if (detectedColor == DetectedColor.RED) {
            //telemetry.addData("Action", "YEET Red Sample");
        } else if (detectedColor == DetectedColor.YELLOW) {
            //telemetry.addData("Action", "Neutral Sample");
        } else {
            //telemetry.addData("Action", "No Action");
        }

        //telemetry.addData("Detected Color", detectedColor);
        //telemetry.addData("Hue Value", hsvValues[0]);
        //telemetry.update();

        return detectedColor;
    }

    // Initialize the color sensor
    private void initColorSensor(HardwareMap hardwareMap) {
        int relativeLayoutId = hardwareMap.appContext.getResources()
                .getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        colorSensor.setGain(2); // Set the gain
    }

    public DetectedColor detectColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        // Determine detected color based on hue
        if (hsvValues[0] >= 190 && hsvValues[0] <= 240) {
            color = "BLUE"; // Set color first
            return DetectedColor.BLUE; // Then return
        } else if (hsvValues[0] >= 50 && hsvValues[0] <= 90) {
            color = "YELLOW"; // Set color first
            return DetectedColor.YELLOW; // Then return
        } else if (hsvValues[0] >= 0 && hsvValues[0] <= 30) {
            color = "RED"; // Set color first
            return DetectedColor.RED; // Then return
        } else {
            color = "OTHER"; // Set color first
            return DetectedColor.OTHER; // Then return
        }
    }
}