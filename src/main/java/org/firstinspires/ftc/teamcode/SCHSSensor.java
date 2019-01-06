package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.util.Log;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class SCHSSensor {

    private ColorSensor sensorColor = null;
    private DistanceSensor sensorDist = null;
    View relativeLayout = null;

    public void initialize(HardwareMap hardwareMap) {
        sensorColor = hardwareMap.get(ColorSensor.class, "colorSensorSCHS");
        sensorDist = hardwareMap.get(DistanceSensor.class , "colorSensorSCHS");
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    }

    public void checkAngleAndCorrect() {

    }

    public void scanBallColor() throws InterruptedException {
        scanColor();
    }

    public void scanPicture() {

    }

    public void scanColor() throws InterruptedException {
        /*relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });*/

        float[] hsvValues = {0F , 0F , 0F};
        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;
/*
        //NormalizedRGBA colors = sensorColor.getNormalizedColors();

        //float[] colorHSV = Color.argb

        //Color.colorToHSV(colors.toColor() , hsvValues);

        //int color = colors.toColor();

        float max = Math.max(Math.max(Math.max(colors.red , colors.green) , colors.blue) , colors.alpha);
        colors.red /= max;
        colors.green /= max;
        colors.blue /= max;
        color = colors.toColor();*/

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        //Color.RGBToHSV(Color.red(color) , Color.green(color) , Color.blue(color) , hsvValues);

        Log.d("Status", "SCHSSensor:scanColor: alpha value" + sensorColor.alpha());
        Log.d("Status", "SCHSSensor:scanColor: red value" + sensorColor.red());
        Log.d("Status", "SCHSSensor:scanColor: green value" + sensorColor.green());
        Log.d("Status", "SCHSSensor:scanColor: blue value" + sensorColor.blue());

        Log.d("Status", "SCHSSensor:scanColor: hue value" + hsvValues[0]);
        Log.d("Status", "SCHSSensor:scanColor: saturation value" + hsvValues[1]);
        Log.d("Status", "SCHSSensor:scanColor: value/brightness value" + hsvValues[2]);

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
    }

}
