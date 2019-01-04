package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SCHSSensor {

    private ColorSensor sensorColor = null;
    private DistanceSensor sensorDist = null;

    public void initialize(HardwareMap hardwareMap) {
        sensorColor = hardwareMap.get(ColorSensor.class, "colorSensorSCHS");
    }

    public void checkAngleAndCorrect() {

    }

    public void scanBallColor() throws InterruptedException {
        scanColor();

    }

    public void scanPicture() {

    }

    public void scanColor() throws InterruptedException {

        int redValue = sensorColor.red();
        int greenValue = sensorColor.green();
        int blueValue = sensorColor.blue();

        Log.d("Status", "SCHSSensor:scanColor: Red Value" + redValue);
        Log.d("Status", "SCHSSensor:scanColor: Green Value" + greenValue);
        Log.d("Status", "SCHSSensor:scanColor: Blue Value" + blueValue);
        Log.d("Status" , "wireless connection 3");

    }

}
