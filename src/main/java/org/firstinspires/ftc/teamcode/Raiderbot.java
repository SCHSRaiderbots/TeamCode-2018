package org.firstinspires.ftc.teamcode;

import android.util.Log;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Raiderbot {

    private SCHSArm minArm;
    private SCHSArm landerArm;
    private SCHSMotor robotMotors;
    private SCHSSensor touchCheck;
    private SCHSSensor colorCheck;
    private SCHSSensor gyroSensor;

    public void initialize() {
        minArm = new SCHSArm();
        landerArm = new SCHSArm();
        robotMotors = new SCHSMotor();

        if (robotMotors == null) {
            Log.d("Status" , "robot motors is null");
        } else {
            Log.d("Status" , "robot motors is not null");
        }

        touchCheck = new SCHSSensor();
        colorCheck = new SCHSSensor();
        gyroSensor = new SCHSSensor();
    }

    public void goToCrater(HardwareMap hardwareMap) {

        initialize();

        Log.d("Status" , "inside dropFromLander method");
        robotMotors.moveToPosition(POWER_HALF_FORWARD,0,MOVE_FROM_LANDER_DIST , hardwareMap);
    }

    public void orientRobot() {

    }

    public void depositMascot(HardwareMap hardwareMap) {

        initialize();

        robotMotors.moveToPosition(POWER_HALF_FORWARD , 0 , MOVE_FROM_LANDER_DIST , hardwareMap );


    }

    public void dropFromLander() {

    }


}
