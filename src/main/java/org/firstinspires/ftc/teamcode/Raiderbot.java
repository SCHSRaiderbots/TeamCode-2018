package org.firstinspires.ftc.teamcode;

import android.util.Log;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Raiderbot {

    private SCHSArm minArm;
    private SCHSArm landerArm;
    private SCHSMotor robotMotors;
    private SCHSSensor robotSensors;

    public void initialize(HardwareMap hardwareMap) {
        minArm = new SCHSArm();
        landerArm = new SCHSArm();
        robotMotors = new SCHSMotor();

        if (robotMotors == null) {
            Log.d("Status" , "robot motors is null");
        } else {
            Log.d("Status" , "robot motors is not null");
        }

        robotMotors.initialize(hardwareMap);

        Log.d("Status" , " Raiderbot:initialize:after robotMotors initialized");

        robotSensors = new SCHSSensor();
    }

    public void goToCrater() {

        Log.d("Status" , "inside dropFromLander method");
        robotMotors.moveToPosition(POWER_HALF_FORWARD , MOVE_FROM_LANDER_DIST);
    }

    public void orientRobot() {

    }

    public void depositMascot() {

        //first move away from lander
        robotMotors.moveToPosition(POWER_HALF_FORWARD , MOVE_FROM_LANDER_DIST );
        Log.d("Status" , "Raiderbot:depositMascot: after move away from lander");
        robotSensors.checkAngleAndCorrect();

        //turn towards the wall
        robotMotors.turnAtAngle(TURN_TO_WALL_DIRECTION , TURN_TO_WALL_ANGLE);
        Log.d("Status" , "Raiderbot:depositMascot: after turn to wall");
        robotSensors.checkAngleAndCorrect();

        //move towards to wall
        robotMotors.moveToPosition(POWER_HALF_FORWARD , GO_TO_WALL_DIST );
        Log.d("Status" , "Raiderbot:depositMascot: after move to wall");
        robotSensors.checkAngleAndCorrect();

        //turn towards the picture
        robotMotors.turnAtAngle(TURN_TO_PICTURE_DIRECTION , TURN_TO_PICTURE_ANGLE);
        Log.d("Status" , "Raiderbot:depositMascot: after turn to picture");
        robotSensors.checkAngleAndCorrect();

        //move towards the picture
        robotMotors.moveToPosition(POWER_HALF_FORWARD , GO_TO_PICTURE_DIST);
        Log.d("Status" , "Raiderbot:depositMascot: after move to picture");
        robotSensors.checkAngleAndCorrect();

    }

    public void dropFromLander() {

    }


}
