package org.firstinspires.ftc.teamcode;

import android.util.Log;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.SCHSConstants.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Raiderbot {

    private SCHSArm minArm;
    private SCHSArm landerArm;
    private SCHSMotor robotMotors;
    private SCHSSensor robotSensors;
    private SCHSObjectDetection robotTFlow;

    public void initialize(HardwareMap hardwareMap) {
        minArm = new SCHSArm();
        landerArm = new SCHSArm();

        robotSensors = new SCHSSensor();
        robotSensors.initialize(hardwareMap);

        robotMotors = new SCHSMotor();
        robotMotors.initialize(hardwareMap);

        robotTFlow = new SCHSObjectDetection();
        robotTFlow.initialize(hardwareMap);

        Log.d("Status" , " Raiderbot:initialize:after robotMotors initialized");
    }

    public void goToCrater() {

        //to be corrected
        //Log.d("Status" , "Raiderbot:goToCrater: inside go to crater");
        //robotMotors.moveToPosition(POWER_FULL_FORWARD, MOVE_FROM_LANDER_DIST);
    }

    public void orientRobot() {

    }

    public void depositMascot() throws InterruptedException {

        //move straight using gyro
        robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, MOVE_FROM_LANDER_DIST, 0);
        //robotMotors.moveToPosition(POWER_FULL_FORWARD,MOVE_FROM_LANDER_DIST);
        Log.d("Status" , "SCHSRaiderbot:depositMascot: after move straight with gyro");

        sleep(5000);

        robotMotors.turnWithGyro(0.25, 45, LEFT_TURN);
        Log.d("Status" , "SCHSRaiderbot:depositMascot: after turn 45 with gyro");

        sleep(5000);

        robotMotors.turnWithGyro(0.25, 90, RIGHT_TURN);
        Log.d("Status" , "SCHSRaiderbot:depositMascot: after turn -90 with gyro");

        sleep(5000);

        robotMotors.turnWithGyro(0.25, 45, LEFT_TURN);
        Log.d("Status" , "SCHSRaiderbot:depositMascot: after turn 45 with gyro");

        //first move away from lander
        //robotMotors.moveToPosition(POWER_FULL_FORWARD, MOVE_FROM_LANDER_DIST );
        //Log.d("Status" , "Raiderbot:depositMascot: after move away from lander");
        //robotSensors.checkAngleAndCorrect();

        //sleep(5000);

        /*
        //turn towards the wall
        robotMotors.turnAtAngle(POWER_FULL_FORWARD, TURN_TO_PICTURE_DIRECTION , TURN_TO_WALL_ANGLE);
        Log.d("Status" , "Raiderbot:depositMascot: after turn to wall");
        //robotSensors.checkAngleAndCorrect();

        //sleep(5000);

        //move towards to wall
        robotMotors.moveToPosition(POWER_FULL_FORWARD, GO_TO_WALL_DIST );
        Log.d("Status" , "Raiderbot:depositMascot: after move to wall");
        //robotSensors.checkAngleAndCorrect();

        //turn towards the picture
        robotMotors.turnAtAngle(POWER_FULL_FORWARD , TURN_TO_PICTURE_DIRECTION , TURN_TO_WALL_ANGLE);
        Log.d("Status" , "Raiderbot:depositMascot: after turn to picture");
        //robotSensors.checkAngleAndCorrect();

        //move towards the picture
        robotMotors.moveToPosition(POWER_FULL_FORWARD, GO_TO_PICTURE_DIST);
        Log.d("Status" , "Raiderbot:depositMascot: after move to picture");
        //robotSensors.checkAngleAndCorrect();
        */

    }

    public void dropFromLander() throws InterruptedException {

    }

    public void senseBallAndSample() throws InterruptedException {
        robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, MOVE_FROM_LANDER_DIST, 0);
        Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: after move straight with gyro");

        sleep(1000);

        robotTFlow.detectGoldMineral();
        Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: after detect gold mineral");
        int turnAngle = robotTFlow.getMineralAngle();
        Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: turn angle " + turnAngle);

        sleep(1000);

        int turnDirection = 0;
        if (turnAngle < 0) {
            turnDirection = LEFT_TURN;
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: left turn");
            robotMotors.turnWithGyro(0.25 , turnAngle , turnDirection);
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: after turn");

        } else if (turnAngle > 0){
            turnDirection = RIGHT_TURN;
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: right turn");
            robotMotors.turnWithGyro(0.25 , turnAngle , turnDirection);
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: after turn");
        } else {
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: no turn");
        }

        sleep(1000);

        robotMotors.gyroReset();

        int moveDist = robotTFlow.getMineralDist();
        Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: moveDist" + moveDist);
        robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, 48, 0);
        //Log.d("Status", "SCHSRaiderbot: before detectGoldMineral");
        //robotTFlow.detectGoldMineral();
        //Log.d("Status", "SCHSRaiderbot: after detectGoldMineral");
    }

    public static void sleep(long sleepTime) {

        long wakeupTime = System.currentTimeMillis() + sleepTime;

        while (sleepTime > 0) {
            try
            {
                Thread.sleep(sleepTime);
            }
            catch (InterruptedException e)
            {
            }
            sleepTime = wakeupTime - System.currentTimeMillis();
        }
    }

}

