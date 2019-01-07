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

    public void cleanShutDown() {
        robotMotors.cleanShutDown();

        robotTFlow.cleanShutDown();
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
        int turnToGoldAngle = robotTFlow.getMineralAngle();
        Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: turn angle " + turnToGoldAngle);

        sleep(1000);

        int turnToGoldDirection = 0;
        int reverseAngle = -turnToGoldAngle;
        int reverseDirection = 0;
        if (turnToGoldAngle < 0) {
            turnToGoldDirection = LEFT_TURN;
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: left turn");
            robotMotors.turnWithGyro(0.25 , turnToGoldAngle, turnToGoldDirection);
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: after turn");
            reverseDirection = RIGHT_TURN;
        } else if (turnToGoldAngle > 0){
            turnToGoldDirection = RIGHT_TURN;
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: right turn");
            robotMotors.turnWithGyro(0.25 , turnToGoldAngle, turnToGoldDirection);
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: after turn");
            reverseDirection = LEFT_TURN;
        } else {
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: no turn");
        }

        sleep(1000);

        int moveDist = robotTFlow.getMineralDist();
        Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: moveDist" + moveDist);

        robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, 48, 0);
        Log.d("Status", "SCHSRaiderbot: after move to mineral");
        //robotTFlow.detectGoldMineral();
        //Log.d("Status", "SCHSRaiderbot: after detectGoldMineral");

        sleep(1000);

        robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD , -48, 0);
        Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: after move back to lander");

        sleep(1000);

        robotMotors.turnWithGyro(0.25, reverseAngle, reverseDirection);
        Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: reverseAngle" + reverseAngle);
        Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: reverseDirection" + reverseDirection);

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

