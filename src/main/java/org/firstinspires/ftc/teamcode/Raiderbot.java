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
    private SCHSPicVuforia robotVuforia;

    public void initialize(HardwareMap hardwareMap) {
        minArm = new SCHSArm();
        landerArm = new SCHSArm();

        //robotSensors = new SCHSSensor();
        //robotSensors.initialize(hardwareMap);

        robotMotors = new SCHSMotor();
        robotMotors.initialize(hardwareMap);

        robotTFlow = new SCHSObjectDetection();
        robotTFlow.initialize(hardwareMap);

        robotVuforia = new SCHSPicVuforia();
        robotVuforia.initialize(hardwareMap, robotTFlow.getVuforia(), robotTFlow.getParameters());

        Log.d("Status" , " Raiderbot:initialize:after robotMotors, robotTFlow, robotVuforia initialized");
    }

    public void cleanShutDown() {
        robotMotors.cleanShutDown();

        //robotTFlow.cleanShutDown();
    }

    public void goToCrater() {

        //to be corrected
        Log.d("Status" , "Raiderbot:goToCrater: inside go to crater");
        robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, GO_TO_CRATER_DIST);
        Log.d("Status" , "Raiderbot:goToCrater: after move to crater");

        robotMotors.moveServo( TURN_MASCOT_SERVO_ANGLE, SERVO_DIRECTION_RIGHT);
        Log.d("Status" , "SCHSRaiderbot:depositMascot: after move servo up");

    }

    //turn servo to drop mascot
    public void depositMascot() {
        Log.d("Status" , "SCHSRaiderbot:depositMascot: inside deposit mascot");
        robotMotors.moveServo( TURN_MASCOT_SERVO_ANGLE, SERVO_DIRECTION_LEFT);
        Log.d("Status" , "SCHSRaiderbot:depositMascot: after move servo");
    }

    public void scanPictures() {
        boolean goldOnLeft = robotTFlow.getIsGoldOnLeft();
        boolean goldOnRight = robotTFlow.getIsGoldOnRight();
        boolean goldOnCenter = robotTFlow.getIsGoldOnCenter();

        Log.d("Status" , "SCHSRaiderbot:scanPictures: goldOnLeft = " + goldOnLeft);
        Log.d("Status" , "SCHSRaiderbot:scanPictures: goldOnRight = " + goldOnRight);
        Log.d("Status" , "SCHSRaiderbot:scanPictures: goldOnCenter = " + goldOnCenter);

        //turn to face the wall picture
        if (goldOnLeft == true){
            robotMotors.turnWithGyro(0.25, TURN_TO_PICTURE_LEFT_ANGLE, TURN_TO_PICTURE_DIRECTION);
            Log.d("Status" , "SCHSRaiderbot:scanPictures: gold on left turn to picture");
            robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, GO_TO_PICTURE_LEFT_DIST);
            Log.d("Status" , "SCHSRaiderbot:scanPictures: gold on left after move to picture");
        } else if (goldOnRight == true){
            robotMotors.turnWithGyro(0.25, TURN_TO_PICTURE_RIGHT_ANGLE, TURN_TO_PICTURE_DIRECTION);
            Log.d("Status" , "SCHSRaiderbot:scanPictures: gold on right turn to picture");
            robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, GO_TO_PICTURE_RIGHT_DIST);
            Log.d("Status" , "SCHSRaiderbot:scanPictures: gold on right after move to picture");
        } else if (goldOnCenter == true){
            robotMotors.turnWithGyro(0.25, TURN_TO_PICTURE_CENTER_ANGLE, TURN_TO_PICTURE_DIRECTION);
            Log.d("Status" , "SCHSRaiderbot:scanPictures: gold on center turn to picture");
            robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, GO_TO_PICTURE_CENTER_DIST);
            Log.d("Status" , "SCHSRaiderbot:scanPictures: gold on center after move to picture");
        } else {
            robotMotors.turnWithGyro(0.25, TURN_TO_PICTURE_CENTER_ANGLE, TURN_TO_PICTURE_DIRECTION);
            Log.d("Status" , "SCHSRaiderbot:scanPictures: no gold detected turn to picture");
            robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, GO_TO_PICTURE_CENTER_DIST);
            Log.d("Status" , "SCHSRaiderbot:scanPictures: no gold detected after move to picture");
        }

        Log.d("Status" , "SCHSRaiderbot:scanPictures: after turn to picture");

        //scan for picture
        robotVuforia.trackPictures();
        Log.d("Status" , "SCHSRaiderbot:scanPictures: after trackPictures");

        String pictureName = robotVuforia.getPictureDetected();

        if (pictureName.equals(ROVER_PIC) || pictureName.equals(FOOT_PIC)) {
            robotMotors.turnWithGyro(0.25, TURN_TO_DEPOT_ANGLE, LEFT_TURN);
            Log.d("Status ", "SCHSRaiderbot:scanPictures: Rover Pic or Foot Pic left turn");
            robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, GO_TO_DEPOT_DIST);
            Log.d("Status ", "SCHSRaiderbot:scanPictures: after move to depot");

        } else if (pictureName.equals(CRATERS_PIC) || pictureName.equals(SPACE_PIC)) {
            robotMotors.turnWithGyro(0.25, 2*TURN_TO_DEPOT_ANGLE, RIGHT_TURN);
            Log.d("Status ", "SCHSRaiderbot:scanPictures: Craters Pic or Space Pic right turn");
            robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, GO_TO_DEPOT_DIST);
            Log.d("Status ", "SCHSRaiderbot:scanPictures: after move to depot");

        } else {
            Log.d("Status ", "SCHSRaiderbot:scanPictures: No picture detected");
            robotMotors.turnWithGyro(0.25, TURN_TO_DEPOT_ANGLE, LEFT_TURN);
            Log.d("Status ", "SCHSRaiderbot:scanPictures: no pic detected left turn");
            robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, GO_TO_DEPOT_DIST);
            Log.d("Status ", "SCHSRaiderbot:scanPictures: after move to depot");
        }
    }

    public void testFunction() throws InterruptedException {
        robotMotors.turnWithGyro(0.25, 45, RIGHT_TURN);
        Log.d("Status ", "SCHSRaiderbot:depositMascot: after turn 20 degrees left");

        Log.d("Status ", "SCHSRaiderbot:depositMascot: turn 1 final angle" + robotMotors.getCurrentAngle());

        robotMotors.turnWithGyro(0.25, 90, RIGHT_TURN);
        Log.d("Status ", "SCHSRaiderbot:depositMascot: after turn 90 degress right");

        Log.d("Status ", "SCHSRaiderbot:depositMascot: turn 2 final angle" + robotMotors.getCurrentAngle());

        robotMotors.turnWithGyro(0.25, 20, RIGHT_TURN);
        Log.d("Status ", "SCHSRaiderbot:depositMascot: after turn 45 degress right");

        Log.d("Status ", "SCHSRaiderbot:depositMascot: turn 3 final angle" + robotMotors.getCurrentAngle());

        robotMotors.turnWithGyro(0.25, 180, RIGHT_TURN);
        Log.d("Status ", "SCHSRaiderbot:depositMascot: after turn 180 degress right");

        Log.d("Status ", "SCHSRaiderbot:depositMascot: turn 4 final angle" + robotMotors.getCurrentAngle());

        /*
        //move straight using gyro
        robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, MOVE_FROM_LANDER_DIST);
        Log.d("Status" , "SCHSRaiderbot:depositMascot: after move forward with gyro");

        sleep(2000);

        //move back using gyro
        robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, -MOVE_FROM_LANDER_DIST);
        Log.d("Status" , "SCHSRaiderbot:depositMascot: after move back with gyro");
        */
    }

    public void dropFromLander() throws InterruptedException {
        Log.d("Status" , "SCHSRaiderbot:dropFromLander: inside drop from lander");
        robotMotors.moveLanderArm(POWER_FULL_FORWARD, DROP_FROM_LANDER_DIST, ARM_MOVE_DOWN_DIRECTION);
        Log.d("Status" , "SCHSRaiderbot:dropFromLander: after move arm");
    }

    public void senseBallAndSample() throws InterruptedException {

        // move straight after dropping from lander. Prepares for sampling maneuvers.
        robotMotors.moveStraightWithGyro(POWER_HALF_FORWARD, MOVE_FROM_LANDER_DIST); //perviously speed was 0.2
        Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: after move from lander");

        sleep(2000);

        // Sense the gold cube. Record the angle of the mineral w/ respect to robot.
        robotTFlow.detectGoldMineral();
        Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: after detect gold mineral");
        double turnToGoldAngle = BALL_ANGLE_ERROR * robotTFlow.getMineralAngle();
        Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: turn angle " + turnToGoldAngle);

        sleep(1000);

        // Turn robot to face the gold mineral.
        int turnToGoldDirection = 0;
        double reverseAngle = Math.abs(turnToGoldAngle);
        int reverseDirection = 0;
        if (turnToGoldAngle < -10) {
            turnToGoldDirection = LEFT_TURN;
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: left turn");
            robotMotors.turnWithGyro(0.25 , turnToGoldAngle - CAMERA_ANGLE_ERROR, turnToGoldDirection);
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: after turn");
            reverseDirection = RIGHT_TURN;
        } else if (turnToGoldAngle > 10){
            turnToGoldDirection = RIGHT_TURN;
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: right turn");
            robotMotors.turnWithGyro(0.25 , turnToGoldAngle + CAMERA_ANGLE_ERROR, turnToGoldDirection);
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: after turn");
            reverseDirection = LEFT_TURN;
        } else if (turnToGoldAngle > -10 && turnToGoldAngle <0) {
            turnToGoldDirection = LEFT_TURN;
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: left turn");
            robotMotors.turnWithGyro(0.25 , turnToGoldAngle + CAMERA_ANGLE_SMALL_ERROR, turnToGoldDirection);
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: after turn");
            reverseDirection = RIGHT_TURN;
            reverseAngle = turnToGoldAngle - CAMERA_ANGLE_SMALL_ERROR;
        } else if (turnToGoldAngle < 10 && turnToGoldAngle > 0) {
            turnToGoldDirection = RIGHT_TURN;
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: right turn");
            robotMotors.turnWithGyro(0.25 , turnToGoldAngle - CAMERA_ANGLE_SMALL_ERROR, turnToGoldDirection);
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: after turn");
            reverseDirection = LEFT_TURN;
            reverseAngle = turnToGoldAngle - CAMERA_ANGLE_SMALL_ERROR;
        } else {
            Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: no turn");
        }

        sleep(1000);

        // Find distance to the mineral in inches.
        int moveDist = robotTFlow.getMineralDist();
        Log.d("Status" , "SCHSRaiderbot:senseBallAndSample: moveDist" + moveDist);

        Log.d("Status ", "angle after turn to mineral" + robotMotors.getCurrentAngle());

        // Move calculated distance to touch the gold mineral.
        robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, MOVE_TO_BALL);
        Log.d("Status", "SCHSRaiderbot: after move to mineral");

        sleep(1000);

        robotMotors.moveStraightWithGyro(POWER_FULL_FORWARD, MOVE_BACK_FROM_BALL);
        Log.d("Status", "SCHSRaiderbot: after move back from mineral");

        robotMotors.turnWithGyro(0.25, reverseAngle + CAMERA_ANGLE_ERROR, reverseDirection);
        Log.d("Status", "SCHSRaiderbot: after turn back");

        // Move back to starting position
        // Turn to face forward (return to initial angle).
    }

    // Causes the current thread to wait a certain number of seconds. Used instead of given sleep function.
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

