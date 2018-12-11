package org.firstinspires.ftc.teamcode;

import android.util.Log;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static com.qualcomm.robotcore.hardware.DcMotor.*;

public class SCHSMotor extends SCHSController {

    private int encoderValue;
    private DcMotor motorLeft = null;
    private DcMotor motorRight = null;
    private HardwareMap myHWmap = null;

    public void initialize(HardwareMap hardwareMap) {
        motorLeft = hardwareMap.get(DcMotor.class, "leftMotor");
        motorRight = hardwareMap.get(DcMotor.class, "rightMotor");
        motorLeft.setDirection(Direction.REVERSE);
        motorRight.setDirection(Direction.FORWARD);

    }

    DcMotor getMotorLeft() {
        return motorLeft;
    }

    DcMotor getMotorRight() {
        return motorRight;
    }

    public void moveToPosition(double powerStart , int position) throws InterruptedException {

        Log.d("Status" , "SCHSMotor:moveToPosition: before movement");

        // This defines the power level; between 1 and -1.

        motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);

        while(motorLeft.getCurrentPosition() != 0 || motorRight.getCurrentPosition() != 0) { //Ensures encoders are zero
            motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
            //waitOneFullHardwareCycle(); //Needed within all loops
        }

        int positionLeftMotor = motorLeft.getCurrentPosition();
        int positionRightMotor = motorRight.getCurrentPosition();

        Log.d("Status" , "SCHSMotor:moveToPosition:positionLeftMotor" + positionLeftMotor);
        Log.d("Status" , "SCHSMotor:moveToPosition:positionRightMotor" + positionRightMotor);

        //set the power of the motor

        // set the motor to position mode
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        verifyDistanceAndMoveTwoMotors(powerStart , position);

        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    //moving robot with both motors
    public void verifyDistanceAndMoveTwoMotors(double powerStart , int desiredPosition) {

        double countsPerInch = (CHMOTOR_COUNTS_PER_REVOLUTION) / (TRACTION_WHEEL_DIAMETER * Math.PI);
        Log.d("Status" , "SCHSMotor:verifyDistanceAndMoveTwoMotors:counts per inch" + countsPerInch);

        double temp = countsPerInch * desiredPosition;
        Log.d("Status" , "SCHSMotor:verifyDistanceAndMoveTwoMotors:temp" + temp);

        int encoderValue = (int)temp;

        Log.d("Status" , "SCHSMotor:verifyDistanceAndMoveTwoMotors:encoder value after cast to int" + encoderValue);

       Log.d("Status" , "SCHSMotor:verifyDistanceAndMoveTwoMotors:before while loop ");

       motorLeft.setTargetPosition(encoderValue);
       motorRight.setTargetPosition(encoderValue);

        //motorLeft.setTargetPosition(500);
        //motorRight.setTargetPosition(500);

        Log.d("Status" , "SCHSMotor:verifyDistanceAndMoveTwoMotors:power start" + powerStart);

        motorLeft.setPower(powerStart);
        motorRight.setPower(powerStart);

        while(motorLeft.getCurrentPosition() < motorLeft.getTargetPosition() || motorRight.getCurrentPosition() < motorRight.getTargetPosition()) { //While target has not been reached
            //waitOneFullHardwareCycle(); //Needed within all loops
        }

        int distanceMovedLeft = motorLeft.getCurrentPosition();
        int distanceMovedRight = motorRight.getCurrentPosition();

        Log.d("Status" , "SCHSMotor:verifyDistanceAndMoveTwoMotors:distanceMovedLeft" + distanceMovedLeft);
        Log.d("Status" , "SCHSMotor:verifyDistanceAndMoveTwoMotors:distanceMovedRight" + distanceMovedRight);

        Log.d("Status" , "SCHSMotor:verifyDistanceAndMoveTwoMotors:after while loop ");
    }

    // for moving arms with one motor, possibly mascot
    public void verifyDistanceAndMoveOneMotors(DcMotor motor, int desiredPosition) {

        int distanceMoved = 0;
        int totalDistance = desiredPosition;

        while (distanceMoved < desiredPosition ) {

            motorLeft.setTargetPosition(totalDistance);
            distanceMoved = motor.getCurrentPosition();
            totalDistance = desiredPosition - distanceMoved;

        }
    }

    //for turning the robot, put power on one motor
    public void turnAtAngle(double powerTurn , int turnDirection , int turnAngle) {

        Log.d("Status" , "SCHSMotor:turnAtAngle:turnAngle" + turnAngle);

        double countsPerInch = (CHMOTOR_COUNTS_PER_REVOLUTION) / (TRACTION_WHEEL_DIAMETER * Math.PI);
        Log.d("Status" , "SCHSMotor:turnAtAngle:counts per inch" + countsPerInch);

        double distToTravel = 18.85;
        //double distToTravel = (Math.PI)*(REAR_WHEEL_BASE_)*(turnAngle/360);
        Log.d("Status" , "SCHSMotor:turnAtAngle:distToTravel" + distToTravel);

        double temp = countsPerInch * distToTravel;
        Log.d("Status" , "SCHSMotor:turnAtAngle:temp" + temp);

        int angleDistance = (int) temp;
        Log.d("Status" , "SCHSMotor:turnAtAngle:angleDistance" + angleDistance);

        motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);

        while(motorLeft.getCurrentPosition() != 0 || motorRight.getCurrentPosition() != 0) { //Ensures encoders are zero
            motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
            //waitOneFullHardwareCycle(); //Needed within all loops
        }

        if (turnDirection == LEFT_TURN) {
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setTargetPosition(angleDistance);
            motorRight.setPower(powerTurn);

            while(motorRight.getCurrentPosition() < motorRight.getTargetPosition()) { //While target has not been reached
                //waitOneFullHardwareCycle(); //Needed within all loops
            }

            Log.d("Status" , "SCHSMotor:turnAtAngle:left turn");

            motorRight.setPower(0);
        }

        if (turnDirection == RIGHT_TURN) {
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeft.setTargetPosition(angleDistance);
            motorLeft.setPower(powerTurn);

            while(motorLeft.getCurrentPosition() < motorLeft.getTargetPosition()) { //While target has not been reached
                //waitOneFullHardwareCycle(); //Needed within all loops
            }

            Log.d("Status" , "SCHSMotor:turnAtAngle:right turn");

            motorLeft.setPower(0);
        }
    }
}
