package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.util.Log;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.*;
//import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;

public class SCHSMotor extends SCHSController {

    private int encoderValue;
    private DcMotor motorLeft = null;
    private DcMotor motorRight = null;
    private HardwareMap myHWmap = null;

    public void initialize(HardwareMap hardwareMap) {
        motorLeft = hardwareMap.get(DcMotor.class, "leftMotor");
        motorRight = hardwareMap.get(DcMotor.class, "rightMotor");
        motorLeft.setDirection(Direction.FORWARD);
        motorRight.setDirection(Direction.REVERSE);

    }

    DcMotor getMotorLeft() {
        return motorLeft;
    }

    DcMotor getMotorRight() {
        return motorRight;
    }

    public void moveToPosition(double powerStart , int position) {

        Log.d("Status" , "SCHSMotor:moveToPosition: inside ");

        //motorLeft = hardwareMap.get(DcMotor.class, "leftMotor");
        //motorRight = hardwareMap.get(DcMotor.class, "rightMotor");
        //motorLeft.setDirection(Direction.FORWARD);
        //motorRight.setDirection(Direction.REVERSE);

        // This defines the power level; between 1 and -1.

        int positionLeftMotor = motorLeft.getCurrentPosition();
        int positionRightMotor = motorRight.getCurrentPosition();

        //set the power of the motor
        motorLeft.setPower(powerStart);
        motorRight.setPower(powerStart);

        // set the motor position
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        verifyDistanceAndMoveTwoMotors(motorLeft , motorRight , position);

        motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setPower(0);
        motorRight.setPower(0);

    }

    //moving robot with both motors
    public void verifyDistanceAndMoveTwoMotors(DcMotor leftMotor, DcMotor rightMotor, int desiredPosition) {

        int distanceMovedLeft = 0;
        int distanceMovedRight = 0;

        int totalDistanceLeft = desiredPosition;
        int totalDistanceRight = desiredPosition;

        Log.d("Status" , "SCHSMotor:verifyDistanceAndMoveTwoMotors:before while loop ");

        while (distanceMovedLeft < desiredPosition || distanceMovedRight < desiredPosition) {

            leftMotor.setTargetPosition(totalDistanceLeft);
            rightMotor.setTargetPosition(totalDistanceRight);

            distanceMovedLeft = leftMotor.getCurrentPosition();
            distanceMovedRight = rightMotor.getCurrentPosition();

            totalDistanceLeft = desiredPosition - distanceMovedLeft;
            totalDistanceRight = desiredPosition - distanceMovedRight;

        }

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
    public void turnAtAngle(int turnDirection , int turnAngle) {

        double countsPerInch = (CHMOTOR_COUNTS_PER_REVOLUTION) / (TRACTION_WHEEL_DIAMETER * Math.PI);
        double distToTravel = (Math.PI)*(REAR_WHEEL_BASE_)*(turnAngle/360);
        int angleDistance = (int) Math.round(countsPerInch * distToTravel);

        if (turnDirection == LEFT_TURN) {
            motorRight.setPower(POWER_HALF_FORWARD);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorRight.setTargetPosition(angleDistance);

            motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setPower(0);
        }

        if (turnDirection == RIGHT_TURN) {
            motorLeft.setPower(POWER_HALF_FORWARD);
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorLeft.setTargetPosition(angleDistance);

            motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setPower(0);
        }
    }
}
