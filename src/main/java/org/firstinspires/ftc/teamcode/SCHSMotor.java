package org.firstinspires.ftc.teamcode;

import android.util.Log;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.bosch.BNO055IMU;

import static com.qualcomm.robotcore.hardware.DcMotor.*;

public class SCHSMotor extends SCHSController {

    private DcMotor motorLeft = null;
    private DcMotor motorRight = null;
    private DcMotor motorArm = null;
    private BNO055IMU.Parameters gyroParameters;
    private BNO055IMU imu;
    private Servo mascotServo = null;

    public void initialize(HardwareMap hardwareMap) {
        motorLeft = hardwareMap.get(DcMotor.class, "leftMotor");
        motorRight = hardwareMap.get(DcMotor.class, "rightMotor");
        motorArm = hardwareMap.get(DcMotor.class, "landerArmMotor");

        mascotServo = hardwareMap.get(Servo.class, "mascotServo");

        motorLeft.setDirection(Direction.REVERSE);
        motorRight.setDirection(Direction.FORWARD);

        gyroParameters = new BNO055IMU.Parameters();

        gyroParameters.mode                = BNO055IMU.SensorMode.IMU;
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParameters);

        while (!isStopRequested() && imu.isGyroCalibrated())  {
            Log.d("Status" , "SCHSMotor:moveStraightWithGyro: gyro is calibrating");
            sleep(50);
            idle();
        }

        Log.d("Status" , "SCHSMotor:moveStraightWithGyro: gyro done calibrating");

    }

    //method to include cleanup before exiting program
    public void cleanShutDown() {

    }

    DcMotor getMotorLeft() {
        return motorLeft;
    }

    DcMotor getMotorRight() {
        return motorRight;
    }

    public int calcEncoderValue(int inches) {
        double countsPerInch = (CHMOTOR_COUNTS_PER_REVOLUTION) / (TRACTION_WHEEL_DIAMETER * Math.PI);
        double temp = countsPerInch * inches;
        int valueForEncoder = (int) temp;
        return valueForEncoder;
    }

    //deprecated
    public void moveToPosition(double powerStart, int position) throws InterruptedException {

        Log.d("Status", "SCHSMotor:moveToPosition: before movement");

        // This defines the power level; between 1 and -1.

        motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);

        while (motorLeft.getCurrentPosition() != 0 || motorRight.getCurrentPosition() != 0) { //Ensures encoders are zero
            motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
            //waitOneFullHardwareCycle(); //Needed within all loops
        }

        int positionLeftMotor = motorLeft.getCurrentPosition();
        int positionRightMotor = motorRight.getCurrentPosition();

        Log.d("Status", "SCHSMotor:moveToPosition:positionLeftMotor" + positionLeftMotor);
        Log.d("Status", "SCHSMotor:moveToPosition:positionRightMotor" + positionRightMotor);

        //set the power of the motor

        // set the motor to position mode
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        verifyDistanceAndMoveTwoMotors(powerStart, position);

        Log.d("Status", "SCHSMotor:moveToPosition:distance Moved Final Left before power zero" + motorLeft.getCurrentPosition());
        Log.d("Status", "SCHSMotor:moveToPosition:distance Moved Final Right before power zero" + motorRight.getCurrentPosition());

        motorLeft.setPower(0);
        motorRight.setPower(0);

        Log.d("Status", "SCHSMotor:moveToPosition:distance Moved Left after power zero" + motorLeft.getCurrentPosition());
        Log.d("Status", "SCHSMotor:moveToPosition:distance Moved Right after power zero" + motorRight.getCurrentPosition());

    }

    //moving robot with both motors and encoders
    public void verifyDistanceAndMoveTwoMotors(double powerStart, int desiredPosition) {

        //converting inches to encoder values
        double countsPerInch = (CHMOTOR_COUNTS_PER_REVOLUTION) / (TRACTION_WHEEL_DIAMETER * Math.PI);
        Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors:counts per inch" + countsPerInch);

        double temp = countsPerInch * desiredPosition;
        Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors:temp" + temp);

        int encoderValue = (int) temp;
        Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors:encoder value after cast to int" + encoderValue);

        Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors:before while loop ");

        motorLeft.setTargetPosition(encoderValue);
        motorRight.setTargetPosition(encoderValue);


        Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors:power start" + powerStart);

        double motorPower = powerStart;

        while (motorLeft.getCurrentPosition() < encoderValue || motorRight.getCurrentPosition() < encoderValue) { //While target has not been reached

            if (motorLeft.getCurrentPosition() > motorRight.getCurrentPosition()) {
                motorLeft.setPower(motorPower);
                motorRight.setPower(motorPower - (motorPower * 0.15));

                //Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors: right drift");
            }

            if (motorLeft.getCurrentPosition() < motorRight.getCurrentPosition()) {
                motorLeft.setPower(motorPower - (motorPower*0.15));
                motorRight.setPower(motorPower);

                //Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors: left drift");
            }

            if (motorLeft.getCurrentPosition() == motorRight.getCurrentPosition()) {
                motorLeft.setPower(motorPower);
                motorRight.setPower(motorPower);
                //Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors: no drift");
            }


            int distanceMovedLeft = motorLeft.getCurrentPosition();
            int distanceMovedRight = motorRight.getCurrentPosition();

            if (distanceMovedLeft >= 0.75 * encoderValue || distanceMovedRight >= 0.75 * encoderValue) {
                motorPower = 0.75 * motorPower;
            }

            //Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors:distanceMovedLeft" + distanceMovedLeft);
            //Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors:distanceMovedRight" + distanceMovedRight);
        }
        Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors:after while loop ");
    }

    /*// for moving arms with one motor, possibly mascot
    public void verifyDistanceAndMoveOneMotors(int desiredPosition) {

        int distanceMoved = 0;
        int totalDistance = desiredPosition;

        while (distanceMoved < desiredPosition ) {

            motorLeft.setTargetPosition(totalDistance);
            distanceMoved = motorSingle.getCurrentPosition();
            totalDistance = desiredPosition - distanceMoved;

        }
    }*/

    //moves straight using gyro
    public void moveStraightWithGyro(double powerStart, int desiredPosition) {
        double currAngle = 0;

        Log.d("Status" , "SCHSMotor:moveStraightWithGyro: Enter Method");
        motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);

        while (motorLeft.getCurrentPosition() != 0 || motorRight.getCurrentPosition() != 0) { //Ensures encoders are zero
            motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
            Log.d("Status" , "SCHSMotor:moveStraightWithGyro: STop and Reset Loop");
            //waitOneFullHardwareCycle(); //Needed within all loops
        }
        Log.d("Status" , "SCHSMotor:moveStraightWithGyro: reset encoders");

        currAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        Log.d("Status" , "SCHSMotor:moveStraightWithGyro: currAngle " + currAngle);

        gyroDrive(powerStart, desiredPosition, currAngle); //desired position in inches
        Log.d("Status" , "SCHSMotor:moveStraightWithGyro: gyroDrive finished");

        motorLeft.setPower(0);
        motorRight.setPower(0);

        double finalAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        Log.d("Status" , "SCHSMotor:moveStraightWithGyro: finalAngle " + finalAngle);

    }

    //aligns using gyro angles
    public void gyroDrive(double speed, double distance, double currentAngle) {
        int     newLeftTarget = 0;
        int     newRightTarget = 0;
        double  max = 0;
        double  error = 0;
        double  steer = 0;
        double  leftSpeed = 0;
        double  rightSpeed = 0;
        double countsPerInch = 0;
        double slowFactor = 0;

        Log.d("Status" , "SCHSMotor:gyroDrive: initial left position " + motorLeft.getCurrentPosition());
        Log.d("Status" , "SCHSMotor:gyroDrive: initial right position " + motorRight.getCurrentPosition());


        //converting inches to encoder values
        countsPerInch = (CHMOTOR_COUNTS_PER_REVOLUTION) / (TRACTION_WHEEL_DIAMETER * Math.PI);
        int encoderValue = (int) (countsPerInch * distance);

        Log.d("Status" , "SCHSMotor:gyroDrive: encoder value " + encoderValue);

        double temp = countsPerInch * distance;

        slowFactor = (Math.abs(temp) + 309)/1550;
        Log.d("Status" , "SCHSMotor:gyroDrive: slowFactor " + slowFactor);

        // Determine new target position, and pass to motor controller
        newLeftTarget = motorLeft.getCurrentPosition() + encoderValue;
        newRightTarget = motorRight.getCurrentPosition() + encoderValue;
        Log.d("Status" , "SCHSMotor:gyroDrive: newLeftTarget " + newLeftTarget);
        Log.d("Status" , "SCHSMotor:gyroDrive: newRightTarget " + newRightTarget);

        // Set Target and Turn On RUN_TO_POSITION
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setTargetPosition(newLeftTarget);
        motorRight.setTargetPosition(newRightTarget);

        // start motion.
        speed = Range.clip(Math.abs(speed), -1.0, 1.0);
        motorLeft.setPower(speed);
        motorRight.setPower(speed);
        Log.d("Status" , "SCHSMotor:gyroDrive: speed " + speed);

        double PCoeff = 0.075;
        // keep looping while we are still active, and BOTH motors are running.
        while (motorLeft.isBusy() && motorRight.isBusy()) {

            Log.d("Status" , "SCHSMotor:gyroDrive:start of loop");

            //Log.d("Status", "SCHSMotor:gyroDrive:before gyro corrects left" + motorLeft.getCurrentPosition());
            //Log.d("Status", "SCHSMotor:gyroDrive:before gyro corrects right" + motorRight.getCurrentPosition());

            // adjust relative speed based on heading error.
            error = getError(currentAngle);
            Log.d("Status" , "SCHSMotor:gyroDrive: error " + error);

            steer = getSteer(error, PCoeff);
            Log.d("Status" , "SCHSMotor:gyroDrive: steer " + steer);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0) {
                steer *= -1.0;
            }

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            int distanceMovedLeft = motorLeft.getCurrentPosition();
            int distanceMovedRight = motorRight.getCurrentPosition();

            /*
            if (distanceMovedLeft >= slowFactor * Math.abs(newLeftTarget)|| distanceMovedRight >= slowFactor * Math.abs(newRightTarget)) {
                PCoeff = 0.75 * PCoeff;
                Log.d("Status" , "SCHSMotor:gyroDrive: PCoeff modified slowfactor " + PCoeff);
            }*/

            if (distanceMovedLeft >= slowFactor * Math.abs(newLeftTarget) || distanceMovedRight >= slowFactor * Math.abs(newRightTarget)) {
                speed = 0.75 * speed;
                Log.d("Status" , "SCHSMotor:gyroDrive: speed modified slowfactor " + speed);
            }

            // if reached the desired position, exit while loop. Helps to stop turning at end of motion.
            if (distanceMovedLeft >= Math.abs(newLeftTarget) || distanceMovedRight >= Math.abs(newRightTarget)) {
                Log.d("Status" , "SCHSMotor:gyroDrive: Position reached. Break while");
                break;
            }

            Log.d("Status" , "SCHSMotor:gyroDrive:leftSpeed " + leftSpeed);
            Log.d("Status" , "SCHSMotor:gyroDrive:rightSpeed " + rightSpeed);

            motorLeft.setPower(leftSpeed);
            motorRight.setPower(rightSpeed);

            Log.d("Status", "SCHSMotor:gyroDrive:after gyro corrects left " + motorLeft.getCurrentPosition());
            Log.d("Status", "SCHSMotor:gyroDrive:after gyro corrects right " + motorRight.getCurrentPosition());
        }
    }

    public double getError(double startAngle) {

        double robotError = 0;

        // calculate error in -179 to +180 range  (
        //robotError = (startAngle + targetAngle) - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        robotError = startAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        while (robotError > 180) {
            robotError -= 360;
        }

        while (robotError <= -180) {
            robotError += 360;
        }
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    //for turning the robot, put power on one motor, based on one side, not middle
    public void turnAtAngleEncoder(double powerTurn,int turnDirection,double turnAngle) {

        double countsPerInch = (CHMOTOR_COUNTS_PER_REVOLUTION) / (TRACTION_WHEEL_DIAMETER * Math.PI);
        Log.d("Status", "SCHSMotor:turnAtAngle:counts per inch" + countsPerInch);

        double distToTravel = (Math.PI) * (REAR_WHEEL_BASE_) * (turnAngle / 360);
        Log.d("Status", "SCHSMotor:turnAtAngle:distToTravel" + distToTravel);

        double temp = countsPerInch * distToTravel;
        Log.d("Status", "SCHSMotor:turnAtAngle:temp" + temp);

        int angleDistance = (int) temp;
        Log.d("Status", "SCHSMotor:turnAtAngle:angleDistance" + angleDistance);

        motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);

        while (motorLeft.getCurrentPosition() != 0 || motorRight.getCurrentPosition() != 0) { //Ensures encoders are zero
            motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
            //waitOneFullHardwareCycle(); //Needed within all loops
        }

        if (turnDirection == LEFT_TURN) {
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeft.setMode(RunMode.RUN_TO_POSITION);
            motorRight.setTargetPosition(angleDistance);
            motorLeft.setTargetPosition(angleDistance);
            motorRight.setPower(powerTurn);
            motorLeft.setPower(-powerTurn);

            while (motorRight.getCurrentPosition() < motorRight.getTargetPosition()) { //While target has not been reached
                //waitOneFullHardwareCycle(); //Needed within all loops
            }

            Log.d("Status", "SCHSMotor:turnAtAngle:left turn");

            motorRight.setPower(0);
        }

        if (turnDirection == RIGHT_TURN) {
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(RunMode.RUN_TO_POSITION);
            motorLeft.setTargetPosition(angleDistance);
            motorRight.setTargetPosition(angleDistance);
            motorLeft.setPower(powerTurn);
            motorRight.setPower(-powerTurn);

            while (motorLeft.getCurrentPosition() < motorLeft.getTargetPosition()) { //While target has not been reached
                //waitOneFullHardwareCycle(); //Needed within all loops
            }

            Log.d("Status", "SCHSMotor:turnAtAngle:right turn");

            motorLeft.setPower(0);
        }

    }

    public void turnWithGyro(double turnSpeed , double orgTurnAngle, double direction) {

        double currGyro = 0;
        double startGyro = 0;
        double turnAngle = Math.abs(orgTurnAngle);
        Log.d("Status ", "SCHSMotorGyro:turnWithGyro: turnAngle" + turnAngle);

        double slowFactor = (0.00475 * turnAngle) + 0.505;

        motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);

        while (motorLeft.getCurrentPosition() != 0 || motorRight.getCurrentPosition() != 0) { //Ensures encoders are zero
            motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
            //waitOneFullHardwareCycle(); //Needed within all loops
        }

        motorLeft.setMode(RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(RunMode.RUN_WITHOUT_ENCODER);

        startGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        Log.d("Status" , "SCHSMotor:turnWithGyro: startGyro" + startGyro);

        currGyro = startGyro;

        //left turn
        if(direction == 1) {
            Log.d("Status" , "SCHSMotor:turnWithGyro: entered left");
            while (currGyro < startGyro + turnAngle) {
                motorLeft.setPower(-turnSpeed);
                motorRight.setPower(turnSpeed);
                currGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                Log.d("Status" , "SCHSMotor:turnWithGyro: currGyro" + currGyro);

                if (turnAngle < 60) {
                    if (Math.abs(currGyro - startGyro) >= 0.8 * turnAngle) {
                        turnSpeed = 0.8 * turnSpeed;
                    }
                }

                Log.d("Status" , "SCHSMotor:turnWithGyro: turnSpeed" + turnSpeed);

                if (currGyro >= startGyro + turnAngle) {
                    Log.d("Status ","break from turn while");
                    break;
                }

            }

        } else if (direction == 2){ //right turn
            Log.d("Status" , "SCHSMotor:turnWithGyro: entered right");
            while (currGyro > startGyro - turnAngle) {
                motorLeft.setPower(turnSpeed);
                motorRight.setPower(-turnSpeed);
                currGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                Log.d("Status" , "SCHSMotor:turnWithGyro: currGyro" + currGyro);

                if (turnAngle < 60) {
                    if (Math.abs(currGyro - startGyro) >= 0.8 * turnAngle) {
                        turnSpeed = 0.8 * turnSpeed;
                    }
                }

                Log.d("Status" , "SCHSMotor:turnWithGyro: turnSpeed" + turnSpeed);

                if (currGyro <= startGyro - turnAngle) {
                    Log.d("Status ","break from turn while");
                    break;
                }
            }
        } else {
            Log.d("Status" , "SCHSMotor:turnWithGyro: WRONG TURN DIRECTION");
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);

    }

    public void moveServo(double position, int servoDirection) {
        if (servoDirection == SERVO_DIRECTION_LEFT){
            mascotServo.setDirection(Servo.Direction.REVERSE);
            Log.d("Status", "SCHSMotor:moveServo: set direction to reverse");
        } else {
            mascotServo.setDirection(Servo.Direction.FORWARD);
            Log.d("Status", "SCHSMotor:moveServo: set direction to forward");
        }

        double maxPosition = position;
        double currPosition = mascotServo.getPosition();
        Log.d("Status", "SCHSMotor:moveServo: currPosition" + currPosition);

        while (currPosition >= maxPosition) {
            Log.d("Status", "SCHSMotor:moveServo: entered while");
            maxPosition += INCREMENT;
            mascotServo.setPosition(maxPosition);
            Log.d("Status", "SCHSMotor:moveServo: max position" + maxPosition);
            sleep(CYCLE_MS);
            currPosition = mascotServo.getPosition();
            Log.d("Status", "SCHSMotor:moveServo: currPosition in loop" + currPosition);

            if (currPosition >= position){
                Log.d("Status", "SCHSMotor:moveServo: break");
                break;
            }
        }

    }

    public void moveLanderArm(double powerStart, int distance, int direction) {
        Log.d("Status" , "SCHSMotor:moveFromLander: inside moveLanderArm");

        int moveArmDist = 0;

        motorArm.setMode(RunMode.STOP_AND_RESET_ENCODER);
        while (motorArm.getCurrentPosition() != 0) { //Ensures encoders are zero
            motorArm.setMode(RunMode.STOP_AND_RESET_ENCODER);
            //waitOneFullHardwareCycle(); //Needed within all loops
        }

        motorArm.setMode(RunMode.RUN_TO_POSITION);

        if (direction == ARM_MOVE_DOWN_DIRECTION){
            moveArmDist = distance;

            motorArm.setTargetPosition(moveArmDist);

            while (motorArm.getCurrentPosition() <= moveArmDist) {
                motorArm.setPower(powerStart);
                Log.d("Status" , "SCHSMotor:moveFromLander: powerStart" + powerStart);

                /*if (motorArm.getCurrentPosition() >= 0.9 * moveArmDist) {
                    powerStart = 0.8 * powerStart;
                }*/

                if (motorArm.getCurrentPosition() >= moveArmDist) {
                    Log.d("Status" , "SCHSMotor:moveFromLander: break");
                    break;
                }
            }
        } else {
            moveArmDist = -distance;
            motorArm.setTargetPosition(moveArmDist);

            while (motorArm.getCurrentPosition() >= moveArmDist) {
                motorArm.setPower(powerStart);
                Log.d("Status" , "SCHSMotor:moveFromLander: powerStart" + powerStart);

                /*if (motorArm.getCurrentPosition() <= 0.9 * moveArmDist) {
                    powerStart = 0.8 * powerStart;
                }*/

                if (motorArm.getCurrentPosition() <= moveArmDist) {
                    Log.d("Status" , "SCHSMotor:moveFromLander: break");
                    break;
                }
            }
        }

        motorArm.setPower(0);

    }

    public double getCurrentAngle() {
        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return currentAngle;
    }
}

