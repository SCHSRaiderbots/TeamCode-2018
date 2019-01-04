package org.firstinspires.ftc.teamcode;

import android.util.Log;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.bosch.BNO055IMU;

import static com.qualcomm.robotcore.hardware.DcMotor.*;

public class SCHSMotor extends SCHSController {

    private int encoderValue;
    private DcMotor motorLeft = null;
    private DcMotor motorRight = null;
    private DcMotor motorSingle = null;
    private HardwareMap myHWmap = null;
    private BNO055IMU imu = null;

    public void initialize(HardwareMap hardwareMap) {
        motorLeft = hardwareMap.get(DcMotor.class, "leftMotor");
        motorRight = hardwareMap.get(DcMotor.class, "rightMotor");
        motorLeft.setDirection(Direction.REVERSE);
        motorRight.setDirection(Direction.FORWARD);

        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();

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

    DcMotor getMotorLeft() {
        return motorLeft;
    }

    DcMotor getMotorRight() {
        return motorRight;
    }

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

        Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors:distance Moved Final Left before power zero" + motorLeft.getCurrentPosition());
        Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors:distance Moved Final Right before power zero" + motorRight.getCurrentPosition());


        motorLeft.setPower(0);
        motorRight.setPower(0);

        Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors:distance Moved Left after sleep Final" + motorLeft.getCurrentPosition());
        Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors:distance Moved Right after sleep Final" + motorRight.getCurrentPosition());

    }

    //moving robot with both motors
    public void verifyDistanceAndMoveTwoMotors(double powerStart, int desiredPosition) {

        boolean isMovingStraight = true;

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

        while (motorLeft.getCurrentPosition() < encoderValue || motorRight.getCurrentPosition() < encoderValue ) { //While target has not been reached

            if (motorLeft.getCurrentPosition() > motorRight.getCurrentPosition()) {
                motorLeft.setPower(motorPower);
                motorRight.setPower(motorPower - (motorPower * 0.15));

                Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors: right drift");
            }

            if (motorLeft.getCurrentPosition() < motorRight.getCurrentPosition()) {
                motorLeft.setPower(motorPower - (motorPower*0.15));
                motorRight.setPower(motorPower);

                Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors: left drift");
            }

            if (motorLeft.getCurrentPosition() == motorRight.getCurrentPosition()) {
                motorLeft.setPower(motorPower);
                motorRight.setPower(motorPower);
                Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors: no drift");
            }


            int distanceMovedLeft = motorLeft.getCurrentPosition();
            int distanceMovedRight = motorRight.getCurrentPosition();

            if (distanceMovedLeft >= 0.75 * encoderValue || distanceMovedRight >= 0.75 * encoderValue) {
                motorPower = 0.75 * motorPower;
            }

            Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors:distanceMovedLeft" + distanceMovedLeft);
            Log.d("Status", "SCHSMotor:verifyDistanceAndMoveTwoMotors:distanceMovedRight" + distanceMovedRight);
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

    public void moveStraightWithGyro(double powerStart, int desiredPosition) {
        //sensorGyro.calibrate();

        motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);

        while (motorLeft.getCurrentPosition() != 0 || motorRight.getCurrentPosition() != 0) { //Ensures encoders are zero
            motorLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
            //waitOneFullHardwareCycle(); //Needed within all loops
        }

        Log.d("Status" , "SCHSMotor:moveStraightWithGyro: reset encoders");

        motorLeft.setMode(RunMode.RUN_USING_ENCODER);
        motorRight.setMode(RunMode.RUN_USING_ENCODER);

        //sensorGyro.resetZAxisIntegrator();
        //Log.d("Status" , "SCHSMotor:moveStraightWithGyro: reset z axis integrator");

        gyroDrive(powerStart, desiredPosition, 0.0); //desired position in inches
        Log.d("Status" , "SCHSMotor:moveStraightWithGyro: gyroDrive finished");

    }

    public void gyroDrive(double speed, double distance, double angle) {
        int     newLeftTarget;
        int     newRightTarget;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        //converting inches to encoder values
        double countsPerInch = (CHMOTOR_COUNTS_PER_REVOLUTION) / (TRACTION_WHEEL_DIAMETER * Math.PI);
        double temp = countsPerInch * distance;
        int encoderValue = (int) temp;

        Log.d("Status" , "SCHSMotor:gyroDrive: encoder value" + encoderValue);

        // Determine new target position, and pass to motor controller
        newLeftTarget = motorLeft.getCurrentPosition() + encoderValue;
        newRightTarget = motorRight.getCurrentPosition() + encoderValue;
        Log.d("Status" , "SCHSMotor:gyroDrive: newLeftTarget" + newLeftTarget);
        Log.d("Status" , "SCHSMotor:gyroDrive: newRightTarget" + newRightTarget);

        // Set Target and Turn On RUN_TO_POSITION
        motorLeft.setTargetPosition(newLeftTarget);
        motorRight.setTargetPosition(newRightTarget);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        motorLeft.setPower(speed);
        motorRight.setPower(speed);
        Log.d("Status" , "SCHSMotor:gyroDrive: speed" + speed);

        // keep looping while we are still active, and BOTH motors are running.
        while (motorLeft.isBusy() && motorRight.isBusy()) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, 0.01);
            Log.d("Status" , "SCHSMotor:gyroDrive: error" + error);
            Log.d("Status" , "SCHSMotor:gyroDrive: steer" + steer);

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

            motorLeft.setPower(leftSpeed);
            motorRight.setPower(rightSpeed);

        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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

    //for turning the robot, put power on one motor
    public void turnAtAngle(double powerTurn,int turnDirection,double turnAngle) {

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
}

