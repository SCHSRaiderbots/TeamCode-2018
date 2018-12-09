package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.util.Log;

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

    public void moveToPosition(double powerStart , double powerStop, int position, HardwareMap hardwareMap) {

        Log.d("Status" , "inside moveToPosisition method");

        //motorLeft = hardwareMap.get(DcMotor.class, "leftMotor");
        //motorRight = hardwareMap.get(DcMotor.class, "rightMotor");
        //motorLeft.setDirection(Direction.FORWARD);
        //motorRight.setDirection(Direction.REVERSE);

        initialize(hardwareMap);

        // This defines the power level; between 1 and -1.

        //set the power of the motor
        motorLeft.setPower(powerStart);
        motorRight.setPower(powerStart);

        // set the motor position
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setTargetPosition(position);
        motorRight.setTargetPosition(position);

    }

    public void checkDist() {
        
    }

}
