package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.SCHSConstants.SERVO_DIRECTION_LEFT;
import static org.firstinspires.ftc.teamcode.SCHSConstants.SERVO_DIRECTION_RIGHT;
import static org.firstinspires.ftc.teamcode.SCHSConstants.TURN_MASCOT_SERVO_ANGLE;

//This program can be used to drive the robot like a tank.

@TeleOp
public class TankDriveTestPrecisionMascot extends LinearOpMode {

    private DcMotor leftMotor = null; //config name: leftMotor
    private DcMotor rightMotor = null; //config name: rightMotor
    private DcMotor armMotor = null; //config name: armMotor
    private DcMotor armCoreMotor = null; //config name: armCoreMotor
    private DcMotor backMotor = null; //config name: backMotor
    private Servo mascotServo = null;
    private SCHSMotor mascotControl = null;

    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "mineralTurnArm");
        armCoreMotor = hardwareMap.get(DcMotor.class, "mineralExtendArm");
        backMotor = hardwareMap.get(DcMotor.class, "landerArmMotor");
        mascotServo = hardwareMap.servo.get("mascotServo");

        //mascotControl = new SCHSMotor();
        //mascotControl.initialize(hardwareMap);

        //backMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //backMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //uncomment the above to test the encoder values

        //int armMotorPosition, targetPosition;

        //names the motors for the configuration
        //change the names for the motors in the config to match the strings above

        waitForStart();
        // run until the end of the match (driver presses STOP)

        double powerLeft, powerRight, powerArm, powerArm2, backPower;
        //these doubles will be used to control speed.

        //boolean currentStateA = false, currentStateB = false;

        int holdPosition = 0; //placeholder
        int timesChecked = 0;

        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Resets armMotor's encoder position to 0.
        //Note that because of this the arm must be set fully upward before the position is reset.
        //Maybe at the end of the autonomous period we could include a provision to put the motor up all the way.

        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            //adjusting the motor power from game pad input.

            powerLeft = (this.gamepad1.left_stick_y)/2.0;
            powerRight = (-this.gamepad1.right_stick_y)/2.0;
            powerArm = -this.gamepad2.right_stick_y; //crane part. arbitrary number
            powerArm2 = -this.gamepad2.right_trigger/3; //extending part. arbitrary number

            if (gamepad2.left_trigger >= 0 && gamepad2.right_trigger == 0) //to retract the arm
                powerArm2 = gamepad2.left_trigger/5;

            backPower = -this.gamepad1.right_trigger; //back arm winch part

            if (gamepad1.left_trigger >= 0 && gamepad1.right_trigger == 0) {
                backPower = gamepad1.left_trigger;
            }


            //armMotorPosition = armMotor.getCurrentPosition();
            //targetPosition = armMotorPosition + (int)(powerArm*500);

            //setting motor power.

            if (gamepad1.right_bumper) { //turbo button - when the button is pressed on the driver gamepad, full speed
                powerLeft = powerLeft * 2.0;
                powerRight = powerRight * 2.0;
            }


            if (powerArm == 0) {  //not optimal as it resets the mode every cycle
                if (timesChecked == 0) {
                    holdPosition = armMotor.getCurrentPosition();
                    timesChecked++;
                }
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setTargetPosition(holdPosition);
                armMotor.setPower(0.4); //random value because powerArm is 0
            } else {
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (powerArm < 0) { //faster up, slower down
                    armMotor.setPower(powerArm/2.56);
                }
                if (powerArm > 0)
                    armMotor.setPower(powerArm/1.77);
                timesChecked = 0;
            }

            //problem: shaking



            //power locker
            if (gamepad1.a) { //lock the robot into going straight by sending the same power to both wheels
                leftMotor.setPower(powerLeft);
                rightMotor.setPower(-powerLeft);
            }
            else if (gamepad1.b) { //robot will make minute turning adjustments based on left joystick input.
                //not intended for use longer than a few seconds.
                //only capable of tank type turning.
                leftMotor.setPower(powerLeft/2.77);
                rightMotor.setPower(powerLeft/2.77);
            } else {
                leftMotor.setPower(powerLeft);
                rightMotor.setPower(powerRight);
            }

            armCoreMotor.setPower(powerArm2);
            backMotor.setPower(backPower);

            /*// Move Mascot servo using A and B on arms controller
            if (gamepad2.a) {
                mascotControl.moveServo(TURN_MASCOT_SERVO_ANGLE, SERVO_DIRECTION_LEFT);
            } else if (gamepad2.b) {
                mascotControl.moveServo(-TURN_MASCOT_SERVO_ANGLE, SERVO_DIRECTION_LEFT);
            } else {
                telemetry.addData("TankDriveTestPrecisionMascot: OH NO! Wrong Servo Move!", 0);
            }*/

            if (gamepad2.a) {
                mascotServo.setPosition(0.5);
            } else if (gamepad2.b) {
                mascotServo.setPosition(0.9);
            } else {
                telemetry.addData("TankDriveTestPrecisionMascot: OH NO! Wrong Servo Move!", 0);
            }

            //Range setter. If targetPosition is beyond threshold, sets it to instead match the threshold.

            /*
            if (targetPosition >= upperThreshold) {
                targetPosition = upperThreshold;
            }
            if (targetPosition <= lowerThreshold) {
                targetPosition = lowerThreshold;
            }
            */


            //"Algorithm" for arm's movement. Prevents arm from moving beyond certain points.
            //Assumes motor turns to the right when powerArm is positive and left when negative.
            //Add provision to adjust target position if at ANY point it exceeds threshold - in testing

            //NOTE: inserting gears between the motor and arm will necessitate the logic being "flipped"
            //the final threshold will likely be negative

            //*if (armMotorPosition <= lowerThreshold) {
            //    if (powerArm >= 0) {
            //      armMotor.setTargetPosition(targetPosition);
            //    armMotor.setPower(powerArm);
            //}
            //} else if (armMotorPosition >= upperThreshold) {
            //  if (powerArm <= 0) {
            //    armMotor.setTargetPosition(targetPosition);
            //  armMotor.setPower(powerArm);
            //}
            //} else {
            //  armMotor.setTargetPosition(targetPosition);
            //armMotor.setPower(powerArm);
            //


            telemetry.addData("Target Power Arm Motor", powerArm);
            telemetry.addData("Target Power Back Motor", backPower);
            telemetry.addData("BackMotorEncoder", backMotor.getCurrentPosition());
            telemetry.update();

            //updating to the phone.
        }
    }
}


//modification 1/8/19 Added arm motor
//modifications begun on 1/9/19: adding provision for the armMotor to not move beyond certain point
// 1/9/19 encountered motor slippage issue. bracing to fix.
// 1/22/19 adding servo control. only one pertinent to the robot in Driver period is the hook one


//minus value = winding
//plus value = unwinding
