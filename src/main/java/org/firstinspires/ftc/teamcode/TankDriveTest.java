package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//This program can be used to drive the robot like a tank.

@TeleOp
public class TankDriveTest extends LinearOpMode {

    private DcMotor leftMotor  = null; //config name: leftMotor
    private DcMotor rightMotor = null; //config name: rightMotor
    private DcMotor armMotor   = null; //config name: armMotor
    private DcMotor armMotor2  = null; //config name: armCoreMotor
    private DcMotor backMotor  = null; //config name: backMotor

    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor  = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor2 = hardwareMap.get(DcMotor.class, "armCoreMotor");
        backMotor = hardwareMap.get(DcMotor.class, "backMotor");

        //int armMotorPosition, targetPosition;
        //int backMotorPosition, backTargetPosition;

        //int timesChecked = 0;

        //names the motors for the configuration
        //change the names for the motors in the config to match the strings above

        waitForStart();
        // run until the end of the match (driver presses STOP)

        double powerLeft, powerRight, powerArm, powerArm2, backPower;
        //these doubles will be used to control speed.

        //boolean backMotorLimiter = false;

        //int lowerThreshold = -450, upperThreshold = 0;
        //Encoder thresholds to prevent arms from moving beyond a certain position


        int holdPosition; //filler value;

        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Resets armMotor's encoder position to 0.
        //Note that because of this the arm must be set fully upward before the position is reset.
        //Maybe at the end of the autonomous period we could include a provision to put the motor up all the way.

        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            //adjusting the motor power from game pad input.
            powerLeft = this.gamepad1.left_stick_y;
            powerRight = -this.gamepad1.right_stick_y;
            powerArm = -this.gamepad2.right_stick_y; //crane swivel part
            powerArm2 = -this.gamepad2.right_trigger; //extending part
            backPower = -this.gamepad1.right_trigger;

            //to retract the arm
            if (gamepad2.left_trigger >= 0 && gamepad2.right_trigger == 0) {
                powerArm2 = gamepad2.left_trigger;
            }

            if (gamepad1.left_trigger >= 0 && gamepad1.right_trigger == 0) { //move the motor the other way
                backPower = gamepad1.left_trigger;
            }

            /*if (gamepad1.left_bumper && timesChecked == 0) { //hit left button to fix winch position.
                backMotorLimiter = !backMotorLimiter;
                backMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                timesChecked++;
            }*/

            /*if (backMotorLimiter == true) {
                //determine target position based on current position
                backMotorPosition = backMotor.getCurrentPosition();
                backTargetPosition = backMotorPosition + (int)(powerArm*177);

                //range setter
                if (backTargetPosition >= upperThreshold) {
                    backTargetPosition = upperThreshold;
                }
                if (backTargetPosition <= lowerThreshold) {
                    backTargetPosition = lowerThreshold;
                }

            }
            */


            //armMotorPosition = armMotor.getCurrentPosition();
            //targetPosition = armMotorPosition + (int)(powerArm*500);

            //setting motor power.

            //actual arm algorithm
            if (powerArm == 0) {  //not optimal as it resets the mode every cycle
                holdPosition = armMotor.getCurrentPosition();
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setTargetPosition(holdPosition);
                armMotor.setPower(0.531); //random value because powerArm is 0
            } else {
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armMotor.setPower(powerArm/1.88);
            }


            leftMotor.setPower(powerLeft/2.2);
            rightMotor.setPower(powerRight/2.2);
            armMotor2.setPower(powerArm2);
            backMotor.setPower(backPower/1.7);


            //Range setter. If targetPosition is beyond a threshold, sets it to instead match the threshold.

            /*
            if (targetPosition >= upperThreshold) {
                targetPosition = upperThreshold;
            }
            if (targetPosition <= lowerThreshold) {
                targetPosition = lowerThreshold;
            }



            //"Algorithm" for arm's movement. Prevents arm from moving beyond certain points.
            //Assumes motor turns to the right when powerArm is positive and left when negative.
            //Add provision to adjust target position if at ANY point it exceeds threshold - in testing

            //NOTE: inserting gears between the motor and arm will necessitate the logic being "flipped"
            //the final threshold will likely be negative

            /*if (armMotorPosition <= lowerThreshold) {
                if (powerArm >= 0) {
                    armMotor.setTargetPosition(targetPosition);
                    armMotor.setPower(powerArm);
                }
            } else if (armMotorPosition >= upperThreshold) {
                if (powerArm <= 0) {
                    armMotor.setTargetPosition(targetPosition);
                    armMotor.setPower(powerArm);
                }
            } else {
                armMotor.setTargetPosition(targetPosition);
                armMotor.setPower(powerArm);
            }
            */

            telemetry.addData("Target Power Arm Motor", powerArm/1.7);
            telemetry.addData("Target Power Arm Core Motor", powerArm2);
            telemetry.addData("Target Power Back Motor", backPower);
            telemetry.addData("Blank space, encoders below", 777);
            //telemetry.addData("Theoretical Arm Position", targetPosition);
            telemetry.addData("Arm Motor Encoder Position", armMotor.getCurrentPosition());
            telemetry.addData("Back Motor encoder position", backMotor.getCurrentPosition());
            telemetry.update();

            //updating to the phone.
        }
    }
}



//modification 1/8/19 Added arm motor
//modifications begun on 1/9/19: adding provision for the armMotor to not move beyond certain point
// 1/9/19 encountered motor slippage issue. bracing to fix.
// 1/22/19 adding servo control. only one pertinent to the robot in Driver period is the hook one


//minus value = winding -- arm extension
//plus value = unwinding

//arm turning motor
// positive enc value - down towards ground, negative enc value - upward

/*logical deductions: (to be added)
lower threshold will be a large positive number
therefore the encoder should be <= lower threshold, and >= 0
this requires
 */

/*
back motor pseudocode

create boolean backmotorlimiter set to false. becomes true when some button is pressed
while it is false, do the standard setPower and target position to move the backwinch into the desired position
when the button is pressed, reset encoder and increment time checked. the time checked variable is so that it will not constantly reset encoder
then set the limiters to something like -65516
 */

/*
arm motor pseudocode

reeeeeee

if getPower = yields 0 yet the encoder value is changing, go to position
 */