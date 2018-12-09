package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.*;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;

@TeleOp(name="SCHSController", group="SCHS")
//@Disabled
public class SCHSController extends LinearOpMode {

    private Raiderbot riley = null;
    private SCHSTimer botTimer = null;
    private SCHSLocation botLocation = null;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public void initialize() {
        riley = new Raiderbot();
        botTimer = new SCHSTimer();
        botLocation = new SCHSLocation();

        riley.dropFromLander();
        riley.orientRobot();
        riley.depositMascot();

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the motor (same as name on phone)
        //motorLeft = hardwareMap.get(DcMotor.class, "leftMotor");
        //motorRight = hardwareMap.get(DcMotor.class, "rightMotor");

        // set forward direction
        //motorLeft.setDirection(Direction.FORWARD);
        //motorRight.setDirection(Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            Log.d("Status" , "inside runOpMode after start");

            riley.depositMascot(hardwareMap);

            //riley.goToCrater(hardwareMap);

            /*// Setup a variable save power level for telemetry
            double powerStart;
            double powerStop;

            // This defines the power level; between 1 and -1.
            powerStart = 0.5;
            powerStop = 0;

            //set the power of the motor
            motorLeft.setPower(powerStart);
            motorRight.setPower(powerStart);

            // set the motor position
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int position = 100;
            motorLeft.setTargetPosition(position);
            motorRight.setTargetPosition(position);

            // stop motor
            //motorLeft.setPower(powerStop);
            //motorRight.setPower(powerStop);
*/
            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
