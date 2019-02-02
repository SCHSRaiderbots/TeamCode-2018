package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;


import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.*;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

@Autonomous(name="SCHSControllerRIGHT", group="SCHS")
//@Disabled
public class SCHSControllerRIGHT extends LinearOpMode  {

    private Raiderbot riley = null;
    private SCHSTimer botTimer = null;
    private SCHSLocation botLocation = null;
    private boolean isInitialized = false;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public void initialize() {
        riley = new Raiderbot();
        botTimer = new SCHSTimer();
        botLocation = new SCHSLocation();

        Log.d("Status" , "Controller:initialize: before riley initialized");

        riley.initialize(hardwareMap);
        isInitialized = true;

    }

    public void cleanShutDown() {
        riley.cleanShutDown();
    }

    @Override
    public void runOpMode() throws InterruptedException {


        if (isInitialized != true) {
            initialize();
            Log.d("Status" , "SCHSController:runOpMode: inside initialized runopmode");
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        Log.d("Status" , "SCHSController:runOpMode: program started");

        // run until the end of the match (driver presses STOP)
        //while (opModeIsActive()) {

        Log.d("Status" , "SCHSController:opModeIsActive: before senseBallAndSample");

        //testing move forward and backward - works
        //riley.testFunction();

        //riley.dropFromLander();

        //move towards balls by x inches (if necessary) - yes
        //scan 3 balls, determine gold mineral - yes
        //turn to gold - yes
        //move to gold - yes
        //return back - yes
        //turn back to 0 - yes
        riley.senseBallAndSample();
        Log.d("Status" , "SCHSController:opModeIsActive: after sense ball and sample");

        riley.scanPictures(RIGHT_TURN);
        Log.d("Status" , "SCHSController:opModeIsActive: after scanPictures");

        //013019 - reduced from 2000
        sleep(500);

        riley.depositMascot();
        Log.d("Status" , "SCHSController:opModeIsActive: after deposit mascot");

        //013019 - reduced from 2000
        sleep(1000);

        riley.goToCrater();
        Log.d("Status" , "SCHSController:opModeIsActive: after goToCrater");

        //cleanShutDown();

        // Show the elapsed game time
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        //}

    }
}
