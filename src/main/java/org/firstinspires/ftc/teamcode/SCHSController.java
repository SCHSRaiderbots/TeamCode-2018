package org.firstinspires.ftc.teamcode;

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

@Autonomous(name="SCHSController", group="SCHS")
//@Disabled
public class SCHSController extends LinearOpMode {

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

        Log.d("Status" , "SCHSController:opModeIsActive: before deposit mascot");

        //riley.depositMascot();

        //riley.senseBallAndSample();

        riley.orientRobot();

        //cleanShutDown();

        // Show the elapsed game time
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        //}
    }
}
