package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestMotorClass extends LinearOpMode {

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;


    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        //names the motors for the configuration
        //change the names for the motors in the config to match the strings above

        waitForStart();
        // run until the end of the match (driver presses STOP)

        double powerLeft, powerRight;
        //this double will be used to control speed

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            //setting the power to the motors

            powerLeft = this.gamepad1.left_stick_y;
            powerRight = -this.gamepad1.right_stick_y;

            //"this" applies to the game pad

            leftMotor.setPower(powerLeft);
            rightMotor.setPower(powerRight);

            // these are negative because the stick in full bottom position returns a 1
            // and at the top returns -1
            // so we negate the result
            // therefore pushing the stick up will return 1 and move forward

            telemetry.addData("Target Power Left Motor", powerLeft);
            telemetry.addData("Target Power Right Motor", powerRight);
            telemetry.update();
        }
    }
}
