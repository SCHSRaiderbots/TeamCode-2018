package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

public class SCHSPicVuforia {

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    private VuforiaLocalizer.Parameters parametersPic;
    private VuforiaLocalizer vuforiaPic;
    private VuforiaTrackables targetsRoverRuckus;
    private VuforiaTrackable blueRover;
    private VuforiaTrackable redFootprint;
    private VuforiaTrackable frontCraters;
    private VuforiaTrackable backSpace;
    private List<VuforiaTrackable> allTrackables;

    private OpenGLMatrix blueRoverLocationOnField;
    private OpenGLMatrix redFootprintLocationOnField;
    private OpenGLMatrix frontCratersLocationOnField;
    private OpenGLMatrix backSpaceLocationOnField;

    private OpenGLMatrix phoneLocationOnRobot;

    private String pictureDetected;

    public void initialize(HardwareMap hardwareMap , VuforiaLocalizer vuforiaFromObjectDetection, VuforiaLocalizer.Parameters paramsFromObjectDetection) {

        parametersPic = paramsFromObjectDetection;

        //  Instantiate the Vuforia engine
        vuforiaPic = vuforiaFromObjectDetection;


        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsRoverRuckus = this.vuforiaPic.loadTrackablesFromAsset("RoverRuckus");
        blueRover = targetsRoverRuckus.get(0);
        blueRover.setName(ROVER_PIC);
        redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName(FOOT_PIC);
        frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName(CRATERS_PIC);
        backSpace = targetsRoverRuckus.get(3);
        backSpace.setName(SPACE_PIC);

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parametersPic.cameraDirection);
        }

    }

    public void trackPictures() {
        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();

        // check all the trackable target to see which one (if any) is visible.
        targetVisible = false;

        long startTime = System.currentTimeMillis();
        CameraDevice.getInstance().setFlashTorchMode(true);

        while (false || (System.currentTimeMillis() - startTime) < SCAN_PICTURE_TIME) {
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    Log.d("Status", "SCHSPicVuforia:Visible Target " + trackable.getName());
                    pictureDetected = trackable.getName();
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    Log.d("Status", "SCHSPicVuforia:break from for");
                    break;
                } // end of if
            } // end of for

            if (targetVisible == true) {
                Log.d("Status", "SCHSPicVuforia:break from while");
                break;
            } // end of if

        } //end of while

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            Log.d("Status ","SCHSPicVuforia: Pos (in)" + "{X, Y, Z} = %.1f, %.1f, %.1f " +
                    translation.get(0) / mmPerInch + "," + translation.get(1) / mmPerInch + "," + translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            Log.d("Status" , "SCHSPicVuforia: Rot (deg)" + "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f " + rotation.firstAngle + "," + rotation.secondAngle + "," + rotation.thirdAngle);
        }
        else {
            Log.d("Status" , "SCHSPicVuforia: Visible Target: none");
        }

        CameraDevice.getInstance().setFlashTorchMode(false);

    }// end of trackPicture

    public String getPictureDetected () {
        return pictureDetected;
    }

} //end of class
