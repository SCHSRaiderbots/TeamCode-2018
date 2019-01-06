package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Vuforia;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class SCHSObjectDetection {

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private int tfodMonitorViewId;
    private int mineralAngle = 0;
    private int mineralDist = 0;

    public void initialize(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    }

    public void detectGoldMineral() throws InterruptedException {
        Log.d("Status", "SCHSObjectDetection: inside detectGoldMineral");

        initVuforia();

        Log.d("Status", "SCHSObjectDetection: after initVuforia");

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            Log.d("Status:", "Sorry! This device is not compatible with TFOD");
        }

        Log.d("Status", "SCHSObjectDetection: after initTfod");

        if (tfod != null) {
            tfod.activate();
            Log.d("Status", "SCHSObjectDetection: after tfod.activate()");
            //wait(10000);
            sleep(1000);
            Log.d("Status", "SCHSObjectDetection: after sleep");

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            long startTime = System.currentTimeMillis();
            while(false||(System.currentTimeMillis()-startTime)<10000) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    Log.d("Status", "# Object Detected " + updatedRecognitions.size());

                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        //int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();

                                mineralAngle = (int)(recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                mineralDist = calcImageDist((int)(recognition.getHeight()));

                                /*Log.d("Status", "SCHSObjectDetection: Gold Angle value " + (int) recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                Log.d("Status", "SCHSObjectDetection: Gold Left value " + (int) recognition.getLeft());
                                Log.d("Status", "SCHSObjectDetection: Gold Right value " + (int) recognition.getRight());
                                Log.d("Status", "SCHSObjectDetection: Gold Top value " + (int) recognition.getTop());
                                Log.d("Status", "SCHSObjectDetection: Gold Bottom value " + (int) recognition.getBottom());
                                Log.d("Status", "SCHSObjectDetection: Gold Height value " + (int) recognition.getHeight());
                                Log.d("Status", "SCHSObjectDetection: Gold Width value " + (int) recognition.getWidth());
                                Log.d("Status", "SCHSObjectDetection: Gold Image Height value " + (int) recognition.getImageHeight());
                                Log.d("Status", "SCHSObjectDetection: Gold Image Width value " + (int) recognition.getImageWidth());
                                Log.d("Status", "SCHSObjectDetection: Distance Inches value " + calcImageDist((int)(recognition.getHeight())));*/

                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();

                                mineralAngle = (int)(recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                mineralDist = calcImageDist((int)(recognition.getHeight()));

                                /*Log.d("Status", "SCHSObjectDetection: Silver Angle value " + (int) recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                Log.d("Status", "SCHSObjectDetection: Silver Left value " + (int) recognition.getLeft());
                                Log.d("Status", "SCHSObjectDetection: Silver Right value " + (int) recognition.getRight());
                                Log.d("Status", "SCHSObjectDetection: Silver Top value " + (int) recognition.getTop());
                                Log.d("Status", "SCHSObjectDetection: Silver Bottom value " + (int) recognition.getBottom());
                                Log.d("Status", "SCHSObjectDetection: Silver Height value " + (int) recognition.getHeight());
                                Log.d("Status", "SCHSObjectDetection: Silver Width value " + (int) recognition.getWidth());
                                Log.d("Status", "SCHSObjectDetection: Silver Image Height value " + (int) recognition.getImageHeight());
                                Log.d("Status", "SCHSObjectDetection: Silver Image Width value " + (int) recognition.getImageWidth());
                                Log.d("Status", "SCHSObjectDetection: Distance Inches value " + calcImageDist((int)(recognition.getHeight())));*/

                            } else {
                                // silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 /*&& silverMineral2X != -1*/) {
                            if (goldMineralX < silverMineral1X /*&& goldMineralX < silverMineral2X*/) {
                                Log.d("Status", "SCHSObjectDetection: Gold Mineral Position: Left");
                            } else if (goldMineralX > silverMineral1X /*&& goldMineralX > silverMineral2X*/) {
                                Log.d("Status", "SCHSObjectDetection: Gold Mineral Position: Right");
                            } else {
                                Log.d("Status", "SCHSObjectDetection: Gold Mineral Position: Center");
                            }
                        }
                    }
                } //end of while
            }
        }


    }

    public int getMineralAngle() {
        return mineralAngle;
    }

    public int getMineralDist() {
        return mineralDist;
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        Log.d("Status", "SCHSObjectDetection: inside initVuforia");

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod() {
        Log.d("Status", "SCHSObjectDetection: inside initTfod");

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public int calcImageDist(int objectHeight) {
        int mmDist = (FOCAL_LENGTH * REAL_HEIGHT_GOLD * IMG_HEIGHT) / (CAMERA_HEIGHT * objectHeight);
        int inchDist = (int) (0.0393701 * mmDist);
        return inchDist;
    }

}
