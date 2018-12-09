package org.firstinspires.ftc.teamcode;

public final class SCHSConstants {

    // distance measurements for each movement
    static final int MOVE_FROM_LANDER_DIST= 100;
    static final int goToWallDist = 10;
    static final int goToPictureDist = 10;
    static final int goToDepotDist = 10;
    static final int goToCraterDist = 10;

    // angle measurements for each turn
    static final int turnToWallAngle = 90;
    static final int turnToPictureAngle = 90;
    static final int depotZoneTurnAngle = 90;
    static final int craterZoneTurnAngle = 90;

    //left or right direction for each turn - may be changed to int where 1 = left and 2 = right
    static final String turnToWallDirection = "Left";
    static final String turnToPictureDirection = "Right";
    static final String depotZoneTurnDirection = "Right";
    static final String craterZoneTurnDirection = "Left";

    // blue values as that the color sensor would detect from each picture - may be changed to red/green values
    static final int picCraters = 5;
    static final int picRover = 4;
    static final int picMoon = 3;
    static final int picStars = 2;

    // power constants
    static final double POWER_FULL_FORWARD = 1;
    static final double POWER_FULL_BACKWARD = -1;
    static final double POWER_HALF_FORWARD = 0.5;
    static final double POWER_HALF_BACKWARD = -0.5;
}
