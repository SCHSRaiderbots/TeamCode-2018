package org.firstinspires.ftc.teamcode;

public final class SCHSConstants {

    // distance measurements for each movement in inches
    static final int MOVE_FROM_LANDER_DIST = 120;
    static final int GO_TO_WALL_DIST = 24;
    static final int GO_TO_PICTURE_DIST = 36;
    static final int GO_TO_DEPOT_DIST = 12;
    static final int GO_TO_CRATER_DIST = 12;

    // angle measurements for each turn
    static final double TURN_TO_WALL_ANGLE = 180;
    static final double TURN_TO_PICTURE_ANGLE = 90;
    static final double DEPOT_ZONE_TURN_ANGLE = 90;
    static final double CRATER_ZONE_TURN_ANGLE = 90;

    //left or right direction for each turn - int where 1 = left and 2 = right
    static final int TURN_TO_WALL_DIRECTION = 1;
    static final int TURN_TO_PICTURE_DIRECTION = 2;
    static final int DEPOT_ZONE_TURN_DIRECTION = 2;
    static final int CRATER_ZONE_TURN_DIRECTION = 1;
    static final int LEFT_TURN = 1;
    static final int RIGHT_TURN = 2;

    // blue values as that the color sensor would detect from each picture - may be changed to red/green values
    static final int PIC_CRATERS = 5;
    static final int PIC_ROVER = 4;
    static final int PIC_MOON = 3;
    static final int PIC_STARS = 2;

    // power constants between 1 and -1
    static final double POWER_FULL_FORWARD = 1;
    static final double POWER_FULL_BACKWARD = -1;
    static final double POWER_HALF_FORWARD = 0.5;
    static final double POWER_HALF_BACKWARD = -0.5;

    //core hex motor constants
    static final double CHMOTOR_COUNTS_PER_REVOLUTION = 288;
    static final double REAR_WHEEL_BASE_= 12; //inches
    static final double TRACTION_WHEEL_DIAMETER = 90 * 0.0393701; //90mm converted to inches

    //formula for inches to counts (encoder value): 288 counts/revolution
    //1 revolution = 90pi mm = 3.54331pi inches
    //total counts = 288*rev
    //x inches / 3.54331pi = # rev
    //encoder value = (288*x)/(3.54331pi) = 310.466 at x = 12 inches

    //formula for degrees (encoder value): (a/360)*(3.54331pi) = y inches
    //encoder value = (288*y)/(12pi) = 310.466 at y inches for a degrees
    //at a = 90 degrees, encoder value = 243.839
}
