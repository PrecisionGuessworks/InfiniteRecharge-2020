/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * Add your docs here.
 */
public class Constants {

    public static PIDConstants SHOOTER_LOWER_FLYWHEEL_CONSTANTS = new PIDConstants(0, 0, 0, 0);
    public static PIDConstants SHOOTER_UPPER_FLYWHEEL_CONSTANTS = new PIDConstants(0, 0, 0, 0);
    public static PIDConstants SHOOTER_TURRET_CONSTANTS = new PIDConstants(0, 0, 0, 0);

    public static PIDConstants DRIVETRAIN_LEFT = new PIDConstants(0, 0, 0, 0);
    public static PIDConstants DRIVETRAIN_RIGHT = new PIDConstants(0, 0, 0, 0);

    public static double SHOOTER_LONG_UPPER_WHEEL_SPEED = 3600;
    public static double SHOOTER_LONG_LOWER_WHEEL_SPEED = 3600;
    public static double SHOOTER_SHORT_UPPER_WHEEL_SPEED = 2700;
    public static double SHOOTER_SHORT_LOWER_WHEEL_SPEED = 2700;

    static class PIDConstants {

        public PIDConstants(double p, double i, double d, double f) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
        }
        double p;
        double i;
        double d;
        double f;
    }
}
