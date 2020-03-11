/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Add your docs here.
 */
public class FieldPositions {

    public static Pose2d ORIGIN = new Pose2d();
    public static Pose2d START = new Pose2d(0, 2.3, new Rotation2d());
    public static Pose2d OPPSHOT = new Pose2d(7.2, 7.9, new Rotation2d());
    
    public static Pose2d TRENCH_CELL_1 = new Pose2d(10.2, 2.3, new Rotation2d());
    public static Pose2d TRENCH_CELL_2 = new Pose2d(13.2, 2.3, new Rotation2d());
    public static Pose2d TRENCH_CELL_3 = new Pose2d(16.2, 2.3, new Rotation2d());

    public static Pose2d GEN_CELL_1 = new Pose2d(10.9, 9.5, new Rotation2d());
    public static Pose2d GEN_CELL_2 = new Pose2d(9.3, 10.1, new Rotation2d());
    public static Pose2d GEN_CELL_3 = new Pose2d(9.0, 12.1, new Rotation2d());
    public static Pose2d GEN_CELL_4 = new Pose2d(9.5, 13.4, new Rotation2d());
    public static Pose2d GEN_CELL_5 = new Pose2d(10.0, 14.7, new Rotation2d());

    public static Pose2d OPP_TRENCH_CELL_1 = new Pose2d(10.9, 23.8, new Rotation2d());

    public static double DIST_CENTER_TO_INTAKE = 1.625;

    public static Pose2d addRotation(Pose2d pos, Rotation2d rotation){
        return new Pose2d(pos.getTranslation(), rotation);
    }
    
    public static Pose2d pointAtIntake(Pose2d pos){
        double x = pos.getTranslation().getX();
        double y = pos.getTranslation().getY();
        double heading = pos.getRotation().getRadians();
        return new Pose2d(x - DIST_CENTER_TO_INTAKE*Math.cos(heading), y - DIST_CENTER_TO_INTAKE*Math.sin(heading), pos.getRotation());
    }

}
