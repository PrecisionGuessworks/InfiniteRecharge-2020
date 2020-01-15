/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * Add your docs here.
 */
public class TalonFXFactory {
    public static TalonFX createTalonFX(int id){ //Creates Talon with id
        TalonFX talon = new TalonFX(id);
        return talon;
    }

    public static TalonFX createTalonFX(int id, boolean inverted){ //Creates Talon with id and inversion
        TalonFX talon = new TalonFX(id);
        talon.setInverted(inverted);
        return talon;
    }

    public static TalonFX createPIDTalonFX(int id, double kP, double kI, double kD, double kF){
        TalonFX talon = new TalonFX(id);
        talon = new TalonFX(id);
        talon.configFactoryDefault();
        if(talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30) != null){
            System.out.println("ConfigSelectedFeedbackSensor failed");
        }
        talon.setSensorPhase(true);
        talon.configNominalOutputForward(0, 30);
        talon.configNominalOutputReverse(0, 30);
        talon.configPeakOutputForward(1, 30);
        talon.configPeakOutputReverse(-1, 30);
        talon.config_kP(0, kP, 30);
        talon.config_kI(0, kI, 30);
        talon.config_kD(0, kD, 30);
        talon.config_kF(0, kF, 30);
        return talon;
        
    }

    public static TalonFX createPIDTalonFX(int id, boolean inverted, double kP, double kI, double kD, double kF){
        TalonFX talon = createPIDTalonFX(id, kP, kI, kD, kF);
        talon.setInverted(inverted);
        return talon;
    }

    

    public static TalonFX createFollowerTalonFX(int id, TalonFX master){ //Creates a follower motor
        TalonFX followerTalon = new TalonFX(id);
        followerTalon.setInverted(master.getInverted()); //Sets inverted to the masters inverted
        followerTalon.follow(master);
        return followerTalon;
    }

}
