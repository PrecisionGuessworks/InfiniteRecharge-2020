/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Add your docs here.
 */
public class TalonSRXFactory {

    //Constants for configuring the current (as in amps) draw limits for the motor
    static boolean enable = true;
    static double currentLimit = 8;
    static double thresholdCurrent = currentLimit;
    static double thresholdTime = 0;
    static SupplyCurrentLimitConfiguration currLim = new SupplyCurrentLimitConfiguration(enable, currentLimit, thresholdCurrent, thresholdTime);


    public static TalonSRX createTalonSRX(int id){ //Creates Talon with id
        TalonSRX talon = new TalonSRX(id);
        //talon.configSupplyCurrentLimit(currLim);
        return talon;
    }

    public static TalonSRX createTalonSRX(int id, boolean inverted){ //Creates Talon with id and inversion
        TalonSRX talon = new TalonSRX(id);
        talon.setInverted(inverted);
        //talon.configSupplyCurrentLimit(currLim);
        return talon;
    }

    public static TalonSRX createPIDTalonSRX(int id, double kP, double kI, double kD, double kF, FeedbackDevice encoderType){
        TalonSRX talon = new TalonSRX(id);
        talon = new TalonSRX(id);
        talon.configFactoryDefault();
        if(talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30) != null){
            System.out.println("ConfigSelectedFeedbackSensor failed");
        }
        talon.setSensorPhase(false);
        talon.configNominalOutputForward(0, 30);
        talon.configNominalOutputReverse(0, 30);
        talon.configPeakOutputForward(1, 30);
        talon.configPeakOutputReverse(-1, 30);
        talon.config_kP(0, kP, 30);
        talon.config_kI(0, kI, 30);
        talon.config_kD(0, kD, 30);
        talon.config_kF(0, kF, 30);
        //talon.configSupplyCurrentLimit(currLim);
        return talon;
        
    }

    public static TalonSRX createPIDTalonSRX(int id, boolean inverted, double kP, double kI, double kD, double kF, FeedbackDevice encoderType){
        TalonSRX talon = createPIDTalonSRX(id, kP, kI, kD, kF, encoderType);
        talon.setInverted(inverted);
        //talon.configSupplyCurrentLimit(currLim);
        return talon;
    }

    

    public static TalonSRX createFollowerTalonSRX(int id, TalonFX master) { // Creates a follower motor
        TalonSRX followerTalon = new TalonSRX(id);
        followerTalon.setInverted(master.getInverted()); //Sets inverted to the masters inverted
        followerTalon.follow(master);
        followerTalon.configSupplyCurrentLimit(currLim);
        return followerTalon;
    }

}
