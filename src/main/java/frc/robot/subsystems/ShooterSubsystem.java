/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.lib.TalonFXFactory;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
  */

  private TalonFX upperFlywheel;
  private TalonFX lowerFlywheel;

  public static ShooterSubsystem instance;

  private ShooterSubsystem() {
    upperFlywheel = TalonFXFactory.createPIDTalonFX(RobotMap.UPPER_FLYWHEEL_ID, false, 0.0, 0.0, 0.0, 0.0);
    lowerFlywheel = TalonFXFactory.createPIDTalonFX(RobotMap.LOWER_FLYWHEEL_ID, true, 0.0, 0.0, 0.0, 0.0);
  }

  public static ShooterSubsystem getInstance(){
    if(instance==null){
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  public void setShooterPower(double upperPower, double lowerPower){
    upperFlywheel.set(ControlMode.PercentOutput, upperPower);
    lowerFlywheel.set(ControlMode.PercentOutput, lowerPower);
  }

  public void setShooterSpeed(double upperSpeed, double lowerSpeed){
    double upperSpeedConverted = upperSpeed;
    double lowerSpeedConverted = lowerSpeed;    
    upperFlywheel.set(ControlMode.Velocity, upperSpeedConverted);
    lowerFlywheel.set(ControlMode.Velocity, lowerSpeedConverted);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
