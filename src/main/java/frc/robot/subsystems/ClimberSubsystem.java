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

public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimberSubsystem.
   */

   private TalonFX climbmotor;
  public enum climberStates{
    CLIMBING, //Allow climber full range of motion without locking
    HANGING, //Moves to locked and fully retracted
    STOWED; //Keeps the climber under 45 inches and not locked

  }

  public ClimberSubsystem() {
    //climbmotor = TalonFXFactory.createPIDTalonFX(RobotMap.CLIMB_MOTOR_ID, 0.0, 0.0, 0.0, 0.0);
  }

  public void setClimberPower(double pow){
    climbmotor.set(ControlMode.PercentOutput, pow);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
