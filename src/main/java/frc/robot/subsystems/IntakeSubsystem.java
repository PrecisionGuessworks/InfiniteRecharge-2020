/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
  */

  public enum intakeStates {
    STAGING, //Feeding the balls until sensor sees balls
    FEEDING, //Loading balls into shooter
    OFF; //No motors are spinning
  }

  private VictorSPX intakeMotor;

  public IntakeSubsystem() {
    intakeMotor = new VictorSPX(RobotMap.INTAKE_MOTOR_ID);
  }

  public void setIntakePower(double power){
    intakeMotor.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
