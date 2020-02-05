/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   * Motors include: Hopper intake, hopper movement, lift intake and anti-jammer, lift to motor, and turret spinner
  */
  private Talon intakeRoller;
  private Talon intakeArmMotor;
  private Talon hopperIndexer;
  private TalonSRX hopperAggitator;
  private Talon liftMotor;

  public enum intakeStates {
    STAGING, //Feeding the balls until sensor sees balls
    FEEDING, //Loading balls into shooter
    OFF; //No motors are spinning
  }

  public IntakeSubsystem() {
    intakeRoller = new Talon(RobotMap.INTAKE_INTAKE_ROLLER_ID);
    intakeArmMotor = new Talon(RobotMap.INTAKE_INTAKE_ARM_MOTOR_ID);
    hopperIndexer = new Talon(RobotMap.INTAKE_HOPPER_INDEXER_ID);
    hopperAggitator = new TalonSRX(RobotMap.INTAKE_HOPPER_AGGITATOR_ID);
    liftMotor = new Talon(RobotMap.INTAKE_LIFT_MOTOR_ID);
  }

  public void setIntakePower(double power){
    intakeRoller.set(power);
  }

  public void setIntakeArmPower(double power){
    intakeArmMotor.set(power);
  }

  public void setIndexerPower(double power){
    hopperIndexer.set(power);
  }

  public void setAggitatorPower(double power){    //this one is funkier than it seems?
    hopperAggitator.set(ControlMode.PercentOutput, power);
  }

  public void setLiftMotorPower(double power){
    liftMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
