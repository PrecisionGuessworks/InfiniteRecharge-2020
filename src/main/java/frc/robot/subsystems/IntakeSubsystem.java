/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   * Motors include: Hopper intake, hopper movement, lift intake and anti-jammer, lift to motor, and turret spinner
  */
  private VictorSPX intakeRoller;
  private VictorSPX intakeArmMotor;
  private VictorSPX hopperIndexer;
  private TalonSRX hopperAggitator;
  private VictorSPX liftMotor;

  private intakeStates currentState;

  public enum intakeStates {
    STAGING, //Feeding the balls until sensor sees balls
    FEEDING, //Loading balls into shooter
    OFF; //No motors are spinning
  }

  public static IntakeSubsystem instance;

  
  public IntakeSubsystem() {
    intakeRoller = new VictorSPX(RobotMap.INTAKE_INTAKE_ROLLER_ID);
    intakeArmMotor = new VictorSPX(RobotMap.INTAKE_INTAKE_ARM_MOTOR_ID);
    hopperIndexer = new VictorSPX(RobotMap.INTAKE_HOPPER_INDEXER_ID);
    hopperAggitator = new TalonSRX(RobotMap.INTAKE_HOPPER_AGGITATOR_ID);
    liftMotor = new VictorSPX(RobotMap.INTAKE_LIFT_MOTOR_ID);
    liftMotor.setInverted(true);
    hopperIndexer.setInverted(true);
  }

  public static IntakeSubsystem getInstance(){
    if(instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  public void setIntakePower(double power){
    intakeRoller.set(ControlMode.PercentOutput, power);
  }

  public void setIntakeArmPower(double power){
    intakeArmMotor.set(ControlMode.PercentOutput, power);
  }

  public void setIndexerPower(double power){
    hopperIndexer.set(ControlMode.PercentOutput, power);
  }

  public void setAggitatorPower(double power){    //this one is funkier than it seems?
    hopperAggitator.set(ControlMode.PercentOutput, power);
  }

  public void setLiftMotorPower(double power){
    liftMotor.set(ControlMode.PercentOutput, power);
  }
//TODO: LIMELIGHT THINGS
  public void setState(intakeStates newState){
    currentState = newState;
    //TODO: Do turret stuff
    switch (currentState) {
      case OFF:
        setIntakePower(0);
        break;

        case STAGING:
        setIntakePower(0.5);
        //TODO: ULTRASONIC STUFF
        break;

        case FEEDING:
        setIntakePower(0.5);
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
