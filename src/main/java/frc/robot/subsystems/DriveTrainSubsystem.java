/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDrive;
import frc.robot.lib.TalonFXFactory;

/**
 * Add your docs here.
 */
public class DriveTrainSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  TalonFX[] leftmotor;
  TalonFX[] rightmotor;

  public static final double WHEEL_DIAMETER = 6.0;
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

  private static DriveTrainSubsystem instance;
  

  private DriveTrainSubsystem(){

    //TODO: actually do something with our PIDF values
    double leftkP = 0.0;
    double leftkI = 0.0;
    double leftkD = 0.0;
    double leftkF = 0.0;
    
    double rightkP = 0.0;
    double rightkI = 0.0; 
    double rightkD = 0.0; 
    double rightkF = 0.0;

    leftmotor = new TalonFX[3];
    rightmotor = new TalonFX[3];

    leftmotor[0] = TalonFXFactory.createPIDTalonFX(10, true, leftkP, leftkI, leftkD, leftkF);
    leftmotor[1] = TalonFXFactory.createFollowerTalonFX(11, leftmotor[0]);
    leftmotor[2] = TalonFXFactory.createFollowerTalonFX(12, leftmotor[0]);

    rightmotor[0] = TalonFXFactory.createPIDTalonFX(20, false, rightkP, rightkI, rightkD, rightkF);
    rightmotor[1] = TalonFXFactory.createFollowerTalonFX(21, rightmotor[0]);
    rightmotor[2] = TalonFXFactory.createFollowerTalonFX(22, rightmotor[0]);



    SmartDashboard.putNumber("Speed", 0.5);
    SmartDashboard.putNumber("Passcode", 0.0);


  }

  public static DriveTrainSubsystem getInstance(){
    if(instance==null){
      instance = new DriveTrainSubsystem();
    }
    return instance;
  }

  public void setDrivePower(final double powL, final double powR){

    if (SmartDashboard.getNumber("Passcode", 0.0) == 1539){
      leftmotor[0].set(ControlMode.PercentOutput, powL * SmartDashboard.getNumber("Speed", 0.5));
      rightmotor[0].set(ControlMode.PercentOutput, powR * SmartDashboard.getNumber("Speed", 0.5));
    }else{
      leftmotor[0].set(ControlMode.PercentOutput, powL * 0.5);
      rightmotor[0].set(ControlMode.PercentOutput, powR * 0.5);
    }
  }

  /**
   * Unit conversion - input targetVel ft/s * 1/10 s/100ms * 1/Circumfrence rev/ft * 4096 count/rev = count/100ms
   * @param targetVelL
   * @param targetVelR
   */
  public void setDriveVelocity(final double targetVelL, final double targetVelR){
    leftmotor[0].set(ControlMode.Velocity, targetVelL * (1./10) * (1./WHEEL_CIRCUMFERENCE) * 4096);
    rightmotor[0].set(ControlMode.Velocity, targetVelR * (1./10) * (1./WHEEL_CIRCUMFERENCE) * 4096);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ArcadeDrive());
  }
}
