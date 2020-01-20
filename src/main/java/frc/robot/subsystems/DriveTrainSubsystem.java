/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.TalonFXFactory;
import frc.robot.OI;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrainSubsystem implements Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  TalonFX[] leftmotor;
  TalonFX[] rightmotor;
  public OI oi;

  public static final double WHEEL_DIAMETER = 6.0;
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

  private static DriveTrainSubsystem instance;
  
  

  private DriveTrainSubsystem(){

    oi = OI.getInstance();
    
    //TODO: actually do something with our PIDF values
    double leftkP = 0.0;
    double leftkI = 0.0;
    double leftkD = 0.0;
    double leftkF = 1.0;
    
    double rightkP = 0.0;
    double rightkI = 0.0; 
    double rightkD = 0.0; 
    double rightkF = 1.0;

    leftmotor = new TalonFX[3];
    rightmotor = new TalonFX[3];

    leftmotor[0] = TalonFXFactory.createPIDTalonFX(RobotMap.DRIVETRAIN_LEFT_MASTER, true, leftkP, leftkI, leftkD, leftkF);
    leftmotor[1] = TalonFXFactory.createFollowerTalonFX(RobotMap.DRIVETRAIN_LEFT_FOLLOWER1, leftmotor[0]);
    leftmotor[2] = TalonFXFactory.createFollowerTalonFX(RobotMap.DRIVETRAIN_LEFT_FOLLOWER2, leftmotor[0]);

    rightmotor[0] = TalonFXFactory.createPIDTalonFX(RobotMap.DRIVETRAIN_RIGHT_MASTER, false, rightkP, rightkI, rightkD, rightkF);
    rightmotor[1] = TalonFXFactory.createFollowerTalonFX(RobotMap.DRIVETRAIN_RIGHT_FOLLOWER1, rightmotor[0]);
    rightmotor[2] = TalonFXFactory.createFollowerTalonFX(RobotMap.DRIVETRAIN_RIGHT_FOLLOWER2, rightmotor[0]);



    SmartDashboard.putNumber("Speed", 0.5);
    SmartDashboard.putNumber("Passcode", 0.0);

    SmartDashboard.putNumber("Encoder Counts", 0.0);

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

  public double getError(){
    return ((double) leftmotor[0].getClosedLoopError()) * 10.0 * Math.PI * 6.0 / 49152.0;
  }

  public double getDriveVelocity(){
      return (getEncoderCounts() * 10 * Math.PI * 6. / (49152 * 7.56)) * 2;
  }

  public double getEncoderCounts(){ //this is currently not working
    return leftmotor[0].getSensorCollection().getIntegratedSensorVelocity();
  }

  public void periodic(){
    double throttle = oi.getDriverThrottle();
    double turn = oi.getDriverTurn();
    setDrivePower(throttle-turn, throttle+turn);
    //setDriveVelocity((throttle-turn)*25, (throttle+turn)*25);
    System.out.println("throttle:" + throttle);
  }
}