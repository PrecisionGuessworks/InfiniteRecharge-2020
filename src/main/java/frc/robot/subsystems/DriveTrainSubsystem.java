/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.spline.Spline.ControlVector;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import frc.robot.commands.ArcadeDrive;

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

  Trajectory testTraj;

  public static final double WHEEL_DIAMETER = 6.0;
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

  /*
  fps x 1s/10hms x 12 in/1 ft x 1 wrev/ 6pi in x 7.56 mrev/ 1 wrev x 2048 counts/ 1 mrev
  hms = hundred milliseconds wrev = wheel rev mrev = motor rev
  */

  public static final double FPS_TO_CP100MS = (1./10) * 12 * (1./WHEEL_CIRCUMFERENCE) * 2048 * 7.56;
  public static final double MAX_VEL = 20; //fps 
  public static final double KV = 1/MAX_VEL;
  public static final double COUNTS_TO_FEET = (1/12.0) * WHEEL_CIRCUMFERENCE * (1/2048.0) * (1/7.56); 
  private static DriveTrainSubsystem instance;
  
  

  private DriveTrainSubsystem(){

    oi = OI.getInstance();
    
    //TODO: actually do something with our PIDF values
    double leftkP = 0.12;
    double leftkI = 0.0;
    double leftkD = 0.0;
    double leftkF = 0.0;
    
    double rightkP = 0.12;
    double rightkI = 0.0; 
    double rightkD = 0.0; 
    double rightkF = 0.0;

    

    leftmotor = new TalonFX[3];
    rightmotor = new TalonFX[3];

    leftmotor[0] = TalonFXFactory.createPIDTalonFX(RobotMap.DRIVETRAIN_LEFT_MASTER, false, leftkP, leftkI, leftkD, leftkF);
    leftmotor[1] = TalonFXFactory.createFollowerTalonFX(RobotMap.DRIVETRAIN_LEFT_FOLLOWER1, leftmotor[0]);
    leftmotor[2] = TalonFXFactory.createFollowerTalonFX(RobotMap.DRIVETRAIN_LEFT_FOLLOWER2, leftmotor[0]);

    rightmotor[0] = TalonFXFactory.createPIDTalonFX(RobotMap.DRIVETRAIN_RIGHT_MASTER, true, rightkP, rightkI, rightkD, rightkF);
    rightmotor[1] = TalonFXFactory.createFollowerTalonFX(RobotMap.DRIVETRAIN_RIGHT_FOLLOWER1, rightmotor[0]);
    rightmotor[2] = TalonFXFactory.createFollowerTalonFX(RobotMap.DRIVETRAIN_RIGHT_FOLLOWER2, rightmotor[0]);


    SmartDashboard.putNumber("Speed", 0.5);
    SmartDashboard.putNumber("Passcode", 0.0);

    SmartDashboard.putNumber("Encoder Counts", 0.0);

    

    //testTraj = TrajectoryGenerator.generateTrajectory(new Pose2d(), null, new Pose2d(0, 6.096, null), new TrajectoryConfig(6.096, 7.315));
    ArrayList<Pose2d> pointList = new ArrayList<>();
    pointList.add(new Pose2d());
    pointList.add(new Pose2d(0, 5, new Rotation2d()));
    pointList.add(new Pose2d(0, 10, new Rotation2d()));

    //pointList.add(new Pose2d(0, 3.048, new Rotation2d()));
    //pointList.add(new Pose2d(0, 6.096, new Rotation2d()));

   // testTraj = TrajectoryGenerator.generateTrajectory(pointList, new TrajectoryConfig(3.048, 3.657));
   testTraj = TrajectoryGenerator.generateTrajectory(pointList, new TrajectoryConfig(5, 24));



  }

  


  

public static DriveTrainSubsystem getInstance(){
    if(instance==null){
      instance = new DriveTrainSubsystem();
    }
    return instance;
  }

  public double setSpeedbyTrajectory(double time){

  double velocity = testTraj.sample(time).velocityMetersPerSecond;
  setDriveVelocity(velocity, velocity);
  return velocity;

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
    //TODO motor.set(ControlMode.Velocity, vel, DemandType.ArbitraryFeedForward, feedforward);
    leftmotor[0].set(ControlMode.Velocity, targetVelL * FPS_TO_CP100MS, DemandType.ArbitraryFeedForward, targetVelL * KV);
    rightmotor[0].set(ControlMode.Velocity, targetVelR * FPS_TO_CP100MS, DemandType.ArbitraryFeedForward, targetVelR * KV);
    
  }

  public double getError(){
    return ((double) leftmotor[0].getClosedLoopError()) / FPS_TO_CP100MS;
  }

  public double getDriveVelocity(){
      //return (getEncoderCounts() * 10 * Math.PI * 6. / (49152 * 7.56)) * 2;
      return (getEncoderCounts() / FPS_TO_CP100MS);
  }

  public double getEncoderCounts(){ //this is currently not working
    return leftmotor[0].getSelectedSensorVelocity();
  }
  public double getDrivePosition(){
    return leftmotor[0].getSelectedSensorPosition() * COUNTS_TO_FEET;
  }
  public void zeroEncoders(){
    leftmotor[0].setSelectedSensorPosition(0);
    rightmotor[0].setSelectedSensorPosition(0);
  }


  public void periodic(){
    double throttle = oi.getDriverThrottle();
    double turn = oi.getDriverTurn();
    setDrivePower(throttle-turn, throttle+turn);
    //setDriveVelocity((throttle-turn)*25, (throttle+turn)*25);
    System.out.println("throttle:" + throttle);
  }
}