/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.lib.TalonFXFactory;
import frc.robot.lib.TalonSRXFactory;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
  */

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

 
  public enum shooterStates {
    STATIONARY, //Not moving, sets motors to coast mode
    LONG_SHOT, //Shoot from far side of field, allow limelight to zoom
    PRECISION_SHOT, //Especially aligns with 3 point shot
    QUICK_SHOT; //Aims for 3 pointers, will shoot at 2 pointers
  }

  private TalonFX upperFlywheel;
  private TalonFX lowerFlywheel;
  private TalonSRX turretMotor;


  private shooterStates currentState;

  public static final double TURRET_POSITION = 6 + 6;   //TODO: Find/make the conversion factor
  public static final double SHOOTER_RPM_TO_CP100MS = (2048f / (60f * 10f));

  public static ShooterSubsystem instance;

  private ShooterSubsystem() {
    upperFlywheel = TalonFXFactory.createPIDTalonFX(RobotMap.SHOOTER_UPPER_FLYWHEEL_ID, false, 0.0, 0.0, 0.0, 0.0);
    lowerFlywheel = TalonFXFactory.createPIDTalonFX(RobotMap.SHOOTER_LOWER_FLYWHEEL_ID, true, 0.0, 0.0, 0.0, 0.0);
    turretMotor = TalonSRXFactory.createPIDTalonSRX(RobotMap.SHOOTER_TURRET_MOTOR_ID, 0.01, 0, 0, 0, FeedbackDevice.CTRE_MagEncoder_Absolute);
    currentState = shooterStates.STATIONARY;

    
  }

  public static ShooterSubsystem getInstance(){
    if(instance==null){
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  public void setState(shooterStates newState){
    currentState = newState;
    //TODO: Do turret stuff
    switch (currentState) {
      case STATIONARY:
        setShooterPower(0.5, 0.5);
        limelightTable.getEntry("ledmode").setNumber(1);
        limelightTable.getEntry("cammode").setNumber(1);

        break;
      case LONG_SHOT:
      limelightTable.getEntry("pipeline").setNumber(1);
      limelightTable.getEntry("ledmode").setNumber(0);
      limelightTable.getEntry("cammode").setNumber(0);
      setShooterSpeed(3500, 3500);
      //TODO: Adjust speeds accordingly / make a variable
      //TODO: Adjust angle based on limelight

        break;
      case PRECISION_SHOT:

      //If not aligned, align
      setShooterSpeed(2700, 2700);
      limelightTable.getEntry("pipeline").setNumber(0);
      limelightTable.getEntry("ledmode").setNumber(0);
      limelightTable.getEntry("cammode").setNumber(0);
       //TODO: Adjust angle based on limelight
        break;
      case QUICK_SHOT:
      limelightTable.getEntry("pipeline").setNumber(0);
      limelightTable.getEntry("ledmode").setNumber(0);
      limelightTable.getEntry("cammode").setNumber(0);
      setShooterSpeed(2700, 2700);
      //TODO: Adjust speeds accordingly / make a variable
      //TODO: Adjust angle based on limelight
        break;
    }
  }

  public void setShooterPower(double upperPower, double lowerPower){
    upperFlywheel.set(ControlMode.PercentOutput, upperPower);
    lowerFlywheel.set(ControlMode.PercentOutput, lowerPower);

  }

  public void setShooterSpeed(double upperSpeed, double lowerSpeed){
    double upperSpeedConverted = upperSpeed * SHOOTER_RPM_TO_CP100MS;
    double lowerSpeedConverted = lowerSpeed * SHOOTER_RPM_TO_CP100MS;    
    upperFlywheel.set(ControlMode.Velocity, upperSpeedConverted);
    lowerFlywheel.set(ControlMode.Velocity, lowerSpeedConverted);
  }

  public void setTurretPosition(double position){
    if (position < -11000){
      position = -10000;
    }
    if (position > 8000){
      position = 10000;
    }
    turretMotor.set(ControlMode.Position, position);
  }

  public double getTurretPosition(){
    return turretMotor.getSelectedSensorPosition();
  }

  public boolean rotateToTarget(double limelightX, double tolerance) {
    //TODO: Finish this method
    return false;
  }
  public void setTurretPower(double turretPower){
    turretMotor.set(ControlMode.PercentOutput, turretPower);
  }

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  NetworkTableEntry tx = limelightTable.getEntry("tx");
  NetworkTableEntry ty = limelightTable.getEntry("ty");
  NetworkTableEntry ta = limelightTable.getEntry("ta");
  NetworkTableEntry tv = limelightTable.getEntry("tv");

  double limelightX = tx.getDouble(0.0);
  double limelightY = ty.getDouble(0.0);
  double limelightArea = ta.getDouble(0.0);
  double limelightHasTarget = tv.getDouble(0);

  }
}
