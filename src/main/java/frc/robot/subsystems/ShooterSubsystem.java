/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.lib.TalonFXFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
  */

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  //TODO: put this in a method, lower in the code
  NetworkTableEntry tx = limelightTable.getEntry("tx");
  NetworkTableEntry ty = limelightTable.getEntry("ty");
  NetworkTableEntry ta = limelightTable.getEntry("ta");

  double limelightX = tx.getDouble(0.0);
  double limelightY = ty.getDouble(0.0);
  double limelightArea = ta.getDouble(0.0);




  public enum shooterStates {
    STATIONARY, //Not moving, sets motors to coast mode
    LONG_SHOT, //Shoot from far side of field, allow limelight to zoom
    PRECISION_SHOT, //Especially aligns with 3 point shot
    QUICK_SHOT; //Aims for 3 pointers, will shoot at 2 pointers
  }

  private TalonFX upperFlywheel;
  private TalonFX lowerFlywheel;
  private TalonSRX turretMotor;

  public static final double TURRET_POSITION = 6 + 6;   //TODO: Find/make the conversion factor

  public static ShooterSubsystem instance;

  private ShooterSubsystem() {
    upperFlywheel = TalonFXFactory.createPIDTalonFX(RobotMap.SHOOTER_UPPER_FLYWHEEL_ID, false, 0.0, 0.0, 0.0, 0.0);
    lowerFlywheel = TalonFXFactory.createPIDTalonFX(RobotMap.SHOOTER_LOWER_FLYWHEEL_ID, true, 0.0, 0.0, 0.0, 0.0);
    turretMotor = new TalonSRX(RobotMap.SHOOTER_TURRET_MOTOR_ID);   //TODO: make it a PID
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

  public void setTurretPower(double power){   //TODO: Set soft-stop limits
    turretMotor.set(ControlMode.PercentOutput, power);
  }

  public void setTurretPosition(double position){
    turretMotor.set(ControlMode.Position, position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
