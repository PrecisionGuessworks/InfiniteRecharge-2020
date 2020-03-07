/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.lib.TalonFXFactory;
import frc.robot.lib.TalonSRXFactory;
import frc.robot.util.Constants;

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
  public static final double SHOOTER_RADIANS_TO_CP100MS = (22161/Math.PI);
  public static final double LIMELIGHTX_TO_RADIANS = (1);

  public static ShooterSubsystem instance;

  private ShooterSubsystem() {
    upperFlywheel = TalonFXFactory.createPIDTalonFX(RobotMap.SHOOTER_UPPER_FLYWHEEL_ID, 
                                                    false, 
                                                    Constants.SHOOTER_UPPER_FLYWHEEL_CONSTANTS.p, 
                                                    Constants.SHOOTER_UPPER_FLYWHEEL_CONSTANTS.i, 
                                                    Constants.SHOOTER_UPPER_FLYWHEEL_CONSTANTS.d, 
                                                    Constants.SHOOTER_UPPER_FLYWHEEL_CONSTANTS.f);
    lowerFlywheel = TalonFXFactory.createPIDTalonFX(RobotMap.SHOOTER_LOWER_FLYWHEEL_ID, 
                                                    true, 
                                                    Constants.SHOOTER_LOWER_FLYWHEEL_CONSTANTS.p, 
                                                    Constants.SHOOTER_LOWER_FLYWHEEL_CONSTANTS.i, 
                                                    Constants.SHOOTER_LOWER_FLYWHEEL_CONSTANTS.d, 
                                                    Constants.SHOOTER_LOWER_FLYWHEEL_CONSTANTS.f);
    turretMotor = TalonSRXFactory.createPIDTalonSRX(RobotMap.SHOOTER_TURRET_MOTOR_ID, 0.2, 0, 0, 0, FeedbackDevice.CTRE_MagEncoder_Absolute);
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
    if (position < -9842){
      position = -9842;
    }
    if (position > 12319){
      position = 12319;
    }
    turretMotor.set(ControlMode.Position, position);
  }

  public void setTurretPositionRadians(double radians) {
    setTurretPosition(-1238 + radians * SHOOTER_RADIANS_TO_CP100MS);
  }

  public double getTurretPositionRadians(){
    return (turretMotor.getSelectedSensorPosition() + 1238) /SHOOTER_RADIANS_TO_CP100MS;
  }

  public void setTurretWithLimelight() {
    double curPos = getTurretPositionRadians();
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    double limelightX = tx.getDouble(0.0);
    double limeLightRadians = limelightX * LIMELIGHTX_TO_RADIANS;
  }

  public void setTurretPower(double turretPower){
    turretMotor.set(ControlMode.PercentOutput, turretPower);
  }

  public void setCoastMode(){
    upperFlywheel.setNeutralMode(NeutralMode.Coast);
    lowerFlywheel.setNeutralMode(NeutralMode.Coast);
  }

  public double[] getFlywheelSpeed(){
    double upperSpeed = upperFlywheel.getSelectedSensorVelocity() * 1/SHOOTER_RPM_TO_CP100MS;
    double lowerSpeed = lowerFlywheel.getSelectedSensorVelocity() * 1/SHOOTER_RPM_TO_CP100MS;
    double[] speeds = new double[]{upperSpeed, lowerSpeed};
    return speeds;
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
