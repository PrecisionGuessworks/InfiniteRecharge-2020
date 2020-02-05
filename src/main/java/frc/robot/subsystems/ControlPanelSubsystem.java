/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.revrobotics.ColorSensorV3;

public class ControlPanelSubsystem extends SubsystemBase {
  /**
   * Creates a new ControlPanelSubsystem.
   */
  //ColorSensorV3 sensor;

  public ControlPanelSubsystem() {

    //sensor = new ColorSensorV3(I2C.Port.kOnboard);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
