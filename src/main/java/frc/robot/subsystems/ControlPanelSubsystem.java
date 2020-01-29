/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlPanelSubsystem extends SubsystemBase {
  /**
   * Creates a new ControlPanelSubsystem.
   */
  public enum controlPanelStates {
    POSITION_CONTROL, //Stage 2 spins until sensor reads x color
    ROTATION_CONTROL, //Stage 1 spins between 3 and 5 times
    OFF; //Isn't spinning
  }

  public ControlPanelSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
