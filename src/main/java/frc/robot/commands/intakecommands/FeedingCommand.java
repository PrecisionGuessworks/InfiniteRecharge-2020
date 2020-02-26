/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.intakeStates;

public class FeedingCommand extends CommandBase {

  private IntakeSubsystem intake;

  public FeedingCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    intake = IntakeSubsystem.getInstance();
    addRequirements(intake);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    intake.setState(intakeStates.FEEDING);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    intake.setState(intakeStates.OFF);
  }
}
