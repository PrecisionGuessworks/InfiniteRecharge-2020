/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.shootercommands.QuickShotCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick driver;
  public Joystick operator;

  private static OI instance;


  private OI(){
    driver = new Joystick(0);
    operator = new Joystick(1);

    //JoystickButton b = new JoystickButton(driver, 4);
/*
    ArrayList<Pose2d> pointList = new ArrayList<>();
    pointList.add(new Pose2d());
    pointList.add(new Pose2d(10, 0, new Rotation2d()));
    Trajectory testTraj = TrajectoryGenerator.generateTrajectory(pointList, new TrajectoryConfig(20, 24));
    RamseteController ramCont = new RamseteController();
    double trackWidth = 20.75/12.0;//feet
    DifferentialDriveKinematics kitnematic = new DifferentialDriveKinematics(trackWidth);
    Pose2d startPose = new Pose2d();
    BiConsumer<Double, Double> setRamSpeeds = (targetVelL, targetVelR) -> {
      DriveTrainSubsystem.getInstance().setDriveVelocity(targetVelL, targetVelR);
    };

    //RamseteCommand ramComm = new RamseteCommand(testTraj, startPose, ramCont, kitnematic, setRamSpeeds, DriveTrainSubsystem.getInstance());

    
    JoystickButton driverA = new JoystickButton(driver, 3);*/
    //driverA.whenPressed(comman);
  }

  public static OI getInstance(){
    if(instance==null){
      instance = new OI();
    }
    return instance;
  }

  public double getDriverThrottle(){
    final double throttle = driver.getRawAxis(1);
    return Math.pow(throttle, 3);// * (throttle < 0 ? -1 : 1);
  }

  public double getDriverTurn(){
    final double turn = driver.getRawAxis(4);
    return Math.pow(turn, 3) * 0.4;// * (turn < 0 ? -1 : 1);
  }

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
