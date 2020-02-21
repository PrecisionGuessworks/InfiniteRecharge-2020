/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;
  public static DriveTrainSubsystem drive = DriveTrainSubsystem.getInstance();

  public static final double QTURN_THRESHOLD = 0.1;
  public static Logger logger;
  double startTime = -1.0;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  double lastLoop = 0;

  Trajectory testTraj0;



  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = OI.getInstance();
    //m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    logger = new Logger();

    logger.createLogStream("DrivetrainLog");

    SmartDashboard.putNumber("targetVelL", 0);
    SmartDashboard.putNumber("targetVelR", 0);


    ArrayList<Pose2d> pointList0 = new ArrayList<>();

    pointList0.add(new Pose2d(2.667, 0, new Rotation2d()));
    pointList0.add(new Pose2d(15, 10, new Rotation2d()));
    testTraj0 = TrajectoryGenerator.generateTrajectory(pointList0, new TrajectoryConfig(19, 23).setReversed(false));

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    startTime = -1.0;

    logger.flush("DrivetrainLog");
    
  }

  @Override
  public void disabledPeriodic() {


    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
    drive.zeroEncoders();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    if (startTime == -1.0){
      startTime = Timer.getFPGATimestamp();
    }
    double currTime = Timer.getFPGATimestamp() - startTime;
    
    //double setSpeed = drive.setSpeedbyTrajectory(testTraj0, currTime);
    
    drive.setSpeedbyTrajectory(testTraj0, currTime);
    /*
    logger.logDoubles("DrivetrainLog", currTime, drive.setSpeedbyTrajectory(testTraj0, currTime), drive.getLeftDriveVelocity(), drive.getRightDriveVelocity());
    */

    double[] setSpeed = new double[2];// = drive.setSpeedbyTrajectory(currTime);

    logger.logDoubles("DrivetrainLog", currTime, setSpeed[0], setSpeed[1], drive.getLeftDriveVelocity(), drive.getRightDriveVelocity());

    //SmartDashboard.putNumber("Trajectory Speeds", setSpeed);
    SmartDashboard.putNumber("LeftDriveSpeed", drive.getLeftDriveVelocity());
    SmartDashboard.putNumber("LeftDrivePosition", drive.getDrivePosition());
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //logger.createLogStream("ImplementingTest0");
    if (startTime == -1.0){
      startTime = Timer.getFPGATimestamp();
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    logger.logDoubles("DrivetrainLog", Timer.getFPGATimestamp(), ((double) drive.getLeftDriveVelocity()));
  }


  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    double throttle = m_oi.getDriverThrottle();
    double turn = m_oi.getDriverTurn();
    boolean quickturn = m_oi.driver.getRawButton(6);
    //drive.setDrivePower(throttle+turn, throttle-turn);
    
    if (Math.abs(throttle) < QTURN_THRESHOLD) {
      quickturn = true;
    }
    
    drive.setDrivePowerWithCurvature(throttle, turn, quickturn);

    if(m_oi.driver.getRawButton(1)){
      drive.setDriveVelocity(SmartDashboard.getNumber("targetVelL", 0), SmartDashboard.getNumber("targetVelR", 0));
    }

    /*
    //runs every 100ms
    if (Timer.getFPGATimestamp() - lastLoop > 0.05) {
      logger.logDoubles("ImplementingTest0", Timer.getFPGATimestamp(), drive.getLeftDriveVelocity());
      lastLoop = Timer.getFPGATimestamp();
    }*/
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
