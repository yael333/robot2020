/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.Chassis.MAPath;
import frc.robot.commands.Chassis.tankDrive;
import frc.robot.commands.balance.ClimbBalanceMoveCommand;
import frc.robot.commands.elevator.ElevatorMoveCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ClimbBalanceSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RouletteSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Robot extends TimedRobot {

  private tankDrive tankDrive = new tankDrive();

  private Command m_autonomousCommand;
  public static double x;
  public static double y;
  public static double tshort;
  public static double yaw1;
  public static double distanceFromTargetLimelightX;
  public static double distanceFromTargetLimelightY;
  public static int path;
  private RobotContainer m_robotContainer;



  @Override
  public void robotInit() {
    MAPath.pathnum = 0;
    m_robotContainer = new RobotContainer();

    IntakeSubsystem.getInstance();
    RouletteSubsystem.getInstance();
    ElevatorSubsystem.getInstance();
    ConveyorSubsystem.getInstance();
    ClimbBalanceSubsystem.getInstance();
    ShooterSubsystem.getInstance();
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry yaw = table.getEntry("camtran");
    NetworkTableEntry Tshort = table.getEntry("tshort");

    // read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    tshort = Tshort.getDouble(0.0);
    yaw1 = yaw.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0, 0 })[4];
    distanceFromTargetLimelightX = yaw.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 })[0];
    distanceFromTargetLimelightY = yaw.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 })[2];

    

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    Chassis.getinstance().setidilmodeCoset();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    MAPath.pathnum = 0;
    Chassis.getinstance().resetValue();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    CommandScheduler.getInstance().setDefaultCommand(Chassis.getinstance(), tankDrive);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);

    Chassis.getinstance().setidilmodeBrake();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Chassis.getinstance().resetValue();
    Chassis.getinstance().rampRate(0);

    // CommandScheduler.getInstance().setDefaultCommand(ElevatorSubsystem.getInstance(), new ElevatorMoveCommand());
    //CommandScheduler.getInstance().setDefaultCommand(ClimbBalanceSubsystem.getInstance() , new ClimbBalanceMoveCommand());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
