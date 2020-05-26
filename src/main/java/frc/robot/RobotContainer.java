/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Chassis.MAPath;
import frc.robot.commands.Chassis.PIDVision;
import frc.robot.commands.Chassis.PIDVisionFeeder;

import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/*
import frc.robot.commands.Roulette.roundTwoRoulettePID;
import frc.robot.commands.Roulette.roundThreeRoulettePID;
import frc.robot.subsystems.Roulette;
*/

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static XboxController OperatingJoystick = new XboxController(2);
  public static Joystick leftJoystick = new Joystick(0);
  public static Joystick rightJoystick = new Joystick(1);

  public static JoystickButton aButton = new JoystickButton(OperatingJoystick, ControllerConstants.AButton);
  public static JoystickButton bButton = new JoystickButton(OperatingJoystick, ControllerConstants.BButton);
  public static JoystickButton xButton = new JoystickButton(OperatingJoystick, ControllerConstants.Xbutton);
  public static JoystickButton yButton = new JoystickButton(OperatingJoystick, ControllerConstants.YButton);
  public static JoystickButton l1Button = new JoystickButton(OperatingJoystick, ControllerConstants.L1Button);
  public static JoystickButton r1Button = new JoystickButton(OperatingJoystick, ControllerConstants.R1Button);

  public static JoystickButton lmButton = new JoystickButton(OperatingJoystick, ControllerConstants.LeftMiddleButton);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    aButton.whenPressed();
    bButton.whileHeld();
    
    xButton.whileHeld();

    l1Button.whileHeld();
    r1Button.whileHeld();

    lmButton.whileHeld();
    xButton.whileHeld();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
