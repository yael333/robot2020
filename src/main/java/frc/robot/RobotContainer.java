/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Chassis.MAPath;
import frc.robot.commands.Chassis.PIDVision;
import frc.robot.commands.Chassis.PIDVisionFeeder;
import frc.robot.commands.automation.AutomationPrepareShootCommandGroup;
import frc.robot.commands.automation.AutomationShootCommandGroup;
import frc.robot.commands.conveyor.ConveyorMoveCommand;
import frc.robot.commands.elevator.ElevatorDoubleSolenoidCommand;
import frc.robot.commands.elevator.ElevatorMoveCommand;
import frc.robot.commands.intake.IntakeDoubleSolenoid;
import frc.robot.commands.intake.IntakeMoveCommand;
import frc.robot.commands.roulette.RoulettePIDCommand;
import frc.robot.commands.roulette.RouletteSolenoidCommand;
import frc.robot.commands.shooter.ShooterConveyorCommand;
import frc.robot.commands.shooter.ShooterPIDCommand;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.triggers.LeftTrigger;
import frc.robot.triggers.RightTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

  public static RightTrigger r2 = new RightTrigger();
  public static LeftTrigger l2 = new LeftTrigger();

  public static POVButton UpperPovButton = new POVButton(OperatingJoystick, 0);
  public static POVButton LeftPOVButton = new POVButton(OperatingJoystick, 90);
  public static POVButton DownPOVButton = new POVButton(OperatingJoystick, 180);
  public static POVButton RightPOVButton = new POVButton(OperatingJoystick, 270);

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

    
    aButton.whileHeld(new IntakeMoveCommand(0.5));
    bButton.whileHeld(new IntakeMoveCommand(-0.5));
    
    xButton.whileHeld(new ConveyorMoveCommand(0.5));
    yButton.whileHeld(new ConveyorMoveCommand(0.5));

    l1Button.whileHeld(new ShooterConveyorCommand(0.5));
    r1Button.whileHeld(new ShooterConveyorCommand(-0.5)); 

    r2.whileActiveContinuous(new ShooterPIDCommand(0));
    //l2.whileActiveOnce(new RoulettePIDCommand(null));

    UpperPovButton.whenActive(new IntakeDoubleSolenoid());
    LeftPOVButton.whenActive(new RouletteSolenoidCommand());
    DownPOVButton.whenActive(new ElevatorDoubleSolenoidCommand());

    /* I think this needs to be a default command but not sure, also the equal sign in the velocity thing might be wrong too
    new ConditionalCommand(new AutomationShootCommandGroup(), new AutomationPrepareShootCommandGroup(), 
    () -> ShooterSubsystem.getInstance().getIR() && ShooterSubsystem.getInstance().getEncoderVelocity() == ShooterConstants.velocitySetpoint);
    */
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
