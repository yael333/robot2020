/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMoveCommand extends CommandBase {

  ElevatorSubsystem elevatorSubsystem;

  /**
   * Creates a new ElevatorMoveCommand.
   */
  public ElevatorMoveCommand() {
    elevatorSubsystem = ElevatorSubsystem.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ( (RobotContainer.OperatingJoystick.getAxisType(1) > 0.5 || RobotContainer.OperatingJoystick.getAxisType(1) < -0.5) && 
         elevatorSubsystem.getEncoder() <= ElevatorConstants.MaxHeight && elevatorSubsystem.getEncoder() >= ElevatorConstants.MinHeight) {
           
      elevatorSubsystem.setMotor(elevatorSubsystem.getSolenoid() ? 0.5 : -0.5);
    }
    else {
      elevatorSubsystem.setMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
