/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.conveyor.ConveyorMoveCommand;
import frc.robot.commands.shooter.ShooterConveyorCommand;
import frc.robot.commands.shooter.ShooterPIDCommand;
import frc.robot.subsystems.Automation;
import frc.robot.subsystems.ShooterSubsystem;

public class AutomationShooterCommand extends CommandBase {

  Automation automationSubsystem;
  ShooterSubsystem shooterSubsystem;
  CommandBase shooterPID, shooterConveyorCommand, conveyorMoveCommand;

  /**
   * Creates a new AutomationShooterCommand.
   */
  public AutomationShooterCommand() {
    automationSubsystem = Automation.getinstance();
    shooterSubsystem = ShooterSubsystem.getInstance();

    shooterPID = new ShooterPIDCommand(2000);
    shooterConveyorCommand = new ShooterConveyorCommand(.5);
    conveyorMoveCommand = new ConveyorMoveCommand(.5);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(automationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.getIR() && shooterSubsystem.atSetpoint()) {
      shooterConveyorCommand.execute();
      conveyorMoveCommand.execute();
    }
    else {
     conveyorMoveCommand.execute();
     shooterPID.execute(); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyorMoveCommand.end(true);
    shooterConveyorCommand.end(true);
    shooterPID.end(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
