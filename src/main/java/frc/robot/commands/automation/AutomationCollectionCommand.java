/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.conveyor.ConveyorMoveCommand;
import frc.robot.commands.intake.IntakeDoubleSolenoid;
import frc.robot.commands.intake.IntakeMoveCommand;
import frc.robot.commands.shooter.ShooterConveyorCommand;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutomationCollectionCommand extends ParallelDeadlineGroup {
  /**
   * Creates a new AutomationCollectionCommand.
   */
  public AutomationCollectionCommand() {
    // Add your commands in the super() call.  Add the deadline first.
    super( new WaitUntilCommand(ShooterSubsystem.getInstance()::getIR),
      new SequentialCommandGroup(
        new IntakeDoubleSolenoid(), 
        new IntakeMoveCommand(0.5)), 
      new ConveyorMoveCommand(0.5), 
      new ShooterConveyorCommand(0.5)
    );
  }
}
