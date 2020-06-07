/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterConveyorSubsystem;

public class ShooterConveyorCommand extends CommandBase {

  ShooterConveyorSubsystem shooterConveyorSubsystem;
  double power;
  double lastTimeOnTarget;

  /**
   * Creates a new ShooterConveyorCommand.
   */
   public ShooterConveyorCommand(double power) {
    shooterConveyorSubsystem = ShooterConveyorSubsystem.getInstance();
    this.power = power;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterConveyorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterConveyorSubsystem.getCurrent() < -37 && Timer.getFPGATimestamp() - lastTimeOnTarget > 0.1) {
      shooterConveyorSubsystem.setMotor(.8);
      }
   else {
     shooterConveyorSubsystem.setMotor(-.95);
     lastTimeOnTarget = Timer.getFPGATimestamp();
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterConveyorSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
