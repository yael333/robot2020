/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPIDCommand extends CommandBase {

  ShooterSubsystem shooterSubsystem;
  double setpoint;
  
  double lastTimeOnTarget;
  double waitTime;

  /**
   * Creates a new ShooterPIDCommand.
   */
  public ShooterPIDCommand(double setpoint) {
    shooterSubsystem = ShooterSubsystem.getInstance();
    this.setpoint = setpoint;
    waitTime = ShooterConstants.PIDWaitTime;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power =  -0.5; //shooterSubsystem.getPID(setpoint)
    shooterSubsystem.setMotor(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    if (shooterSubsystem.atSetpoint()) {
      lastTimeOnTarget = Timer.getFPGATimestamp();
    } 
    return shooterSubsystem.atSetpoint() && Timer.getFPGATimestamp() - lastTimeOnTarget > waitTime;
    */
    return false;
  }
}
