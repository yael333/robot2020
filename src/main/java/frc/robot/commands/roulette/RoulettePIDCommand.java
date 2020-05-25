/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.roulette;


import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RouletteSubsystem;

public class RoulettePIDCommand extends CommandBase {

  RouletteSubsystem rouletteSubsystem;
  int setpoint;

  /**
   * Creates a new RoulettePIDCommand.
   */
  public RoulettePIDCommand(Color wantedColor) {
    rouletteSubsystem = RouletteSubsystem.getInstance();
    this.setpoint = rouletteSubsystem.getColorEncoder() + rouletteSubsystem.getOptimalWay(wantedColor);
    rouletteSubsystem.setReversed(setpoint < 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rouletteSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rouletteSubsystem.setMotor(rouletteSubsystem.getPID(setpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
