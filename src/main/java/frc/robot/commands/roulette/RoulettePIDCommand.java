/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.roulette;

import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RouletteConstants;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RouletteSubsystem;

public class RoulettePIDCommand extends CommandBase {

  RouletteSubsystem rouletteSubsystem;
  int setpoint;
  Color wantedColor;
  double lastTimeOnTarget;
  double waitTime;

  /**
   * Creates a new RoulettePIDCommand.
   */
  public RoulettePIDCommand() {
    
    rouletteSubsystem = RouletteSubsystem.getInstance();
    this.waitTime = RouletteConstants.PIDWaitTime;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rouletteSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();

    switch (gameData.charAt(0)) {
      case 'r' :
      wantedColor = RouletteConstants.Red;
      break;
    case 'b' :
      wantedColor = RouletteConstants.Blue;
      break;
    case 'y':
      wantedColor = RouletteConstants.Yellow;
      break;
    case 'g':
      wantedColor = RouletteConstants.Green;
      break;  
    }
    
    int OptimalWay = rouletteSubsystem.getOptimalWay(wantedColor);
  
    rouletteSubsystem.setSetpoint(rouletteSubsystem.getColorEncoder() + OptimalWay);
    rouletteSubsystem.setReversed(OptimalWay < 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = 0.5; //rouletteSubsystem.getPID();
    rouletteSubsystem.setMotor(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rouletteSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (rouletteSubsystem.atSetpoint()) {
      lastTimeOnTarget = Timer.getFPGATimestamp();
    } 
    return rouletteSubsystem.atSetpoint() && Timer.getFPGATimestamp() - lastTimeOnTarget > waitTime;
  }
}
