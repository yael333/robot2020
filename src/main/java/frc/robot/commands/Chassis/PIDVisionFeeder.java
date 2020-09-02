/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.limelight;

public class PIDVisionFeeder extends CommandBase {

  private Chassis chassis;

  public PIDVisionFeeder() {
    chassis = Chassis.getinstance();
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.getinstance().camMode(0);
    limelight.getinstance().pipeline(1);
    chassis.rampRate(0);
    chassis.setidilmodeBrake(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double angel = chassis.anglePIDVisionOutput(0);
    double distacne = chassis.distancePIDVisionOutput(66);
    chassis.ArcadeDrive(angel, distacne);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.getinstance().camMode(1);
    chassis.tankDrive(0, 0);
    chassis.reset();
    chassis.setidilmodeBrake(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.isPIDVisionOnTargetDistance();
  }
}