/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

/**
 * @author yuval rader
 */
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Path.Path;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.RobotConstants;

public class MAPath extends CommandBase {
  /**
   * Creates a new MAPath.
   */

  private Chassis chassis;
  public static int stage = 0;
  public static int pathnum = 0;
  private double lastTimeOnTarget;
  private double waitTime;

  public MAPath(double waitTime) {
    this.waitTime = waitTime;
    chassis = Chassis.getinstance();
    addRequirements(chassis);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.rampRate(RobotConstants.Ramp_Rate_Auto);
    chassis.setidilmodeBrake(false);
    stage = 0;
    if (SmartDashboard.getNumber("auto", 1) == 1) {
      Path.mainPath = Path.roulettePath;
    } else if (SmartDashboard.getNumber("auto", 1) == 2) {
      Path.mainPath = Path.enemyRoultte;
    } else if (SmartDashboard.getNumber("auto", 1) == 3) {
      Path.mainPath = Path.standart1;
    } else if (SmartDashboard.getNumber("auto", 1) == 4) {
      Path.mainPath = Path.standart;
    } else {
      if (pathnum == 0) {
        Path.mainPath = Path.roulettePath1;
      } else {
        Path.mainPath = Path.roulettePath2;
      }

    }

    chassis.setpoint(Path.mainPath[0][0], Path.mainPath[0][1], Path.mainPath[0][4], Path.mainPath[0][5]);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.pathfinder();

    if (MAPath.stage <= Path.mainPath.length - 1) {
      if (Math.abs(chassis.distanceEror()) < Path.mainPath[stage][2] * RobotConstants.ticksPerMeter
          && Math.abs(chassis.angleEror()) < Path.mainPath[stage][3]) {
        stage++;
        chassis.setpoint(Path.mainPath[stage][0], Path.mainPath[stage][1], Path.mainPath[stage][4],
            Path.mainPath[stage][5]);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      chassis.tankDrive(0, 0);
      chassis.setidilmodeBrake(true);
    } else {

      pathnum++;
      chassis.tankDrive(0, 0);
      chassis.setidilmodeBrake(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (!(Math.abs(chassis.distanceEror()) < Path.mainPath[Path.mainPath.length - 1][2] * RobotConstants.ticksPerMeter
        && Math.abs(chassis.angleEror()) < Path.mainPath[Path.mainPath.length - 1][3]
        && stage == Path.mainPath.length)) {
      lastTimeOnTarget = Timer.getFPGATimestamp();
    }
    return Math.abs(chassis.distanceEror()) < Path.mainPath[Path.mainPath.length - 1][2] * RobotConstants.ticksPerMeter
        && Math.abs(chassis.angleEror()) < Path.mainPath[Path.mainPath.length - 1][3] && stage == Path.mainPath.length
        && Timer.getFPGATimestamp() - lastTimeOnTarget > waitTime;
  }
}