/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterConveyorSubsystem extends SubsystemBase {

  private static ShooterConveyorSubsystem shooterConveyorSubsystem;
  private TalonSRX shooterConveyorTalon;

  /**
   * Creates a new ShooterConveyorSubsystem.
   */
  private ShooterConveyorSubsystem() {
    shooterConveyorTalon = new TalonSRX(ShooterConstants.TalonID);
  }

  public void setMotor(double power) {
    shooterConveyorTalon.set(ControlMode.PercentOutput, power);
  }

  public double getCurrent() {
    return shooterConveyorTalon.getStatorCurrent();
  }

  public static ShooterConveyorSubsystem getInstance() {
    if (shooterConveyorSubsystem == null) {
      shooterConveyorSubsystem = new ShooterConveyorSubsystem();
    }
    return shooterConveyorSubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter conv current:", shooterConveyorTalon.getStatorCurrent());
  }
}
