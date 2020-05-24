/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.ClimbBalanceConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbBalanceSubsystem extends SubsystemBase {

  private static ClimbBalanceSubsystem climbBalanceSubsystem;
  private CANSparkMax balanceSMX;

  /**
   * Creates a new ClimbBalanceSubsystem.
   */
  private ClimbBalanceSubsystem() {
    balanceSMX = new CANSparkMax(ClimbBalanceConstants.SparkMaxID, MotorType.fromId(ClimbBalanceConstants.SparkMaxID)); // not sure whats the motor type hopefully this function works
  }

  public void setMotor(double power) {
    balanceSMX.set(power);
  }

  public static ClimbBalanceSubsystem getInstance() {
    if (climbBalanceSubsystem == null) {
      climbBalanceSubsystem = new ClimbBalanceSubsystem();
    }
    return climbBalanceSubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
