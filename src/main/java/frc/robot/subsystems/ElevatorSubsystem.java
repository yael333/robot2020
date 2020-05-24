/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {

  private static ElevatorSubsystem elevatorSubsystem;
  private CANSparkMax elevatorSMX;
  private CANEncoder SMXEncoder;
  private DoubleSolenoid elevatorDoubleSolenoid;

  /**
   * Creates a new ElevatorSubsystem.
   */
  private ElevatorSubsystem() {
    elevatorSMX = new CANSparkMax(ElevatorConstants.SparkMaxID, MotorType.fromId(ElevatorConstants.SparkMaxID)); // same problem with motor type dunno
    SMXEncoder = elevatorSMX.getEncoder();

    SMXEncoder.setPositionConversionFactor(RobotConstants.tiksPerPulse);
    SMXEncoder.setVelocityConversionFactor(RobotConstants.tiksPerPulse);
    SMXEncoder.setPosition(0);

    elevatorDoubleSolenoid = new DoubleSolenoid(ElevatorConstants.SolenoidFowardChannel, ElevatorConstants.SolenoidReverseChannel);
  }

  public void setMotor(double power) {
    elevatorSMX.set(power);
  }

  public double getEncoder() {
    return SMXEncoder.getPosition();
  }

  public void setDoubleSolenoid(boolean state) {
    if (state) {
      elevatorDoubleSolenoid.set(Value.kForward);
    }
    else {
      elevatorDoubleSolenoid.set(Value.kReverse);
    }
  }

  public boolean getDoubleSolenoid() {
    if (elevatorDoubleSolenoid.get() == Value.kForward) {
      return true;
    }
    else {
      return false;
    }
  }

  public static ElevatorSubsystem getInstance() {
    if (elevatorSubsystem == null) {
      elevatorSubsystem = new ElevatorSubsystem();
    }
    return elevatorSubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
