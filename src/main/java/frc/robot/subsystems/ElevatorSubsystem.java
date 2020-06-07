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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {

  private static ElevatorSubsystem elevatorSubsystem;
  private CANSparkMax elevatorSMX;
  private CANEncoder SMXEncoder;
  private Solenoid elevatorSolenoid;

  /**
   * Creates a new ElevatorSubsystem.
   */
  private ElevatorSubsystem() {
    elevatorSMX = new CANSparkMax(ElevatorConstants.SparkMaxID, MotorType.kBrushless);
    SMXEncoder = elevatorSMX.getAlternateEncoder();

    SMXEncoder.setPositionConversionFactor(RobotConstants.tiksPerPulse);
    SMXEncoder.setVelocityConversionFactor(RobotConstants.tiksPerPulse);
    SMXEncoder.setPosition(0);

    elevatorSMX.setIdleMode(IdleMode.kBrake);

    elevatorSolenoid = new Solenoid(ElevatorConstants.SolenoidChannel);
  }

  public void setMotor(double power) {
    elevatorSMX.set(power);
  }

  public double getEncoder() {
    return SMXEncoder.getPosition();
  }

  public void setSolenoid(boolean state) {
    elevatorSolenoid.set(state);
  }

  public boolean getSolenoid() {
    return elevatorSolenoid.get();
  }

  public static ElevatorSubsystem getInstance() {
    if (elevatorSubsystem == null) {
      elevatorSubsystem = new ElevatorSubsystem();
    }
    return elevatorSubsystem;
  }

  public void printDashBoard() {
    SmartDashboard.putNumber("Elevator SparkMax speed:", elevatorSMX.get());
    SmartDashboard.putNumber("Elevator encoder:", getEncoder());
    SmartDashboard.putBoolean("Elevator solenoid state:", getSolenoid());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    printDashBoard();
  }
}
