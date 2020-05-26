/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private static IntakeSubsystem intakeSubsystem;
  private VictorSPX intakeVictor;
  private DoubleSolenoid intakeDoubleSolenoid;

  /**
   * Creates a new IntakeSubsystem.
   */
  private IntakeSubsystem() {
    intakeVictor = new VictorSPX(IntakeConstants.VictorID);
    intakeDoubleSolenoid = new DoubleSolenoid(IntakeConstants.SolenoidFowardChannel, IntakeConstants.SolenoidReverseChannel);
  }

  public void setMotor(double power) {
    intakeVictor.set(ControlMode.PercentOutput, power);
  }

  public void setDoubleSolenoid(boolean state) {
    if (state) {
      intakeDoubleSolenoid.set(Value.kForward);
    }
    else {
      intakeDoubleSolenoid.set(Value.kReverse);
    }
  }

  public boolean getDoubleSolenoid() {
    return intakeDoubleSolenoid.get() == Value.kForward;
  }

  public static IntakeSubsystem getInstance() {
    if (intakeSubsystem == null) {
      intakeSubsystem = new IntakeSubsystem();
    }
    return intakeSubsystem;
  }

  public void printDashBoard() {
    SmartDashboard.putNumber("Intake Victor voltage:", intakeVictor.getBusVoltage());
    SmartDashboard.putString("Intake Double Solenoid state:", intakeDoubleSolenoid.get().name());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    printDashBoard();
  }
}
