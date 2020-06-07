/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.ConveyorConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase {

  private static ConveyorSubsystem conveyorSubsystem;
  private TalonSRX conveyorTalon;

  /**
   * Creates a new ConveyorSubsystem.
   */
  private ConveyorSubsystem() {
    conveyorTalon = new TalonSRX(ConveyorConstants.TalonID);
  }

  public void setMotor(double power) {
    conveyorTalon.set(ControlMode.PercentOutput, power);
  }

  public double getCurrent() {
    return conveyorTalon.getStatorCurrent();
  }

  public static ConveyorSubsystem getInstance() {
    if (conveyorSubsystem == null) {
      conveyorSubsystem = new ConveyorSubsystem();
    }
    return conveyorSubsystem;
  }

  public void printDashBoard() {
    SmartDashboard.putNumber("Conveyor current:", conveyorTalon.getStatorCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    printDashBoard();
  }
}
