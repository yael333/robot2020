/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem shooterSubsystem;

  private CANSparkMax shooterSparkMax1;
  private CANSparkMax shooterSparkMax2;

  private CANEncoder sparkMaxEncoder;
  private DigitalInput shooterIR;

  private PIDController shooterPID;

  /**
   * Creates a new ShooterSubsystem.
   */
  private ShooterSubsystem() {
    shooterSparkMax1 = new CANSparkMax(ShooterConstants.SparkMaxID1, MotorType.kBrushless);
    shooterSparkMax2 = new CANSparkMax(ShooterConstants.SparkMaxID2, MotorType.kBrushless);

    shooterSparkMax2.follow(shooterSparkMax1);

    sparkMaxEncoder = shooterSparkMax1.getEncoder();
    shooterIR = new DigitalInput(ShooterConstants.IRID);

    sparkMaxEncoder.setPositionConversionFactor(RobotConstants.tiksPerPulse);
    sparkMaxEncoder.setVelocityConversionFactor(RobotConstants.tiksPerPulse);

    shooterPID = new PIDController(ShooterConstants.Kp, ShooterConstants.Ki, ShooterConstants.Kd);
  }

  public void setMotor(double power) {
    shooterSparkMax1.set(power);
  }

  public double getEncoderVelocity() {
    return sparkMaxEncoder.getVelocity();
  }

  public double getEncoderPosition() {
    return sparkMaxEncoder.getPosition();
  }

  public double getPID(double setpoint) {
    return MathUtil.clamp(shooterPID.calculate(getEncoderVelocity(), setpoint), -1, 1);
  }

  public boolean atSetpoint() {
    return shooterPID.atSetpoint();
  }

  public Boolean getIR() {
    return shooterIR.get();
  }

  public static ShooterSubsystem getInstance() {
    if (shooterSubsystem == null) {
      shooterSubsystem = new ShooterSubsystem();
    }
    return shooterSubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
