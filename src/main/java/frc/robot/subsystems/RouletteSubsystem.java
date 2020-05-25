/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.RouletteConstants;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

public class RouletteSubsystem extends SubsystemBase {

  private static RouletteSubsystem rouletteSubsystem;
  private TalonSRX rouletteTalon;
  private Solenoid roulettSolenoid;

  private PIDController colorPID;
  private ColorSensorV3 rouletteColorSensor;
  private ColorMatch colorMatcher;

  private Color rouletteColors[] = {RouletteConstants.Red, RouletteConstants.Yellow, RouletteConstants.Blue, RouletteConstants.Green};
  private ColorMatchResult closestColor;
  private Color lastColor;
  
  private boolean isReversed;
  private int colorEncoder = 0;

  /**
   * Creates a new RouletteSubsystem.
   */
  private RouletteSubsystem() {
    rouletteTalon = new TalonSRX(RouletteConstants.TalonID);
    rouletteColorSensor = new ColorSensorV3(I2C.Port.kOnboard); // maybe it work dont know
    roulettSolenoid = new Solenoid(RouletteConstants.SolenoidID);
    colorMatcher = new ColorMatch();

    for (Color color: rouletteColors) {
      colorMatcher.addColorMatch(color);
    }
  }

  public void setMotor(double power) {
    rouletteTalon.set(ControlMode.PercentOutput, power);
  }

  public Color getColor() {
    return rouletteColorSensor.getColor();
  }

  public double getPID(int setpoint) {
    return MathUtil.clamp(colorPID.calculate(colorEncoder, setpoint), -1, 1);
  }

  public boolean atSetpoint() {
    return colorPID.atSetpoint();
  }

  public int getColorEncoder() {
    return colorEncoder;
  }

  public void setReversed(boolean state) {
    isReversed = state;
  }

  public int getOptimalWay(Color wantedColor) {
    int currentColorIndex = Arrays.binarySearch(rouletteColors, closestColor);
    int wantedColorIndex = Arrays.binarySearch(rouletteColors, wantedColor);
    int positive_way = currentColorIndex + wantedColorIndex;
    int negative_way = currentColorIndex - (4 - wantedColorIndex);

    return Math.abs(positive_way) > Math.abs(negative_way) ? positive_way : negative_way;
  }

  public static RouletteSubsystem getInstance() {
    if (rouletteSubsystem == null) {
      rouletteSubsystem = new RouletteSubsystem();
    }
    return rouletteSubsystem;
  }

  public void printDashBoard() {
    SmartDashboard.putNumber("Roulette talon voltage:", rouletteTalon.getBusVoltage());
    SmartDashboard.putBoolean("Roulette solenoid state:", roulettSolenoid.get());
    SmartDashboard.putString("Roulette Color:", closestColor.color.toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    closestColor =  colorMatcher.matchClosestColor(getColor());
    if (closestColor.color != lastColor) {
        if (!isReversed) {
          colorEncoder++;
        }
        else {
          colorEncoder--;
        }
      }
    lastColor = closestColor.color;

    printDashBoard();
  }
}