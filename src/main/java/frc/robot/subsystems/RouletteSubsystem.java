/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.RouletteConstants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RouletteSubsystem extends SubsystemBase {

  private static RouletteSubsystem rouletteSubsystem;
  private TalonSRX rouletteTalon;
  private ColorSensorV3 rouletteColorSensor;
  private ColorMatch colorMatcher;

  private Color lastColor;

  /**
   * Creates a new RouletteSubsystem.
   */
  private RouletteSubsystem() {
    rouletteTalon = new TalonSRX(RouletteConstants.TalonID);
    rouletteColorSensor = new ColorSensorV3(RouletteConstants.ColorPort); // maybe it work dont know
    colorMatcher = new ColorMatch();

    colorMatcher.addColorMatch(RouletteConstants.Red);
    colorMatcher.addColorMatch(RouletteConstants.Yellow);
    colorMatcher.addColorMatch(RouletteConstants.Blue);
    colorMatcher.addColorMatch(RouletteConstants.Green);
  }

  public Color getColor() {
    return rouletteColorSensor.getColor();
  }

  public static RouletteSubsystem getInstance() {
    if (rouletteSubsystem == null) {
      rouletteSubsystem = new RouletteSubsystem();
    }
    return rouletteSubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    ColorMatchResult closestColor =  colorMatcher.matchClosestColor(getColor());
  }
}
