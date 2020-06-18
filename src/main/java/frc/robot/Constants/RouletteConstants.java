/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Constants;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Add your docs here.
 */
public final class RouletteConstants {
  public static final int VictorID = 10;
  public static final int SolenoidID = 6;

  //colors (need chage real values)
  public static final Color Red = ColorMatch.makeColor(0.65, 0.28, 0.03);
  public static final Color Yellow = ColorMatch.makeColor(0.44, 0.48, 0.07);
  public static final Color Green = ColorMatch.makeColor(0.228760, 0.588379, 0.176514);
  public static final Color Blue = ColorMatch.makeColor(0.195801, 0.478271, 0.325684);

  public static final double Kp = 1;
  public static final double Ki = 1;
  public static final double Kd = 1;
  public static final double PIDWaitTime = 5;
}
