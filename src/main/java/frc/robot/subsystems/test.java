/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.MAMotorControler;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.RobotConstants.motorType;

public class test extends SubsystemBase {
  /**
   * Creates a new test.
   */
  private static test Test;

  MAMotorControler sparkMAX1;
  MAMotorControler sparkMAX2;
  MAMotorControler talon1;
  MAMotorControler talon2;
  MAMotorControler talon3;
  MAMotorControler victor;
  MAMotorControler Spark1;

  private test() {

    Spark1 = new MAMotorControler(motorType.SPARK, 0, 60, false, 0);
    victor = new MAMotorControler(motorType.VICTOR, 6, 60, false, 0);
    talon1 = new MAMotorControler(motorType.TALON, RobotConstants.m_ID3, false, true, 60, false, 0);
    talon2 = new MAMotorControler(motorType.TALON, RobotConstants.m_ID4, true, false, 60, false, 0);
    sparkMAX1 = new MAMotorControler(motorType.SPARK_MAX, RobotConstants.m_ID1, true, false, 60, false, 0);
    sparkMAX1.configRampRate(10);
    talon1.configRampRate(10);
    victor.configRampRate(10);

  }

  public static test getinstance() {
    if (Test == null) {
      Test = new test();
    }
    return Test;
  }

  @Override
  public void periodic() {
    talon2.set(1);
    talon1.set(-1);
    sparkMAX1.set(1);

  }

}
