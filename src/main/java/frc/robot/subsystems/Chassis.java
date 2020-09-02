/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

/**
 * @author yuval rader
 */

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.utils.*;
import frc.robot.Path.Path;
import frc.robot.commands.Chassis.MAPath;
import frc.robot.utils.RobotConstants.motorType;
import frc.robot.utils.RobotConstants.encoderType;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Chassis extends SubsystemBase {

  private static final double KP_MApath_distance = 40e-6;
  private static final double KI_MApath_distance = 0;
  private static final double KD_MApath_distance = 20e-7;

  private static final double KP_MApath_angle = 1e-2;
  private static final double KI_MApath_angle = 0;
  private static final double KD_MApath_angle = 1e-3;

  private static final double KP_Vision_angle = 2.5e-2;
  private static final double KI_Vision_angle = 8e-4;
  private static final double KD_Vision_angle = 1e-3;

  private static final double KP_Vision_distance = 1.6e-2;
  private static final double KI_Vision_distance = 0;
  private static final double KD_Vision_distance = 0;

  private static final double KP_right_velocity_control = 1.2e-3;
  private static final double KI_right_velocity_control = 0;
  private static final double KD_right_velocity_control = 0;

  private static final double KP_left_velocity_control = 1.2e-3;
  private static final double KI_left_velocity_control = 0;
  private static final double KD_left_velocity_control = 0;

  private static final double anglePIDVisionSetInputRange = 44.5;
  private static final double anglePidMApathSetInputRange = 180;

  private double angle;
  private double sign;
  private double modle = sign;

  private MAMotorControler leftFrontMotor;
  private MAMotorControler leftMotor;

  private MAMotorControler rightFrontMotor;
  private MAMotorControler rightMotor;

  private AHRS navx;

  private MAPidController distancePidMApath; // PID controler of the distance in the pathfinder
  private MAPidController anglePidMApath; // PID controler of the angel in the pathfinder

  private MAPidController leftvelocityControl;
  private MAPidController rightvelocityControl;

  private MAPidController anglePIDVision; // the angel PID in the vison PID
  private MAPidController distancePIDVision;

  private static Chassis chassis;

  private Chassis() {

    leftFrontMotor = new MAMotorControler(motorType.SPARK_MAX, RobotConstants.m_ID1, encoderType.Encoder, null,
        60, true, 0);
    leftMotor = new MAMotorControler(motorType.SPARK_MAX, RobotConstants.m_ID2, encoderType.Alternate_Encoder,
        null, 60, true, 0);

    rightFrontMotor = new MAMotorControler(motorType.SPARK_MAX, RobotConstants.m_ID3, encoderType.Encoder, null,
        60, false, 0);
    rightMotor = new MAMotorControler(motorType.SPARK_MAX, RobotConstants.m_ID4, encoderType.Alternate_Encoder,
        null, 60, false, 0);

    leftMotor.followSparkMax(leftFrontMotor);
    rightMotor.followSparkMax(rightFrontMotor);

    navx = new AHRS(Port.kMXP);

    // the distance PID Pathfinder
    distancePidMApath = new MAPidController(KP_MApath_distance, KI_MApath_distance, KD_MApath_distance, 0, 0, -1, 1);

    // the angel PID pathfinder
    anglePidMApath = new MAPidController(KP_MApath_angle, KI_MApath_angle, KD_MApath_angle, 0, 0, -1, 1);

    // the angel PID vison
    anglePIDVision = new MAPidController(KP_Vision_angle, KI_Vision_angle, KD_Vision_angle, 0, 2, -1, 1);

    anglePIDVision.enableContinuousInput(-anglePIDVisionSetInputRange, anglePIDVisionSetInputRange);

    anglePidMApath.enableContinuousInput(-anglePidMApathSetInputRange, anglePidMApathSetInputRange);

    leftvelocityControl = new MAPidController(KP_left_velocity_control, KI_left_velocity_control,
        KD_left_velocity_control, 0, 0, -12, 12);

    rightvelocityControl = new MAPidController(KP_right_velocity_control, KI_right_velocity_control,
        KD_right_velocity_control, 0, 0, -12, 12);

    distancePIDVision = new MAPidController(KP_Vision_distance, KI_Vision_distance, KD_Vision_distance, 0, 2, -1, 1);
  }

  public double leftvelocityControl(double setPoint) {
    leftvelocityControl.setF(setPoint / RobotConstants.Neo_RPM);
    return leftvelocityControl.calculate(lefttVelocityControlRPM(), setPoint);
  }

  public double rightvelocityControl(double setPoint) {
    leftvelocityControl.setF(setPoint / RobotConstants.Neo_RPM);
    return leftvelocityControl.calculate(lefttVelocityControlRPM(), setPoint);
  }

  public double lefttVelocityControlRPM() {
    return leftFrontMotor.getVelocity();
  }

  public double rightVelocityControlRPM() {
    return rightFrontMotor.getVelocity();
  }

  // updat the value in the smart dash bord
  public void value() {

    SmartDashboard.putNumber("fixedAngle", fixedAngle());
    SmartDashboard.putNumber("stage", MAPath.stage);
    SmartDashboard.putNumber("distacne", average() / RobotConstants.ticksPerMeter);
    SmartDashboard.putNumber("angelSetPoint", anglePidMApath.getSetpoint());
    SmartDashboard.putBoolean("PIDvisonOnTarget", anglePIDVision.atSetpoint(0.1));
    SmartDashboard.putNumber("DistanceSetPoint", distancePidMApath.getSetpoint() / RobotConstants.ticksPerMeter);
    SmartDashboard.putNumber("Distancevison", distance());

    SmartDashboard.putNumber("leftRPM", lefttVelocityControlRPM());
    SmartDashboard.putNumber("rightRPM", rightVelocityControlRPM());

    SmartDashboard.putNumber("leftRPMSetPoint", rightvelocityControl.getSetpoint());
    SmartDashboard.putNumber("rightRPMSetpoint", leftvelocityControl.getSetpoint());

    SmartDashboard.putNumber("leftPowet", leftFrontMotor.getOutput());
    SmartDashboard.putNumber("rightpower", rightFrontMotor.getOutput());
  }

  public void rampRate(double rampRate) {
    rightFrontMotor.configRampRate(rampRate);
    rightMotor.configRampRate(0);
    leftFrontMotor.configRampRate(rampRate);
    leftMotor.configRampRate(0);
  }

  // the average of the encoders
  public double average() {
    return (rightMotor.getPosition() + rightMotor.getPosition()) / 2;
  }

  public double fixedAngle() {
    if (navx.getYaw() != 0) {
      angle = navx.getYaw();
      sign = angle / Math.abs(angle);
      modle = sign * (Math.abs(angle) % 360);
      return -((180 - modle) % 360) + 180;
    } else {
      return 0;
    }

  }

  public void setidilmodeBrake(boolean onOf) {
    leftFrontMotor.changeMood(onOf);
    leftMotor.changeMood(onOf);
    rightFrontMotor.changeMood(onOf);
    rightMotor.changeMood(onOf);
  }

  // set the left and the right motors powers
  public void tankDrive(double leftSpeed, double rightspped) {
    rightFrontMotor.set(rightspped);
    leftFrontMotor.set(leftSpeed);
  }

  // resat the value of the encoder and the navx
  public void resetValue() {
    navx.reset();
    leftMotor.resetEncoder();
    rightMotor.resetEncoder();
  }

  // pid vison distance
  public double distance() {
    return (-1.3276 * Math.pow(10, 6)
        / (-2.43018 * Math.pow(limelight.getinstance().y, 2) + -101.265 * limelight.getinstance().y + -1854.19)); // TODO

  }

  // pid vosin
  public void reset() {
    anglePIDVision.reset();
  }

  public double anglePIDVisionOutput(double setpoint) {
    return anglePIDVision.calculate(limelight.getinstance().x * -1, setpoint);
  }

  public double distancePIDVisionOutput(double setpoint) {
    return distancePIDVision.calculate(limelight.getinstance().Tshort, setpoint);
  }

  public void ArcadeDrive(double angel, double distacne) {
    double w = (100 - Math.abs(angel * 100)) * (distacne) + distacne * 100;
    double v = (100 - Math.abs(distacne * 100)) * (angel) + angel * 100;
    tankDrive((-(v + w) / 200), ((v - w) / 200));
  }

  // the PIDvison
  public void PIDvisionAngle(double angleSetpoint) {
    double power = anglePIDVisionOutput(angleSetpoint);
    tankDrive(-power, power);
  }

  public boolean isPIDVisionOnTargetAngle() {
    return anglePIDVision.atSetpoint(0.1);
  }

  public boolean isPIDVisionOnTargetDistance() {
    return distancePIDVision.atSetpoint(0.1);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // MAPath
  public void setpoint(double distancesetpoint, double anglesetpoint, double Speedlimitdistance,
      double Speedlimitangle) {
    anglePidMApath.setSetpoint(anglesetpoint);
    distancePidMApath.setSetpoint(distancesetpoint * RobotConstants.ticksPerMeter);

    distancePidMApath.setP(KP_MApath_distance * Speedlimitdistance);
    distancePidMApath.setD(KD_MApath_distance * Speedlimitdistance);

    anglePidMApath.setP(KP_MApath_angle * Speedlimitangle);
    anglePidMApath.setD(KD_MApath_angle * Speedlimitangle);
  }

  public double angleEror() {
    return anglePidMApath.getPositionError();
  }

  public double distanceEror() {
    return distancePidMApath.getPositionError();
  }

  public double angleMApathPIDOutput() {
    return MathUtil.clamp(anglePidMApath.calculate(fixedAngle()), -1.0, 1.0);
  }

  public double distanceMApathPIDOutput() {
    return MathUtil.clamp(distancePidMApath.calculate(average()), -1.0, 1.0);
  }

  // MApath
  public void pathfinder() {

    if (MAPath.stage <= Path.mainPath.length - 1) {
      double angel = angleMApathPIDOutput() * Path.mainPath[MAPath.stage][5];
      double distance = distanceMApathPIDOutput() * Path.mainPath[MAPath.stage][4];
      ArcadeDrive(angel, distance);
    } else {
      tankDrive(0, 0);
    }
  }

  public void leftcontrol(double power) {
    leftFrontMotor.setvoltage(power);
  }

  public void rightcontrol(double power) {
    rightFrontMotor.setvoltage(power);
  }

  public static Chassis getinstance() {
    if (chassis == null) {
      chassis = new Chassis();
    }
    return chassis;
  }

  @Override
  public void periodic() {
    value();

  }

}