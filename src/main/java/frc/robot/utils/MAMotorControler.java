/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

/**
 * @author yuval rader
 */

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.utils.RobotConstants.motorType;
import edu.wpi.first.wpilibj.Spark;

import frc.robot.utils.RobotConstants.motorType;
import frc.robot.utils.RobotConstants.encoderType;

/**
 * Add your docs here.
 */
public class MAMotorControler {
    private CANSparkMax canSparkMax;
    private CANDigitalInput candDigitalInputForward;
    private CANDigitalInput candDigitalInputRevers;
    private CANEncoder canEncoder;

    private WPI_TalonSRX talonSRX;
    private WPI_VictorSPX victorSPX;
    private Spark spark;
    private motorType name;
    private int ID;
    private encoderType encoder;
    private int voltage = 12;
    private boolean hasReverseLimitSwitch;
    private boolean hasForwardLimitSwitch;
    private boolean Inverted;
    private int amp;
    private double rampRate;

    /**
     * Constructor for motor controllers
     * 
     * @param name                  the motor controller that the Constructor work
     *                              for (talon, victor, spark and sparkMax)
     * @param ID                    ID device ID of motor controller
     * @param encoder               if the motor controllers has encoder and the
     *                              encoder Type
     * @param FeedbackDevice        work for talonSRX Select the feedback device for
     *                              the motor controller.
     * @param hasForwardLimitSwitch if the motor controllers hasForwardLimitSwitch
     * @param hasReverseLimitSwitch if the motor controllers hasReverseLimitSwitch
     * @param amp                   Amperes to limit
     * @param Inverted              The state of inversion, true is inverted
     * @param rampRate              Minimum desired time to go from neutral to full
     *                              throttle. A value of '0' will disable the ramp.
     */

    public MAMotorControler(motorType name, int ID, encoderType encoder, boolean hasForwardLimitSwitch,
            boolean hasReverseLimitSwitch, FeedbackDevice FeedbackDevice, int amp, boolean Inverted, double rampRate) {
        this.amp = amp;
        this.name = name;
        this.ID = ID;
        this.Inverted = Inverted;
        this.encoder = encoder;
        this.hasReverseLimitSwitch = hasReverseLimitSwitch;
        this.hasForwardLimitSwitch = hasForwardLimitSwitch;
        this.rampRate = rampRate;

        if (name == motorType.TALON) {
            talonSRX = new WPI_TalonSRX(ID);
            talonSRX.setInverted(Inverted);
            talonSRX.configContinuousCurrentLimit(amp);
            talonSRX.configClosedloopRamp(rampRate);
            talonSRX.configOpenloopRamp(rampRate);

            if (encoder == encoderType.Encoder) {

                talonSRX.configSelectedFeedbackSensor(FeedbackDevice);
            }

            if (hasReverseLimitSwitch) {

                talonSRX.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
                        0);
            }
    
            if (hasForwardLimitSwitch) {
                
                talonSRX.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
                        0);
            }

            
        } else if (name == motorType.VICTOR) {
            victorSPX = new WPI_VictorSPX(ID);
            victorSPX.setInverted(Inverted);
            victorSPX.configClosedloopRamp(rampRate);
            victorSPX.configOpenloopRamp(rampRate);
        } else if (name == motorType.SPARK) {
            spark = new Spark(ID);
            spark.setInverted(Inverted);

        } else {
            canSparkMax = new CANSparkMax(ID, RobotConstants.kBrushless);
            canSparkMax.setSmartCurrentLimit(amp);
            canSparkMax.setInverted(Inverted);
            canSparkMax.setOpenLoopRampRate(rampRate);
            canSparkMax.setClosedLoopRampRate(rampRate);

            if (encoder == encoderType.Encoder) {

                canEncoder = canSparkMax.getEncoder();
                canEncoder.setPositionConversionFactor(RobotConstants.tiksPerPulse);
                canEncoder.setVelocityConversionFactor(RobotConstants.tiksPerPulse);
    
            } else if (encoder == encoderType.Alternate_Encoder) {
    
                canEncoder = canSparkMax.getAlternateEncoder(AlternateEncoderType.kQuadrature, RobotConstants.tiksPerPulse);
                canEncoder.setPositionConversionFactor(RobotConstants.tiksPerPulse);
                canEncoder.setVelocityConversionFactor(RobotConstants.tiksPerPulse);
            }
    
            if (hasForwardLimitSwitch) {
    
                candDigitalInputForward = canSparkMax.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
            }
    
            if (hasReverseLimitSwitch) {
    
                candDigitalInputRevers = canSparkMax.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
            }
        }
    }

    /**
     * Constructor for motor controllers
     * 
     * @param name     the motor controller that the Constructor work for (talon,
     *                 victor, spark and sparkMax)
     * @param ID       ID device ID of motor controller
     * @param amp      Amperes to limit
     * @param Inverted The state of inversion, true is inverted
     * @param rampRate Minimum desired time to go from neutral to full throttle. A
     *                 value of '0' will disable the ramp.
     */
    public MAMotorControler(motorType name, int ID, int amp, boolean Inverted, double rampRate) {
        this.amp = amp;
        this.name = name;
        this.ID = ID;
        this.Inverted = Inverted;
        this.rampRate = rampRate;

        if (name == motorType.TALON) {
            talonSRX = new WPI_TalonSRX(ID);
            talonSRX.setInverted(Inverted);
            talonSRX.configContinuousCurrentLimit(amp);
            talonSRX.configClosedloopRamp(rampRate);
            talonSRX.configOpenloopRamp(rampRate);
            talonSRX.setNeutralMode(NeutralMode.Coast);

        } else if (name == motorType.VICTOR) {
            victorSPX = new WPI_VictorSPX(ID);
            victorSPX.setInverted(Inverted);
            victorSPX.configClosedloopRamp(rampRate);
            victorSPX.configOpenloopRamp(rampRate);
            victorSPX.setNeutralMode(NeutralMode.Coast);
        } else if (name == motorType.SPARK) {
            spark = new Spark(ID);
            spark.setInverted(Inverted);

        } else {
            canSparkMax = new CANSparkMax(ID, RobotConstants.kBrushless);
            canSparkMax.setSmartCurrentLimit(amp);
            canSparkMax.setInverted(Inverted);
            canSparkMax.setOpenLoopRampRate(rampRate);
            canSparkMax.setClosedLoopRampRate(rampRate);
            canSparkMax.setIdleMode(IdleMode.kCoast);
        }
    }

    /**
     * Constructor for motor controllers
     * 
     * @param name           the motor controller that the Constructor work for
     *                       (talon, victor, spark and sparkMax)
     * @param ID             ID device ID of motor controller
     * @param encoder        if the motor controllers has encoder and the encoder
     *                       Type
     * @param FeedbackDevice work for talonSRX Select the feedback device for the
     *                       motor controller.
     * @param amp            Amperes to limit
     * @param Inverted       The state of inversion, true is inverted
     * @param rampRate       Minimum desired time to go from neutral to full
     *                       throttle. A value of '0' will disable the ramp.
     */
    public MAMotorControler(motorType name, int ID, encoderType encoder, FeedbackDevice FeedbackDevice, int amp, boolean Inverted,
            double rampRate) {
        this.amp = amp;
        this.name = name;
        this.ID = ID;
        this.Inverted = Inverted;
        this.encoder = encoder;
        this.rampRate = rampRate;

        if (name == motorType.TALON) {
            talonSRX = new WPI_TalonSRX(ID);
            talonSRX.setInverted(Inverted);
            talonSRX.configContinuousCurrentLimit(amp);
            talonSRX.configClosedloopRamp(rampRate);
            talonSRX.configOpenloopRamp(rampRate);
            
            if (encoder == encoderType.Encoder) {

                talonSRX.configSelectedFeedbackSensor(FeedbackDevice);
            }

        } else if (name == motorType.VICTOR) {
            victorSPX = new WPI_VictorSPX(ID);
            victorSPX.setInverted(Inverted);
            victorSPX.configClosedloopRamp(rampRate);
            victorSPX.configOpenloopRamp(rampRate);
        } else if (name == motorType.SPARK) {
            spark = new Spark(ID);
            spark.setInverted(Inverted);

        } else {
            canSparkMax = new CANSparkMax(ID, RobotConstants.kBrushless);
            canSparkMax.setSmartCurrentLimit(amp);
            canSparkMax.setInverted(Inverted);
            canSparkMax.setOpenLoopRampRate(rampRate);
            canSparkMax.setClosedLoopRampRate(rampRate);

            if (encoder == encoderType.Encoder) {

                canEncoder = canSparkMax.getEncoder();
                canEncoder.setPositionConversionFactor(RobotConstants.tiksPerPulse);
                canEncoder.setVelocityConversionFactor(RobotConstants.tiksPerPulse);
    
            } else if (encoder == encoderType.Alternate_Encoder) {
    
                canEncoder = canSparkMax.getAlternateEncoder(AlternateEncoderType.kQuadrature, RobotConstants.tiksPerPulse);
                canEncoder.setPositionConversionFactor(RobotConstants.tiksPerPulse);
                canEncoder.setVelocityConversionFactor(RobotConstants.tiksPerPulse);
            }
        }

    }

    /**
     * Constructor for motor controllers
     * 
     * @param name                  the motor controller that the Constructor work
     *                              for (talon, victor, spark and sparkMax)
     * @param ID                    ID device ID of motor controller
     * @param hasForwardLimitSwitch if the motor controllers hasForwardLimitSwitch
     * @param hasReverseLimitSwitch if the motor controllers hasReverseLimitSwitch
     * @param amp                   Amperes to limit
     * @param Inverted              The state of inversion, true is inverted
     * @param rampRate              Minimum desired time to go from neutral to full
     *                              throttle. A value of '0' will disable the ramp.
     */
    public MAMotorControler(motorType name, int ID, boolean hasForwardLimitSwitch, boolean hasReverseLimitSwitch, int amp,
            boolean Inverted, double rampRate) {
        this.amp = amp;
        this.name = name;
        this.ID = ID;
        this.Inverted = Inverted;
        this.hasReverseLimitSwitch = hasReverseLimitSwitch;
        this.hasForwardLimitSwitch = hasForwardLimitSwitch;
        this.rampRate = rampRate;

        if (name == motorType.TALON) {
            talonSRX = new WPI_TalonSRX(ID);
            talonSRX.setInverted(Inverted);
            talonSRX.configContinuousCurrentLimit(amp);
            talonSRX.configClosedloopRamp(rampRate);
            talonSRX.configOpenloopRamp(rampRate);

            if (hasReverseLimitSwitch) {
                talonSRX.overrideLimitSwitchesEnable(true);
                talonSRX.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                        LimitSwitchNormal.NormallyOpen);
            }
    
            if (hasForwardLimitSwitch) {
                talonSRX.overrideLimitSwitchesEnable(true);
                talonSRX.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                        LimitSwitchNormal.NormallyOpen);
    
            }

        } else if (name == motorType.VICTOR) {
            victorSPX = new WPI_VictorSPX(ID);
            victorSPX.setInverted(Inverted);
            victorSPX.configClosedloopRamp(rampRate);
            victorSPX.configOpenloopRamp(rampRate);
        } else if (name == motorType.SPARK) {
            spark = new Spark(ID);
            spark.setInverted(Inverted);

        } else {
            canSparkMax = new CANSparkMax(ID, RobotConstants.kBrushless);
            canSparkMax.setSmartCurrentLimit(amp);
            canSparkMax.setInverted(Inverted);
            canSparkMax.setOpenLoopRampRate(rampRate);
            canSparkMax.setClosedLoopRampRate(rampRate);

            if (hasForwardLimitSwitch) {

                candDigitalInputForward = canSparkMax.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
            }
    
            if (hasReverseLimitSwitch) {
    
                candDigitalInputRevers = canSparkMax.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
            }
        }

    }

    public MAMotorControler(motorType sparkMax, int mId9) {
	}

	/**
     * Sets the voltage output of the SpeedController. Compensates for the current
     * bus voltage to ensure that the desired voltage is output even if the battery
     * voltage is below 12V - highly useful when the voltage outputs are
     * "meaningful" (e.g. they come from a feedforward calculation).
     *
     * <p>
     * NOTE: This function *must* be called regularly in order for voltage
     * compensation to work properly - unlike the ordinary set function, it is not
     * "set it and forget it."
     *
     * @param setPower The voltage to output Value should be between -1.0 and 1.0.
     */
    public void setvoltage(double setPower) {
        if (name == motorType.TALON) {
            talonSRX.setVoltage(setPower * voltage);
        } else if (name == motorType.VICTOR) {
            victorSPX.setVoltage(setPower * voltage);
        } else if (name == motorType.SPARK) {
            spark.setVoltage(setPower * voltage);
        } else {
            canSparkMax.setVoltage(setPower * voltage);
        }
    }

    /**
     * Common interface for setting the speed of a speed controller.
     *
     * @param setPower The speed to set. Value should be between -1.0 and 1.0.
     */
    public void set(double setPower) {
        if (name == motorType.TALON) {
            talonSRX.set(ControlMode.PercentOutput, setPower);
        } else if (name == motorType.VICTOR) {
            victorSPX.set(ControlMode.PercentOutput, setPower);
        } else if (name == motorType.SPARK) {
            spark.set(setPower);
        } else {
            canSparkMax.set(setPower);
        }
    }

    /**
     * Sets the sensor position to 0.
     * 
     * @return Error Code generated by function. 0 indicates no error.
     */
    public void resetEncoder() {
        if (name == motorType.TALON && encoder == encoderType.Encoder) {
            talonSRX.setSelectedSensorPosition(0);
        } else if (name == motorType.SPARK_MAX && encoder != encoderType.No_Encoder) {
            canEncoder.setPosition(0);
        } else {
            System.out.println("error");
        }
    }

    /**
     * Enables clearing the position of the feedback sensor when the forward limit
     * switch is triggered. work with TalonSrx
     * 
     * @param clearPositionOnLimitF Whether clearing is enabled, defaults false
     * @param timeoutMs             Timeout value in ms. If nonzero, function will
     *                              wait for config success and report an error if
     *                              it times out. If zero, no blocking or checking
     *                              is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    public void resatOnLimitF(boolean limit) {
        if (name == motorType.TALON && encoder == encoderType.Encoder) {
            talonSRX.configClearPositionOnLimitF(limit, 0);
        } else {
            System.out.println("error");
        }
    }

    /**
     * Enables clearing the position of the feedback sensor when the reverse limit
     * switch is triggered work with TalonSrx
     * 
     * @param clearPositionOnLimitR Whether clearing is enabled, defaults false
     * @param timeoutMs             Timeout value in ms. If nonzero, function will
     *                              wait for config success and report an error if
     *                              it times out. If zero, no blocking or checking
     *                              is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    public void resatOnLimitR(boolean limit) {
        if (name == motorType.TALON && encoder == encoderType.Encoder) {
            talonSRX.configClearPositionOnLimitR(limit, 0);
        } else {
            System.out.println("error");
        }
    }

    /**
     * Sets the phase of the sensor. Use when controller forward/reverse output
     * doesn't correlate to appropriate forward/reverse reading of sensor. Pick a
     * value so that positive PercentOutput yields a positive change in sensor.
     * After setting this, user can freely call SetInverted() with any value.
     * 
     * work with TalonSrx
     * 
     * @param PhaseSensor Indicates whether to invert the phase of the sensor.
     */
    public void PhaseSensor(boolean PhaseSensor) {
        if (name == motorType.TALON && encoder == encoderType.Encoder) {
            talonSRX.setSensorPhase(PhaseSensor);
        } else {
            System.out.println("error");
        }
    }

    /**
     * Get the selected sensor position (in raw sensor units).
     *
     * @return Position of selected sensor (in raw sensor units).
     */
    public double getPosition() {
        if (name == motorType.TALON && encoder == encoderType.Encoder) {
            return talonSRX.getSelectedSensorPosition();
        } else if (name == motorType.SPARK_MAX && encoder != encoderType.No_Encoder) {
            return canEncoder.getPosition();
        } else {
            return 0;
        }
    }

    /**
     * Get the selected sensor velocity.
     *
     * @return selected sensor (in raw sensor units) per 100ms or the NEO RPM .
     * 
     */
    public double getVelocity() {
        if (name == motorType.TALON && encoder == encoderType.Encoder) {
            return talonSRX.getSelectedSensorVelocity();
        } else if (name == motorType.SPARK_MAX && encoder != encoderType.No_Encoder) {
            return canEncoder.getVelocity();
        } else {
            return 0;
        }
    }

    /**
     * Get the value from the Forward digital input.
     *
     * Retrieve the value of a single digital input channel from a motor controller.
     * This method will return the state of the limit input based on the selected
     * polarity, whether or not it is enabled.
     *
     * @return The state of the limit switch based on the configured polarity
     */
    public boolean getForwardLimitSwitch() {
        if (name == motorType.TALON && hasForwardLimitSwitch) {
            return talonSRX.getSensorCollection().isFwdLimitSwitchClosed();
        } else if (name == motorType.SPARK_MAX && hasForwardLimitSwitch) {
            return candDigitalInputForward.get();
        } else {
            return true;
        }
    }

    /**
     * Get the value from the Revers digital input.
     *
     * Retrieve the value of a single digital input channel from a motor controller.
     * This method will return the state of the limit input based on the selected
     * polarity, whether or not it is enabled.
     *
     * @return The state of the limit switch based on the configured polarity
     */
    public boolean getReversLimitSwitch() {
        if (name == motorType.TALON && hasReverseLimitSwitch) {
            return talonSRX.getSensorCollection().isRevLimitSwitchClosed();
        } else if (name == motorType.SPARK_MAX && hasReverseLimitSwitch) {
            return candDigitalInputRevers.get();
        } else {
            return true;
        }
    }

    /**
     * Sets the enable state for limit switches.
     *
     * @param overrid Enable state for limit switches.
     **/
    public void overrideLimitSwitches(boolean overrid) {
        if (name == motorType.TALON && hasForwardLimitSwitch) {
            talonSRX.overrideLimitSwitchesEnable(overrid);
        } else if (name == motorType.SPARK_MAX && hasForwardLimitSwitch) {
            candDigitalInputForward.enableLimitSwitch(overrid);

        } else if (name == motorType.TALON && hasReverseLimitSwitch) {
            talonSRX.overrideLimitSwitchesEnable(overrid);
        } else if (name == motorType.SPARK_MAX && hasReverseLimitSwitch) {
            candDigitalInputRevers.enableLimitSwitch(overrid);
        }
    }

    /**
     * Common interface for getting the current set speed of a speed controller.
     *
     * @return The current set speed. Value is between -1.0 and 1.0.
     */
    public double getOutput() {
        if (name == motorType.TALON) {
            return talonSRX.getMotorOutputPercent();
        } else if (name == motorType.VICTOR) {
            return victorSPX.getMotorOutputPercent();
        } else if (name == motorType.SPARK) {
            return spark.get();
        } else {
            return canSparkMax.get();
        }
    }

    /**
     * @return The motor controller's output current in Amps.
     */
    public double getStatorCurrent() {
        if (name == motorType.TALON) {
            System.out.println("takon");
            return talonSRX.getStatorCurrent();
        } else if (name == motorType.SPARK_MAX) {
            System.out.println("sparkMAx");
            return canSparkMax.getOutputCurrent();
        } else {
            return 0;
        }
    }

    /**
     * Sets the ramp rate for open loop control modes.
     *
     * This is the maximum rate at which the motor controller's output is allowed to
     * change.
     *
     * @param rampRate Time in seconds to go from 0 to full throttle.
     *
     * @return CANError Set to CANError.kOK if successful
     */
    public void configRampRate(double rampRate) {
        if (name == motorType.TALON) {
            talonSRX.configClosedloopRamp(rampRate);
            talonSRX.configOpenloopRamp(rampRate);
        } else if (name == motorType.VICTOR) {
            victorSPX.configClosedloopRamp(rampRate);
            victorSPX.configOpenloopRamp(rampRate);
        } else if (name == motorType.SPARK_MAX) {
            canSparkMax.setClosedLoopRampRate(rampRate);
            canSparkMax.setOpenLoopRampRate(rampRate);
        } else {
            System.out.println("error");
        }
    }

    /**
     * Common interface for inverting direction of a speed controller.
     *
     * @param setInverted The state of inversion, true is inverted.
     */
    public void setInverted(Boolean setInverted) {
        if (name == motorType.TALON) {
            talonSRX.setInverted(setInverted);
        } else if (name == motorType.VICTOR) {
            victorSPX.setInverted(setInverted);
        } else if (name == motorType.SPARK) {
            spark.setInverted(setInverted);
        } else {
            canSparkMax.setInverted(setInverted);
        }

    }

    /**
     * Disables limit switches triggering (if enabled) when the sensor is no longer
     * detected.
     *
     * @param onOff disable triggering
     * 
     * @return Error Code generated by function. 0 indicates no error.
     */
    public void DisableLimit(boolean onOff) {
        if (name == motorType.TALON) {
            talonSRX.configLimitSwitchDisableNeutralOnLOS(onOff, 0);
        } else {
            System.out.println("error");
        }
    }

    /**
     * Sets the mode setting for motorControler.
     *
     * @param onOff Idle mode (coast or brake).
     *
     */
    public void changeMood(boolean onOff) {
        if (name == motorType.TALON) {
            talonSRX.setNeutralMode(onOff ? NeutralMode.Brake : NeutralMode.Coast);
        } else if (name == motorType.VICTOR) {
            victorSPX.setNeutralMode(onOff ? NeutralMode.Brake : NeutralMode.Coast);
        } else if (name == motorType.SPARK_MAX) {
            canSparkMax.setIdleMode(onOff ? IdleMode.kBrake : IdleMode.kCoast);
        } else {
            System.out.println("error");
        }
    }

    private CANSparkMax getSparkMax() {
        return canSparkMax;
    }

    private IMotorController getIMotorController() {

        if (name == motorType.TALON) {
            return talonSRX;
        } else
            return victorSPX;
    }

    /**
     * Set the control mode and output value so that this motor controller will
     * follow another motor controller. Currently supports following Victor SPX and
     * Talon SRX.
     * 
     * @param motor Motor Controller to follow
     */
    public void followIMotorController(MAMotorControler motor) {
        if (name == motorType.TALON) {
            talonSRX.follow(motor.getIMotorController());
        } else
            victorSPX.follow(motor.getIMotorController());
    }

    /**
     * Causes this controller's output to mirror the provided leader.
     *
     * Only voltage output is mirrored. Settings changed on the leader do not affect
     * the follower.
     * 
     * The motor will spin in the same direction as the leader. This can be changed
     * by passing a true constant after the leader parameter.
     * 
     * Following anything other than a CAN SPARK MAX is not officially supported.
     *
     * @param motor The motor controller to follow.
     *
     * @return CANError Set to CANError.kOK if successful
     */
    public void followSparkMax(MAMotorControler motor) {
        canSparkMax.follow(motor.getSparkMax());
    }
}
