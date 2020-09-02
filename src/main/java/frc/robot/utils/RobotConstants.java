/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Add your docs here.
 */
public class RobotConstants {

    public final static MotorType kBrushless = MotorType.kBrushless;
    public final static MotorType kBrushed = MotorType.kBrushed;
    public final static IdleMode kCoast = IdleMode.kCoast;
    public final static IdleMode kBrake = IdleMode.kBrake;

    public static double Neo_RPM = 5700;
    public static double Ramp_Rate_Auto = 0.35;

    public static enum encoderType {
        Encoder, 
        Alternate_Encoder,
        No_Encoder
    }
    public final static int Encoder = 1;
    public final static int Alternate_Encoder = 2;
    public final static int No_Encoder = 3;

    public final static int tiksPerPulse = 1;
   
    public static enum motorType {
        TALON,
        VICTOR,
        SPARK,
        SPARK_MAX
    }
    public static double ticksPerMeter = 22000; //TODO
    
    // Motors
    public final static int m_ID1 = 1;
    public final static int m_ID2 = 2;
    public final static int m_ID3 = 3;
    public final static int m_ID4 = 4;
    public final static int m_ID5 = 5;
    public final static int m_ID6 = 6;
    public final static int m_ID7 = 7;
    public final static int m_ID8 = 8;
    public final static int m_ID9 = 9;
    public final static int m_ID10 = 10;
    public final static int m_ID11 = 11;
    public final static int m_ID12 = 12;
    public final static int m_ID13 = 13;
    public final static int m_ID14 = 14;
    public final static int m_ID15 = 15;

    // Pneumatics
    public final static int p_ID0 = 0;
    public final static int p_ID1 = 1;
    public final static int p_ID2 = 2;
    public final static int p_ID3 = 3;
    public final static int p_ID4 = 4;
    public final static int p_ID5 = 5;
    public final static int p_ID6 = 6;
    public final static int p_ID7 = 7;

    // DIO
    public final static int DIO_ID0 = 0;
    public final static int DIO_ID1 = 1;
    public final static int DIO_ID2 = 2;
    public final static int DIO_ID3 = 3;
    public final static int DIO_ID4 = 4;
    public final static int DIO_ID5 = 5;
    public final static int DIO_ID6 = 6;
    public final static int DIO_ID7 = 7;
    public final static int DIO_ID8 = 8;
    public final static int DIO_ID9 = 9;

    // Buttons
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;
    public static final int BACK = 7;
    public static final int START = 8;
    public static final int STICK_LEFT = 9;
    public static final int STICK_RIGHT = 10;

    // axis
    public static final int STICK_RIGHT_Yaxis = 5; // TODO
    public static final int STICK_RIGHT_Xaxis = 4; // TODO
    public static final int STICK_LEFT_Yaxis = 1; // TODO
    public static final int STICK_LEFT_Xaxis = 2; // TODO
    public static final int LTriger = 2; // TODO
    public static final int RTriger = 3; // TODO

    // POV
    public static final int POV_CENTER = -1;
    public static final int POV_UP = 0;
    public static final int POV_UP_RIGHT = 45;
    public static final int POV_RIGHT = 90;
    public static final int POV_DOWN_RIGHT = 135;
    public static final int POV_DOWN = 180;
    public static final int POV_DOWN_LEFT = 225;
    public static final int POV_LEFT = 270;
    public static final int POV_LEFT_UP = 315;
}
