/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    //TALON IDs
    public static final int MIDDLE_LEFT_TALON_ID = 1;

    public static final int MIDDLE_RIGHT_TALON_ID = 2;

    public static final int BACK_LEFT_TALON_ID = 3;

    public static final int BACK_RIGHT_TALON_ID = 4;

    //FEEDFORWARD AND FEEDBACK GAINS
    public static final double ksVolts = 0.22;

    public static final double kvVoltSecondsPerMeter = 1.98;

    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    public static final double kPDriveVel = 8.5;

    //DIFFERENTIAL DRIVE KINEMATICS
    public static final double kTrackwidth = 0.69; // meters

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidth);

    //MAX TRAJECTORY VELOCITY AND ACCELERATION
    public static final double kMaxSpeed = 3; // meters per second

    public static final double kMaxAcceleration = 3; // meters per second per second

    //RAMSETE PARAMETERS
    public static final double kRamseteB = 2; // generic ramsete values
    
    public static final double kRamseteZeta = 0.7; // generic ramsete values



}
