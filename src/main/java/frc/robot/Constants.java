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
    public static final int LEADER_LEFT_TALON_ID = 1;

    public static final int LEADER_RIGHT_TALON_ID = 3;

    public static final int FOLLOWER_LEFT_TALON_ID = 6;

    public static final int FOLLOWER_RIGHT_TALON_ID = 4;

    //ENCODER VALUES
    public static final int[] LEFT_ENCODER_PORTS = new int[]{6, 7}; //TODO: THESE ARE STAND INS

    public static final int[] RIGHT_ENCODER_PORTS = new int[]{8, 9}; //TODO: THESE ARE STAND INS

    public static final double DISTANCE_PER_PULSE = 0.00187022937;

    //FEEDFORWARD AND FEEDBACK GAINS
    public static final double ksVolts = 1.11; 

    public static final double kvVoltSecondsPerMeter = 3.0; 

    public static final double kaVoltSecondsSquaredPerMeter = 0.368; 

    public static final double kPDriveVel = 13.3; 

    //DIFFERENTIAL DRIVE KINEMATICS
    public static final double kTrackwidth = 0.55245; // in meters

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidth);

    //MAX TRAJECTORY VELOCITY AND ACCELERATION
    public static final double kMaxSpeed = 1.5; // TODO: ASSIGN A REAL VALUE meters per second

    public static final double kMaxAcceleration = 0.5; // TODO: ASSIGN A REAL VALUE meters per second per second

    //RAMSETE PARAMETERS
    public static final double kRamseteB = 2; // generic ramsete values

    public static final double kRamseteZeta = 0.7; // generic ramsete values


}
