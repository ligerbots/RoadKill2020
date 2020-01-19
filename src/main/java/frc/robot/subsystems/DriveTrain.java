/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("all")
public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private final SpeedControllerGroup leftMotors =
      new SpeedControllerGroup(new WPI_TalonSRX(Constants.MIDDLE_LEFT_TALON_ID),
                               new WPI_TalonSRX(Constants.BACK_LEFT_TALON_ID));

  private final SpeedControllerGroup rightMotors  =
      new SpeedControllerGroup(new WPI_TalonSRX(Constants.MIDDLE_RIGHT_TALON_ID),
                               new WPI_TalonSRX(Constants.BACK_RIGHT_TALON_ID));

  DifferentialDrive robotDrive = new DifferentialDrive(leftMotors, rightMotors);

  AHRS navx = new AHRS(Port.kMXP, (byte) 200);

  DifferentialDriveOdometry odometry;

  public DriveTrain() {
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
