/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.EncoderWrapper;

@SuppressWarnings("all")
public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  WPI_TalonSRX leftLeader = new WPI_TalonSRX(Constants.LEADER_LEFT_TALON_ID);
  WPI_TalonSRX rightLeader = new WPI_TalonSRX(Constants.LEADER_RIGHT_TALON_ID);
  WPI_TalonSRX leftFollower = new WPI_TalonSRX(Constants.FOLLOWER_LEFT_TALON_ID);
  WPI_TalonSRX rightFollower = new WPI_TalonSRX(Constants.FOLLOWER_RIGHT_TALON_ID);

  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftLeader, leftFollower);

  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightLeader, rightFollower);

  DifferentialDrive robotDrive;

  AHRS navx = new AHRS(Port.kMXP, (byte) 200);

  DifferentialDriveOdometry odometry;

  Encoder leftEncoder = new Encoder(Constants.LEFT_ENCODER_PORTS[0], Constants.LEFT_ENCODER_PORTS[1]);
  Encoder rightEncoder = new Encoder(Constants.RIGHT_ENCODER_PORTS[0], Constants.RIGHT_ENCODER_PORTS[1]);

  public DriveTrain() {

    // leftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    // rightLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    // rightLeader.setSensorPhase(true);
    rightEncoder.setReverseDirection(true);


    robotDrive = new DifferentialDrive(leftMotors, rightMotors);
    leftEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
    
    Arrays.asList(leftLeader, rightLeader, leftFollower, rightFollower)
        .forEach((WPI_TalonSRX talon) -> talon.setNeutralMode(NeutralMode.Brake));

  }

  public Pose2d getPose () {
    return odometry.getPoseMeters();
  }

  public void go () {
    leftLeader.set(ControlMode.PercentOutput, 0.7);
    rightLeader.set(ControlMode.PercentOutput, 0.7);
  }

  public void arcadeDrive (double speed, double rotation) {
    robotDrive.arcadeDrive(speed, rotation);
  }

  public void tankDriveVolts (double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);// make sure right is negative becuase sides are opposite
    robotDrive.feed();
  }

  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  public double getLeftEncoderDistance() {
    return leftEncoder.getDistance();
  }

  public double getRightEncoderDistance() {
    return rightEncoder.getDistance();
  }

  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360) * -1; // -1 here for unknown reason look in documatation
  }

  public void resetHeading () {
    navx.reset();
  }

  public void resetEncoders () {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void resetOdometry (Pose2d pose) {
    resetEncoders();
    resetHeading();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));

  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds () {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  @Override
  public void periodic() {
    //leftEncoder.update();
    //rightEncoder.update();
    odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(), rightEncoder.getDistance());
    // This method will be called once per scheduler run
  }
}
