/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.hal.SimDouble;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// simulation classes
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.Field2d;

import frc.robot.Constants;
import frc.robot.simulation.AHRSSimWrapper;

public class DriveTrain extends SubsystemBase
{
  WPI_TalonSRX leftLeader = new WPI_TalonSRX(Constants.LEADER_LEFT_TALON_ID);
  WPI_TalonSRX rightLeader = new WPI_TalonSRX(Constants.LEADER_RIGHT_TALON_ID);
  WPI_TalonSRX leftFollower = new WPI_TalonSRX(Constants.FOLLOWER_LEFT_TALON_ID);
  WPI_TalonSRX rightFollower = new WPI_TalonSRX(Constants.FOLLOWER_RIGHT_TALON_ID);

  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftLeader, leftFollower);

  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightLeader, rightFollower);

  DifferentialDrive robotDrive;

  AHRSSimWrapper navx = null;

  DifferentialDriveOdometry odometry;

  Encoder leftEncoder = new Encoder(Constants.LEFT_ENCODER_PORTS[0], Constants.LEFT_ENCODER_PORTS[1]);
  Encoder rightEncoder = new Encoder(Constants.RIGHT_ENCODER_PORTS[0], Constants.RIGHT_ENCODER_PORTS[1]);

  // Simulation classes
  public DifferentialDrivetrainSim drivetrainSimulator;
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;
  // The Field2d class simulates the field in the sim GUI. Note that we can have only one
  // instance!
  // Does this belong somewhere else??
  private Field2d fieldSim;
  // This is needed set the value of the angle, since there is no 
  //   equivalent method in the normal class
  private SimDouble gyroAngleSim;

  public DriveTrain() {

    // leftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    // rightLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    // rightLeader.setSensorPhase(true);
    rightEncoder.setReverseDirection(true);

    robotDrive = new DifferentialDrive(leftMotors, rightMotors);
    
    leftEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);

    navx = new AHRSSimWrapper(SPI.Port.kMXP, (byte) 200);
    
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
    
    Arrays.asList(leftLeader, rightLeader, leftFollower, rightFollower)
        .forEach((WPI_TalonSRX talon) -> talon.setNeutralMode(NeutralMode.Brake));

    if (RobotBase.isSimulation()) {
      // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      drivetrainSimulator = new DifferentialDrivetrainSim(
            Constants.kDrivetrainPlant,
            Constants.kDriveGearbox,
            Constants.kDriveGearing,
            Constants.kTrackwidth,
            Constants.kWheelDiameterMeters / 2.0);

      // The encoder and gyro angle sims let us set simulated sensor readings
      leftEncoderSim = new EncoderSim(leftEncoder);
      rightEncoderSim = new EncoderSim(rightEncoder);

      // get the angle simulation variable
      // SimDevice is found by name and index, like "name[index]"
      gyroAngleSim = new SimDeviceSim("AHRS[" + SPI.Port.kMXP.value + "]").getDouble("Angle");

      // the Field2d class lets us visualize our robot in the simulation GUI.
      fieldSim = new Field2d();
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getHeading() {
    return odometry.getPoseMeters().getRotation().getDegrees();
  }

  public void setPose(Pose2d pose) {
    // The left and right encoders MUST be reset when odometry is reset
    leftEncoder.reset();
    rightEncoder.reset();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));

    if (RobotBase.isSimulation()) {
      fieldSim.setRobotPose(pose);
    }
  }
  
  public void arcadeDrive(double speed, double rotation) {
    SmartDashboard.putNumber("Drive Cmd Throttle", speed);
    SmartDashboard.putNumber("Drive Cmd Turn", rotation);

    robotDrive.arcadeDrive(speed, rotation);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts); // make sure right is negative because sides are opposite
    robotDrive.feed();
  }

  public double getLeftEncoderDistance() {
    return leftEncoder.getDistance();
  }

  public double getRightEncoderDistance() {
    return rightEncoder.getDistance();
  }

  // private: not needed outside this class. Use the Pose angle for the robot heading
  private double getGyroAngle() {   
    // Note gyro angle goes opposite to the field angle, thus the minus sign
    return -Math.IEEEremainder(navx.getAngle(), 360.0);
  }

  // this is used in the Ramsete controllers
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(Rotation2d.fromDegrees(getGyroAngle()), leftEncoder.getDistance(), rightEncoder.getDistance());
    SmartDashboard.putNumber("Heading", getHeading());
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    drivetrainSimulator.setInputs(leftMotors.get() * RobotController.getBatteryVoltage(),
                                  -rightMotors.get() * RobotController.getBatteryVoltage());
    drivetrainSimulator.update(0.020);
  
    leftEncoderSim.setDistance(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kLeftPosition));
    leftEncoderSim.setRate(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kLeftVelocity));
    
    rightEncoderSim.setDistance(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kRightPosition));
    rightEncoderSim.setRate(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kRightVelocity));

    gyroAngleSim.set(-drivetrainSimulator.getHeading().getDegrees());

    fieldSim.setRobotPose(getPose());
  }

}
