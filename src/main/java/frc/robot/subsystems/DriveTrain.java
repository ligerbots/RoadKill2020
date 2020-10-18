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
import com.kauailabs.navx.frc.AHRS;

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
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import frc.robot.Constants;
//import frc.robot.EncoderWrapper;

//@SuppressWarnings("all")
public class DriveTrain extends SubsystemBase
{
  WPI_TalonSRX leftLeader = new WPI_TalonSRX(Constants.LEADER_LEFT_TALON_ID);
  WPI_TalonSRX rightLeader = new WPI_TalonSRX(Constants.LEADER_RIGHT_TALON_ID);
  WPI_TalonSRX leftFollower = new WPI_TalonSRX(Constants.FOLLOWER_LEFT_TALON_ID);
  WPI_TalonSRX rightFollower = new WPI_TalonSRX(Constants.FOLLOWER_RIGHT_TALON_ID);

  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftLeader, leftFollower);

  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightLeader, rightFollower);

  DifferentialDrive robotDrive;

  AHRS navx = null;

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
  private SimDouble gyroAngleSim;
  private Gyro gyro = null;

  public DriveTrain() {

    // leftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    // rightLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    // rightLeader.setSensorPhase(true);
    rightEncoder.setReverseDirection(true);

    robotDrive = new DifferentialDrive(leftMotors, rightMotors);
    
    leftEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);

    if (RobotBase.isSimulation()) {
      // navX is not yet simulated, so use an different gyro
      gyro = new ADXRS450_Gyro();
    } else {
      navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    }
    
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

      // string is the name. Does the value actually matter??
      gyroAngleSim = new SimDeviceSim("ADXRS450_Gyro" + "[" + SPI.Port.kOnboardCS0.value + "]").getDouble("Angle");

      // the Field2d class lets us visualize our robot in the simulation GUI.
      fieldSim = new Field2d();
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void arcadeDrive(double speed, double rotation) {
    SmartDashboard.putNumber("Drive Cmd Throttle", speed);
    SmartDashboard.putNumber("Drive Cmd Turn", rotation);

    robotDrive.arcadeDrive(speed, rotation);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
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
    if (navx != null) {
      return Math.IEEEremainder(navx.getAngle(), 360) * -1.0; // -1 here for unknown reason look in documatation
    } else {
      return Math.IEEEremainder(gyro.getAngle(), 360) * -1.0;
    }
  }

  public void resetHeading() {
    if (navx != null) {
      navx.reset();
    } else {
      gyro.reset();
    }
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
    // This method will be called once per scheduler run

    odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(), rightEncoder.getDistance());
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putString("Pose", getPose().toString());
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
