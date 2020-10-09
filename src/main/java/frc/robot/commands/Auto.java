/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class Auto extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public Auto(DriveTrain robotDrive) {
    // hello world
    robotDrive.resetOdometry(new Pose2d());
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    TrajectoryConfig configForward =
        new TrajectoryConfig(Constants.kMaxSpeed,
                             Constants.kMaxAcceleration)
            .setKinematics(Constants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);
        
    TrajectoryConfig configBackward =
        new TrajectoryConfig(Constants.kMaxSpeed,
                             Constants.kMaxAcceleration)
            .setKinematics(Constants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint)
            .setReversed(true);

    Trajectory trajectoryForward = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(
            new Translation2d(0.5, 0.),
            new Translation2d(-0.5, 0)
        ),
        new Pose2d(6, 0, new Rotation2d(0)),
        configForward
    ); 

    Trajectory trajectoryBack = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        //new Pose2d(6, 0, new Rotation2d(0)), 
        List.of(),
        //new Pose2d(0, 0, new Rotation2d(0)),
        configBackward
    ); 

    RamseteCommand ramseteCommand1 = new RamseteCommand(
        trajectoryForward,
        robotDrive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
                                   Constants.kvVoltSecondsPerMeter,
                                   Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        robotDrive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        robotDrive::tankDriveVolts,
        robotDrive
    );

    RamseteCommand ramseteCommand2 = new RamseteCommand(
        trajectoryBack,
        robotDrive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
                                   Constants.kvVoltSecondsPerMeter,
                                   Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        robotDrive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        robotDrive::tankDriveVolts,
        robotDrive
    );

    

    addCommands(
      ramseteCommand1, ramseteCommand2.andThen(() -> robotDrive.tankDriveVolts(0, 0))
    );
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
