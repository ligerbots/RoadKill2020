/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...
  public final DriveTrain robotDrive = new DriveTrain();
  
  private final Throttle throttle = new Throttle();
  private final Turn turn = new Turn();
  private final DriveCommand driveCommand = new DriveCommand(robotDrive, throttle, turn);
  XboxController xbox = new XboxController(0);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    robotDrive.setDefaultCommand(driveCommand);

    // Configure the initial Pose (field position, angle) of the robot
    //  and tell the DriveTrain where it is.
    // In a competition robot, this might be determined by a Chooser
    Rotation2d initialAngle = new Rotation2d(Math.toRadians(30));  // facing straight forward
    Translation2d initialPosition = new Translation2d(Units.feetToMeters(10), Units.feetToMeters(20));
    Pose2d initialPose = new Pose2d(initialPosition, initialAngle);
  
    robotDrive.setPose(initialPose);
  }

  public class Throttle implements DoubleSupplier {
    @Override
    public double getAsDouble() {
      return -xbox.getY(Hand.kLeft);
    }
  }

  public class Turn implements DoubleSupplier {
    @Override
    public double getAsDouble() {
      return xbox.getX(Hand.kRight);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton xboxA = new JoystickButton(xbox, 1);
    xboxA.whenPressed(new go(robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand()
  {
    return new Path1Auto(robotDrive);
    //return new DriveForwardAuto(robotDrive);
  }
}
