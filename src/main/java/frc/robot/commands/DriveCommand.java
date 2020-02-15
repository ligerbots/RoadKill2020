/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain robotDrive;
  private final DoubleSupplier throttle;
  private final DoubleSupplier turn;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveTrain driveTrain, DoubleSupplier throttle, DoubleSupplier turn) {    
    robotDrive = driveTrain;
    this.throttle = throttle;
    this.turn = turn;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //robotDrive.go();
    SmartDashboard.putNumber("Left Encoder Distance", robotDrive.getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoder Distance", robotDrive.getRightEncoderDistance());
    SmartDashboard.putNumber("Heading", robotDrive.getHeading());
    robotDrive.arcadeDrive(throttle.getAsDouble(), turn.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
