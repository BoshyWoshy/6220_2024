// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;


public class LimelightAssistedSwerveCmd extends Command {
  private final VisionSubsystem m_VisionSubsystem;
  private final Swerve s_Swerve;
  private final PIDController limelightPidController;
  public LimelightAssistedSwerveCmd(Swerve s_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_VisionSubsystem = VisionSubsystem.getInstance();
    limelightPidController = new PIDController(0.05,0.0,0.0);
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double steerOutput = limelightPidController.calculate(s_Swerve.getHeadingDegrees(),m_VisionSubsystem.getSteeringOffset()) * SwerveConstants.maxAngularVelocity;
    steerOutput = (steerOutput > SwerveConstants.maxAngularVelocity) ? SwerveConstants.maxAngularVelocity : steerOutput;
    s_Swerve.drive(
            new Translation2d(0d, 0d), 
            // MathUtil.applyDeadband(limelightPidController.calculate(s_Swerve.getHeadingDegrees(),m_VisionSubsystem.getSteeringOffset()), 0), //NOT USABLE UPGRADED
            steerOutput,
            // (s_Swerve.getHeadingDegrees() - m_VisionSubsystem.getSteeringOffset()) * SwerveConstants.maxAngula15/rVelocity, //Original code from TeleopSwerve, NOT USABLE
            true, 
            true
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
