// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class TrajectoryCalculationCmd extends Command {
  /** Creates a new TrajectoryCalculationCmd. */
  private Swerve s_Swerve;
  private double startVelocityMetersPerSec, endVelocityMetersPerSec;
  private List<Pose2d> waypoints;
  private boolean reversed;
  private Trajectory trajectory;
  private final Timer timer = new Timer();
  public TrajectoryCalculationCmd(Swerve s_Swerve, double startVelocityMetersPerSec, List<Pose2d> waypoints, double endVelocityMetersPerSec, boolean reversed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    this.startVelocityMetersPerSec = startVelocityMetersPerSec;
    this.endVelocityMetersPerSec = endVelocityMetersPerSec;
    this.waypoints = waypoints;
    this.reversed = reversed;
    addRequirements(s_Swerve);

    TrajectoryConfig config = new TrajectoryConfig(SwerveConstants.maxSpeed, 5)
    .setKinematics(SwerveConstants.swerveKinematics).setEndVelocity(startVelocityMetersPerSec).setEndVelocity(endVelocityMetersPerSec).setReversed(reversed);
  
    trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
