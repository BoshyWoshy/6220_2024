// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants.*;
// import frc.robot.subsystems.Swerve;

// public class FourNoteAutoCmd extends Command {
//   private Swerve s_Swerve;
//   private boolean hasSent = false;
//   /** Creates a new FourNoteAutoCmd. */
//   public FourNoteAutoCmd(Swerve s_Swerve) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.s_Swerve = s_Swerve;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     //new SpeakerCommand(s_Swerve)
//     //new IntakeCommand(s_Swerve) //MID NOTE
//     //new SpeakerCommand(s_Swerve)
//     AutoBuilder.pathfindToPose(new Pose2d(2.15, 4.1, Rotation2d.fromDegrees(-180)), AutoConstants.constraints); // Go to right note
//     // Equvilant to:   
//     //AutoBuilder.followPath(
//     //  new PathPlannerPath(
//     //       PathPlannerPath.bezierFromPoses(s_Swerve.getPose(), new Pose2d(2.15, 4.1, Rotation2d.fromDegrees(-117.92))), // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
//     //       constraints, 
//     //      new GoalEndState(0.0, Rotation2d.fromDegrees(180)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.)
//     //    )
//     //),

//     //new IntakeCommand(s_Swerve))
//     //new SpeakerCommand(s_Swerve)
//     AutoBuilder.pathfindToPose(new Pose2d(2.95, 6.25, Rotation2d.fromDegrees(95)), AutoConstants.constraints); // Go to left note
//     //new IntakeCommand(s_Swerve))
//     //new SpeakerCommand(s_Swerve)
//     hasSent = true;
//   }

//   //   public SequentialCommandGroup fourNoteAuto()
//   // {
//   //     hasSent = true;
//   //     // final List<Translation2d> bezierPointsToRightNote = PathPlannerPath.bezierFromPoses(
//   //     //     s_Swerve.getPose(),
//   //     //     new Pose2d(3.0, 4.85, Rotation2d.fromDegrees(0))
//   //     // );

//   //     // final PathPlannerPath pathToRightNote = new PathPlannerPath(
//   //     //     bezierPointsToRightNote, 
//   //     //     new PathConstraints(SwerveConstants.maxSpeed, 5, SwerveConstants.maxAngularVelocity, SwerveConstants.turnMaxAccel), 
//   //     //     new GoalEndState(0.0, Rotation2d.fromDegrees(90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
//   //     // );
      
//   //     // pathToRightNote.preventFlipping = false;

//   //     // AutoBuilder.followPath(
//   //     //       //Go to right note
//   //     //       new PathPlannerPath(
//   //     //         PathPlannerPath.bezierFromPoses(s_Swerve.getPose(), new Pose2d(2.15, 4.1, Rotation2d.fromDegrees(-117.92))), // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
//   //     //         constraints,
//   //     //         new GoalEndState(0.0, Rotation2d.fromDegrees(180)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.)
//   //     //       ),


//   //     PathConstraints constraints = new PathConstraints(SwerveConstants.maxSpeed, 5, SwerveConstants.maxAngularVelocity, SwerveConstants.turnMaxAccel);
//   //     SequentialCommandGroup test = new SequentialCommandGroup(
//   //       //new SpeakerCommand(s_Swerve),
//   //       //new IntakeCommand(s_Swerve)), //MID NOTE
//   //       //new SpeakerCommand(s_Swerve),
//   //       AutoBuilder.pathfindToPose(new Pose2d(2.15, 4.1, Rotation2d.fromDegrees(-180)), constraints), // Go to right note
//   //       // Equvilant to:   
//   //       //AutoBuilder.followPath(
//   //       //  new PathPlannerPath(
//   //       //       PathPlannerPath.bezierFromPoses(s_Swerve.getPose(), new Pose2d(2.15, 4.1, Rotation2d.fromDegrees(-117.92))), // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
//   //       //       constraints, 
//   //       //      new GoalEndState(0.0, Rotation2d.fromDegrees(180)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.)
//   //       //    )
//   //       //),

//   //       //new IntakeCommand(s_Swerve)),
//   //       //new SpeakerCommand(s_Swerve),
//   //       AutoBuilder.pathfindToPose(new Pose2d(2.95, 6.25, Rotation2d.fromDegrees(95)), constraints) // Go to left note
//   //       //new IntakeCommand(s_Swerve)),
//   //       //new SpeakerCommand(s_Swerve),
//   //     );
//   //     return test;
//   // }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return hasSent;
//   }
// }
