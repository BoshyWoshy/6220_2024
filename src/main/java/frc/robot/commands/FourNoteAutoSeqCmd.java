// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourNoteAutoSeqCmd extends SequentialCommandGroup {
  /** Creates a new FourNoteAutoSeqCmd. */
  public FourNoteAutoSeqCmd(Swerve s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    Command toRightNote = AutoBuilder.pathfindToPose(new Pose2d(2.15, 4.1, Rotation2d.fromDegrees(-180)), AutoConstants.constraints); // Go to right note
    Command toLeftNote = AutoBuilder.pathfindToPose(new Pose2d(2.95, 6.25, Rotation2d.fromDegrees(95)), AutoConstants.constraints); // Go to left note
    //addCommands(new SpeakerCommand(s_Swerve), new IntakeCommand(s_Swerve);
    // addvghCommands(toRightNote, toLeftNote);
    // SpeakerCommand.andThen(IntakeCommand);
    // IntakeCommand.andThen(toRIghtNote);
    // toRightNote.andThen(IntakeCommand);
    // IntakeCommand.andThen(toLeftNote);
    AutoBuilder.pathfindToPose(new Pose2d(2.15, 4.1, Rotation2d.fromDegrees(-180)), AutoConstants.constraints).andThen(AutoBuilder.pathfindToPose(new Pose2d(2.95, 6.25, Rotation2d.fromDegrees(95)), AutoConstants.constraints));
    System.out.println("hi");
  }
}
