package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.commands.*;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  /* Controllers */
    private final XboxController driver = new XboxController(0);

    /* Drive Controls */

    //Trying to add driver control curves
    

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton aimToAmp = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton aimToSpeaker = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton aimToNote = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton zeroOdometry = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton intake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
   //private final PhotonVisionSubsystem p_PhotonVisionSubsystem = PhotonVisionSubsystem.getInstance();


  public RobotContainer() {
    Constants.VisionConstants.setTagHeights();

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(1.35,5.53, Rotation2d.fromDegrees(0)), 
      new Pose2d(3.21, 4.83, Rotation2d.fromDegrees(90))
    );

    // bezierPoints.get(0).div(rotationAxis);

    // PathPlannerPath testPath = new Pathplanner
    // (

    // );

    
    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      new PathConstraints(SwerveConstants.maxSpeed, 10, SwerveConstants.maxAngularVelocity, SwerveConstants.turnMaxAccel),// The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
      new GoalEndState(0.0, Rotation2d.fromDegrees(90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );
    path.preventFlipping = false;

    AutoBuilder.followPath(path);
    PathPlannerPath.fromPathPoints(path.getAllPathPoints(),
          new PathConstraints(SwerveConstants.maxSpeed, 10, SwerveConstants.maxAngularVelocity, SwerveConstants.turnMaxAccel),// The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
          new GoalEndState(0.0, Rotation2d.fromDegrees(90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  );

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve, 
            () -> OIConstants.modifyMoveAxis(-driver.getRawAxis(translationAxis)), 
            () -> OIConstants.modifyMoveAxis(-driver.getRawAxis(strafeAxis)), 
            () -> OIConstants.modifyRotAxis(  driver.getRawAxis(rotationAxis)), 
            () -> robotCentric.getAsBoolean()
        )
    );

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("fourNoteAutoProMax", fourNoteAuto());
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();
    
  }

  private void configureButtonBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    zeroOdometry.onTrue(new InstantCommand(() -> s_Swerve.setPose(new Pose2d(new Translation2d(1.33, 5.60), new Rotation2d(0))))); //Blue Side
    // zeroOdometry.onTrue(new InstantCommand(() -> s_Swerve.setPose(new Pose2d(new Translation2d(15.2, 5.60), Rotation2d.fromDegrees(180))))); //Red side
    // aimToSpeaker.whileTrue(new TeleopAimSwerve(
    //     s_Swerve,
    //     () -> OIConstants.modifyMoveAxis(-driver.getRawAxis(translationAxis)), 
    //     () -> OIConstants.modifyMoveAxis(-driver.getRawAxis(strafeAxis)), 
    //     () -> s_Swerve.getHeadingToSpeaker()
    //   )
    // );
    aimToAmp.whileTrue(
      new AmpTestCmd(
        s_Swerve,
        () -> driver.getRightBumper())
      );
    // aimToAmp.whileTrue(
    //   new TeleopAimSwerve(
    //    s_Swerve,
    //     () -> OIConstants.modifyMoveAxis(-driver.getRawAxis(translationAxis)), 
    //     () -> OIConstants.modifyMoveAxis(-driver.getRawAxis(strafeAxis)), 
    //     () -> -90
    //   )
    // );
    // aimToNote.whileTrue(new ShootingTestCommand());
    // intake.whileTrue(new IntakeTest());
  }

  public Command getAutonomousCommand() {
    //s_Swerve.setPose(new Pose2d(15.3,5.55,new Rotation2d(0)));
      return autoChooser.getSelected();
  }

  
  public SequentialCommandGroup fourNoteAuto()
  {
      List<Translation2d> bezierPointsToRightNote = PathPlannerPath.bezierFromPoses(
          s_Swerve.getPose(),
          new Pose2d(2.15, 4.1, Rotation2d.fromDegrees(0))
      );

      final PathPlannerPath pathToRightNote = new PathPlannerPath(
          bezierPointsToRightNote, 
          AutoConstants.constraints, 
          new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
      );


      List<Translation2d> bezierPointsToLeftNote = PathPlannerPath.bezierFromPoses(
          s_Swerve.getPose(),
          new Pose2d(2.95, 6.25, Rotation2d.fromDegrees(0))
      );

      final PathPlannerPath pathToLeftNote = new PathPlannerPath(
          bezierPointsToLeftNote, 
          AutoConstants.constraints, 
          new GoalEndState(0.0, Rotation2d.fromDegrees(90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
      );
      

      // AutoBuilder.followPath(pathToRightNote);


    

      SequentialCommandGroup test = new SequentialCommandGroup(
        //new SpeakerCommand(s_Swerve),
        //new IntakeCommand(s_Swerve)), //MID NOTE
        //new SpeakerCommand(s_Swerve),
        AutoBuilder.followPath(pathToRightNote),
        // AutoBuilder.pathfindToPose(new Pose2d(2.15, 4.1, Rotation2d.fromDegrees(0)), AutoConstants.constraints), // Go to right note
        // Equvilant to:   
        //AutoBuilder.followPath(
        //  new PathPlannerPath(
        //       PathPlannerPath.bezierFromPoses(s_Swerve.getPose(), new Pose2d(2.15, 4.1, Rotation2d.fromDegrees(-117.92))), // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        //       constraints, 
        //      new GoalEndState(0.0, Rotation2d.fromDegrees(180)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.)
        //    )
        //),

        //new IntakeCommand(s_Swerve)),
        //new SpeakerCommand(s_Swerve),
        AutoBuilder.followPath(pathToLeftNote) // Go to left note
        //new IntakeCommand(s_Swerve)),
        //new SpeakerCommand(s_Swerve),
      );
      return test;
  }

}
