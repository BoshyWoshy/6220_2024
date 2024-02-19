package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
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

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve, 
            () -> OIConstants.modifyMoveAxis(-driver.getRawAxis(translationAxis)), 
            () -> OIConstants.modifyMoveAxis(-driver.getRawAxis(strafeAxis)), 
            () -> OIConstants.modifyRotAxis(-driver.getRawAxis(rotationAxis)), 
            () -> robotCentric.getAsBoolean()
        )
    );

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();
    
  }

  private void configureButtonBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    zeroOdometry.onTrue(new InstantCommand(() -> s_Swerve.setPose(new Pose2d(new Translation2d(15.3, 5.55), new Rotation2d(0)))));
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

}
