// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
// import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AmpTestCmd extends Command {

  // final PhotonVisionSubsystem m_VisionSubsystem;

  // private final ArmSubsystem armSubsystem;
  // private final ShooterSubsystem shooterSubsystem;
  // private final IntakeSubsystem intakeSubsystem;

  // private final ProfiledPIDController limelightPidController; //Comment this out in case if we don't want to use limelight
  private final ProfiledPIDController leftAndRightPID, fowardAndBackPID;// from the driver's point of view and 0,0 is at the right hand side of the driver

  private final Supplier<Boolean> shootSupplier; // Use this so that it's the driver click the button for it to shoot.

  // private final TunableNumber LimelightAmpkP = new TunableNumber("Limelight ampkP", 0);
  // private final TunableNumber LimelightAmpkI = new TunableNumber("Limelight ampkI", 0);
  // private final TunableNumber LimelightAmpkD = new TunableNumber("Limelight ampkD", 0);
  // private final TunableNumber LimelightampTolerance = new TunableNumber("Limelight ampTolerance", 3);
  
  //TODO: MAKE ALL OF THESE TUNABLE NUMBERS INTO CONSTANTS.
  private final TunableNumber kP = new TunableNumber(" ampkP", 0);
  private final TunableNumber kI = new TunableNumber(" ampkI", 0);
  private final TunableNumber kD = new TunableNumber(" ampkD", 0);
  private final TunableNumber Vel = new TunableNumber(" MaxVel", 0.25);
  private final TunableNumber Accel = new TunableNumber(" Accel", 1);
  private final TunableNumber Tolerance = new TunableNumber(" Tolerance", 3);
  
  // private final TunableNumber kP = new TunableNumber("Left and right ampkP", 0);
  // private final TunableNumber kI = new TunableNumber("leftAndRightAmp ampkI", 0);
  // private final TunableNumber kD = new TunableNumber("leftAndRightAmp ampkD", 0);
  // private final TunableNumber Vel = new TunableNumber("leftAndRightAmp MaxVel", 0.25);
  // private final TunableNumber Accel = new TunableNumber("leftAndRightAmp Accel", 1);
  // private final TunableNumber leftAndRightAmpTolerance = new TunableNumber("leftAndRightamp Tolerance", 3);

  private Swerve s_Swerve;

  /** Creates a new AmpTestCmd. */
  public AmpTestCmd(Swerve s_Swerve, Supplier<Boolean> shootSupplier) {
    // limelightPidController = new PIDController(LimelightAmpkP.get(),LimelightAmpkI.get(),LimelightAmpkD.get());
    // limelightPidController.setTolerance(turnTolerance.get());
    // limelightPidController.setIZone(4);

    fowardAndBackPID = new ProfiledPIDController(kP.get(), kI.get(),kD.get(), new TrapezoidProfile.Constraints(Vel.get(), Accel.get()));
    fowardAndBackPID.setTolerance(Tolerance.get());
    // fowardAndBackPID.setIZone();//Not sure if we need this or not
    
    leftAndRightPID = new ProfiledPIDController(kP.get(), kI.get(), kD.get(),  new TrapezoidProfile.Constraints(Vel.get(), Accel.get()));
    leftAndRightPID.setTolerance(Tolerance.get());
    // leftAndRightPID.setIZone();//Not sure if we need this or not
    
    this.s_Swerve = s_Swerve;
    // armSubsystem = ArmSubsystem.getInstance();
    // shooterSubsystem = ShooterSubsystem.getInstance();
    // intakeSubsystem = IntakeSubsystem.getInstance();
    // m_VisionSubsystem = PhotonVisionSubsystem.getInstance();
    this.shootSupplier = shootSupplier;
    addRequirements(s_Swerve);
    // addRequirements(m_VisionSubsystem, armSubsystem,shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.resetTurnController();
    fowardAndBackPID.reset(s_Swerve.getPose().getX());
    leftAndRightPID.reset(s_Swerve.getPose().getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("dringing arm to goal");// armSubsystem.driveToGoal(armSubsystem.armAmpAngle.get());
    SmartDashboard.putNumber("heading swerve", s_Swerve.getHeadingDegrees());
    fowardAndBackPID.setGoal(s_Swerve.getForwardBackwardToAmp());
    leftAndRightPID.setGoal(s_Swerve.getLeftAndRightToAmp());
    SmartDashboard.putNumber("x setpoint", fowardAndBackPID.getSetpoint().position);
    SmartDashboard.putNumber("y setpoint", leftAndRightPID.getSetpoint().position);
    double xOutput = fowardAndBackPID.calculate(s_Swerve.getPose().getX());
    double yOutput = leftAndRightPID.calculate(s_Swerve.getPose().getY());
    s_Swerve.setAutoTurnHeading(90);
    double rotationVal =s_Swerve.getTurnPidSpeed();
    // int invert =  (Constants.isRed) ? -1 : 1; //I don't think we need this
    s_Swerve.drive(
                new Translation2d(xOutput, yOutput), //.times(SwerveConstants.maxSpeed *invert) //I don't think we need this neither 
                rotationVal, 
                true, 
                true
            );
      if(fowardAndBackPID.atGoal() && leftAndRightPID.atGoal() && shootSupplier.get()) //&& armSubsystem.isAtGoal() 
      {
        System.out.println("shooter & intake move");
        // shooterSubsystem.spinManually(ArmConstants.ampShooterSpeed);
        // intakeSubsystem.feedAmp();
      } else {
        // shooterSubsystem.stop();
        // intakeSubsystem.stop();
      }

      if(kP.hasChanged()
        || kI.hasChanged()
        || kD.hasChanged()) {
            fowardAndBackPID.setPID(kP.get(), kI.get(), kD.get());
            leftAndRightPID.setPID(kP.get(), kI.get(), kD.get());
            leftAndRightPID.reset(s_Swerve.getPose().getY());
            fowardAndBackPID.reset(s_Swerve.getPose().getX());
        }
        if(Vel.hasChanged() || Accel.hasChanged()) {
            fowardAndBackPID.setConstraints(new TrapezoidProfile.Constraints(Vel.get(), Accel.get()));
            leftAndRightPID.setConstraints(new TrapezoidProfile.Constraints(Vel.get(), Accel.get()));
            leftAndRightPID.reset(s_Swerve.getPose().getY());
            fowardAndBackPID.reset(s_Swerve.getPose().getX());
        }

        // if(kP.hasChanged()
        // || kI.hasChanged()
        // || kD.hasChanged()) {
        //     leftAndRightPID.setPID(kP.get(), kI.get(), kD.get());
        //     leftAndRightPID.reset(s_Swerve.getPose().getY());
        // }
        // if(Vel.hasChanged() || Accel.hasChanged()) {
        //     leftAndRightPID.setConstraints(new TrapezoidProfile.Constraints(Vel.get(), Accel.get()));
        //     leftAndRightPID.reset(s_Swerve.getPose().getY());
        // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // armSubsystem.stop();
    // shooterSubsystem.stop();
    // intakeSubsystem.stop();
  }

  
}
