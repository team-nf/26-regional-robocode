// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve.Commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.States.SwerveStates.SwerveState;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveAimToPass extends Command {
  /** Creates a new SwerveAimToHub. */

  private final CommandSwerveDrivetrain swerveDrivetrain;

  private PIDController aimingPID;

  private Pose2d passAimPose;

  private double MaxAngularRate = DriveConstants.AIMING_MAX_ANGULAR_VELOCITY.in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private double[] prevErrors = new double[10];
  private double averageError = 0.0;

  private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public SwerveAimToPass(CommandSwerveDrivetrain swerveDrivetrain) {
    this.swerveDrivetrain = swerveDrivetrain;

    aimingPID = swerveDrivetrain.getAimingPID();
    addRequirements(swerveDrivetrain);



    //hubAimPose = new Pose2d(4.61, 4.1, new Rotation2d());

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aimingPID.reset();
    averageError = 1.0;

    prevErrors = new double[15];
    for (int i = 0; i < prevErrors.length; i++) {
      prevErrors[i] = 1.0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = swerveDrivetrain.getPose();

    double goalAngle;

    if(swerveDrivetrain.isRedAlliance())
    {
       goalAngle = Math.toRadians(0);
    }
    else
    {
      goalAngle = Math.toRadians(180);
    }

    double angleError = goalAngle - robotPose.getRotation().getRadians();

    // Normalize angle error to the range [-pi, pi]
    angleError = Math.atan2(Math.sin(angleError), Math.cos(angleError));
    double output = aimingPID.calculate(-angleError);

    swerveDrivetrain.updateSwerveErrors(robotPose.plus(new Transform2d(0.0, 0.0, new Rotation2d(averageError))));
    swerveDrivetrain.setSwerveState(SwerveState.AIMING);
    swerveDrivetrain.updateSwerveData();
    
    swerveDrivetrain.setControl(drive
      .withRotationalRate(output)
    );

    for (int i = prevErrors.length - 1; i > 0; i--) {
      prevErrors[i] = prevErrors[i - 1];
    }
    prevErrors[0] = Math.abs(angleError);

    double errorSum = 0.0;
    for (double error : prevErrors) {
      errorSum += error;
    }
    averageError = errorSum / prevErrors.length;

    swerveDrivetrain.setIsAimed(averageError < DriveConstants.AIMING_TOLERANCE_RADIANS*3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrivetrain.setIsAimed(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
