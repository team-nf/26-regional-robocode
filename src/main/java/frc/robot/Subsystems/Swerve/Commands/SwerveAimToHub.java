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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.States.SwerveStates.SwerveState;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveAimToHub extends Command {
  /** Creates a new SwerveAimToHub. */

  private final CommandSwerveDrivetrain swerveDrivetrain;

  private PIDController aimingPID;

  private Pose2d hubAimPose;

  private double[] prevErrors = new double[15];
  private double averageError = 0.0;

  private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public SwerveAimToHub(CommandSwerveDrivetrain swerveDrivetrain) {
    this.swerveDrivetrain = swerveDrivetrain;

    aimingPID = swerveDrivetrain.getAimingPID();
    addRequirements(swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aimingPID.reset();
    averageError = 1.0;

    if(swerveDrivetrain.isRedAlliance())
    {
      hubAimPose = PoseConstants.RED_HUB_AIM_POSE;
    }
    else
    {
      hubAimPose = PoseConstants.BLUE_HUB_AIM_POSE;
    }

    prevErrors = new double[15];
    for (int i = 0; i < prevErrors.length; i++) {
      prevErrors[i] = 1.0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = swerveDrivetrain.getPose();
    double robotAngleToHub = Math.atan2(hubAimPose.getY() - robotPose.getY(), hubAimPose.getX() - robotPose.getX());

    double angleError = robotAngleToHub - robotPose.getRotation().getRadians();

    // Normalize angle error to the range [-pi, pi]
    angleError = Math.atan2(Math.sin(angleError), Math.cos(angleError));
    double output = aimingPID.calculate(-angleError);

    swerveDrivetrain.updateSwerveErrors(robotPose.plus(new Transform2d(0.0, 0.0, Rotation2d.fromRadians(averageError))));
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

    swerveDrivetrain.setIsAimed(averageError < DriveConstants.AIMING_TOLERANCE_RADIANS);
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
