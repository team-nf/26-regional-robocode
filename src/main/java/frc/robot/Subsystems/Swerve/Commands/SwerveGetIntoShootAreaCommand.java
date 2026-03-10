// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve.Commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.States.SwerveStates.SwerveState;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveGetIntoShootAreaCommand extends ParallelCommandGroup {
  /** Creates a new SwerveGetIntoShootArea. */

  private CommandSwerveDrivetrain drivetrain;
  private Pose2d hubAimPose;
  private boolean isBlueAlliance;

  public SwerveGetIntoShootAreaCommand(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    isBlueAlliance = !drivetrain.isRedAlliance();

    hubAimPose = isBlueAlliance
        ? new Pose2d(4.61, 4.1, Rotation2d.kZero)
        : new Pose2d(11.92, 4.1, Rotation2d.kZero);


    Command pathFindToShootingPose = new DeferredCommand(() -> AutoBuilder.pathfindToPose(calculateShootingPose().get(), DriveConstants.PATH_CONSTRAINTS_TO_POSE)
                                        , Set.of(drivetrain));

    addCommands(
      pathFindToShootingPose
          .alongWith(new RunCommand(() -> {
            drivetrain.updateSwerveErrors(calculateShootingPose().get());
            drivetrain.setSwerveState(SwerveState.PATH_FOLLOWING);
        }))
      .until(isAtShootingArea()));
  }

  public Supplier<Pose2d> calculateShootingPose() {
    return () -> {
      Pose2d robotPose = drivetrain.getPose();
      double robotAngleToHub = Math.atan2(hubAimPose.getY() - robotPose.getY(), hubAimPose.getX() - robotPose.getX());

      double poseX = robotPose.getX();
      double poseY = robotPose.getY(); 

      if(isBlueAlliance) {
        if(poseX < Dimensions.BLUE_SHOOT_AREA_X_MIN.in(Meters)) {
          poseX = Dimensions.BLUE_SHOOT_AREA_X_MIN.in(Meters);
        }
        else if(poseX > Dimensions.BLUE_SHOOT_AREA_X_MAX.in(Meters)) {
          poseX = Dimensions.BLUE_SHOOT_AREA_X_MAX.in(Meters);
        }

        if(poseY < Dimensions.BLUE_SHOOT_AREA_Y_MIN.in(Meters)) {
          poseY = Dimensions.BLUE_SHOOT_AREA_Y_MIN.in(Meters);
        }
        else if(poseY > Dimensions.BLUE_SHOOT_AREA_Y_MAX.in(Meters)) {
          poseY = Dimensions.BLUE_SHOOT_AREA_Y_MAX.in(Meters);
        }
      }
      else {
        if(poseX < Dimensions.RED_SHOOT_AREA_X_MIN.in(Meters)) {
          poseX = Dimensions.RED_SHOOT_AREA_X_MIN.in(Meters);
        }
        else if(poseX > Dimensions.RED_SHOOT_AREA_X_MAX.in(Meters)) {
          poseX = Dimensions.RED_SHOOT_AREA_X_MAX.in(Meters);
        }

        if(poseY < Dimensions.RED_SHOOT_AREA_Y_MIN.in(Meters)) {
          poseY = Dimensions.RED_SHOOT_AREA_Y_MIN.in(Meters);
        }
        else if(poseY > Dimensions.RED_SHOOT_AREA_Y_MAX.in(Meters)) {
          poseY = Dimensions.RED_SHOOT_AREA_Y_MAX.in(Meters);
        }
      }

      return new Pose2d(
          poseX, 
          poseY, 
          new Rotation2d(robotAngleToHub)
        );
    };
  }

  public BooleanSupplier isAtShootingArea() {
    return () -> {
      Pose2d robotPose = drivetrain.getPose();
      if(isBlueAlliance) {
        return robotPose.getX() > Dimensions.BLUE_SHOOT_AREA_X_MIN.in(Meters) &&
               robotPose.getX() < Dimensions.BLUE_SHOOT_AREA_X_MAX.in(Meters) &&
               robotPose.getY() > Dimensions.BLUE_SHOOT_AREA_Y_MIN.in(Meters) &&
               robotPose.getY() < Dimensions.BLUE_SHOOT_AREA_Y_MAX.in(Meters);
      }
      else {
        return robotPose.getX() > Dimensions.RED_SHOOT_AREA_X_MIN.in(Meters) &&
               robotPose.getX() < Dimensions.RED_SHOOT_AREA_X_MAX.in(Meters) &&
               robotPose.getY() > Dimensions.RED_SHOOT_AREA_Y_MIN.in(Meters) &&
               robotPose.getY() < Dimensions.RED_SHOOT_AREA_Y_MAX.in(Meters);
      }
    };
  }

}
