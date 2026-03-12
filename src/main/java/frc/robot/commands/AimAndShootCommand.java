// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.TheMachine;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.PoseConstants;
import frc.robot.constants.States.SwerveStates.SwerveState;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.Container;
import frc.robot.utils.ShooterCalculator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndShootCommand extends Command {

  private double MaxSpeed = 0.2 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private CommandXboxController driverController;
  private CommandSwerveDrivetrain swerveDrivetrain;
  private TheMachine theMachine;

  private Pose2d hubAimPose;


  private double[] prevErrors = new double[10];
  private double averageError = 0.0;

  /** Creates a new AimAndPass. */
  public AimAndShootCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick, TheMachine theMachine) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrivetrain = drivetrain;
    this.driverController = joystick;
    this.theMachine = theMachine;

    drive.HeadingController.setPID(DriveConstants.AIMED_DRIVING_kP, DriveConstants.AIMED_DRIVING_kI, DriveConstants.AIMED_DRIVING_KD);

    addRequirements(drivetrain);
    addRequirements(theMachine.getSubsystems());
    
    if(Container.isBlue)
    {
      hubAimPose = PoseConstants.BLUE_HUB_AIM_POSE;
    }
    else
    {
      hubAimPose = PoseConstants.RED_HUB_AIM_POSE;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    prevErrors = new double[15];
    for (int i = 0; i < prevErrors.length; i++) {
      prevErrors[i] = 1.0;
    }

    if(Container.isBlue)
    {
      hubAimPose = PoseConstants.BLUE_HUB_AIM_POSE;
    }
    else
    {
      hubAimPose = PoseConstants.RED_HUB_AIM_POSE;
    }
  }

  private Pose2d robotPose = new Pose2d();
  double robotAngleToHub =  0;
  double angleError = 10;

  ChassisSpeeds speeds;
  double distance;
  double time;
  double aimX;
  double aimY;

  double[] shootParams;
  double velocityRPS;
  double hoodAngle;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    robotPose = swerveDrivetrain.getPose();
    speeds = swerveDrivetrain.getFieldSpeeds();
    distance = hubAimPose.getTranslation().getDistance(robotPose.getTranslation());
    time = ShooterCalculator.flightTimeOfFuelFormula(distance);


    aimX = hubAimPose.getX() + (speeds.vxMetersPerSecond * time) * (Container.isBlue ? -1 : 1);
    aimY = hubAimPose.getY() + (speeds.vyMetersPerSecond * time) * (Container.isBlue ? -1 : 1);

    robotAngleToHub = Math.atan2(aimY - robotPose.getY(), aimX - robotPose.getX());
    angleError = robotAngleToHub - robotPose.getRotation().getRadians();

    if(!Container.isBlue) robotAngleToHub += Math.PI;

    swerveDrivetrain.setControl(
        drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withTargetDirection(new Rotation2d(robotAngleToHub))// Drive counterclockwise with negative X (left)
    );

    shootParams = ShooterCalculator.calculateShootingParameters(speeds, robotPose, time);
    velocityRPS = shootParams[0];
    hoodAngle = shootParams[1];

    if (Math.abs(angleError) < DriveConstants.AIMING_TOLERANCE_RADIANS) {

      if(theMachine.isShooterReady()) {
        theMachine.shoot(velocityRPS, hoodAngle);
      } else {
        theMachine.getReady(velocityRPS, hoodAngle);
      }
    } else {
      theMachine.getReady(velocityRPS, hoodAngle);
    }

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
