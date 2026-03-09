// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.States.FeederStates.FeederControlState;
import frc.robot.Constants.States.HopperStates.HopperControlState;
import frc.robot.Constants.States.IntakeStates.IntakeControlState;
import frc.robot.Constants.States.ShooterStates.FlywheelState;
import frc.robot.Constants.States.ShooterStates.ShooterControlState;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Hopper.HopperSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Shooter.Utils.ShooterCalculator;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Subsystems.TheMachine.TheMachine;
import frc.robot.Utils.FuelSim;
import frc.robot.Utils.HopperSim;
import frc.robot.Utils.Localization;
import frc.robot.Utils.MatchTrackerSim;
import frc.robot.Utils.ShooterSim;
import frc.robot.Utils.SwerveFieldContactSim;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {

  private CommandSwerveDrivetrain m_swerveDrivetrain;

  private ShooterCalculator m_shooterCalculator;
  private ShooterSubsystem m_shooterSubsystem;

  private FeederSubsystem m_feederSubsystem;

  
  private HopperSubsystem m_hopperSubsystem;
  
  private IntakeSubsystem m_intakeSubsystem;
  

  private TheMachine m_theMachine;

  private final CommandXboxController m_driverController;

  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    m_feederSubsystem = new FeederSubsystem();

    m_hopperSubsystem = new HopperSubsystem();

    m_intakeSubsystem = new IntakeSubsystem();

    m_swerveDrivetrain = TunerConstants.createDrivetrain();

    m_shooterCalculator = new ShooterCalculator(m_swerveDrivetrain.swerveDataSupplier());
    m_shooterSubsystem = new ShooterSubsystem(m_shooterCalculator);

    
    m_theMachine = new TheMachine(
                            m_shooterSubsystem, 
                            m_feederSubsystem, 
                            m_hopperSubsystem, 
                            m_intakeSubsystem, 
                            m_swerveDrivetrain.swerveDataSupplier());
    

    m_driverController = new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

    configureBindings();
    if(Utils.isSimulation()) configureSims();

    boolean isCompetition = false;
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );

    SmartDashboard.putData("Conf/Auto Chooser", autoChooser);

  }

  private void configureBindings() {

    m_swerveDrivetrain.setDefaultCommand(m_swerveDrivetrain.teleopCommand(m_driverController));

    m_driverController.b()
        .onTrue(m_theMachine.reverseRequest());

    m_driverController.y()
        .whileTrue( 
          new ConditionalCommand(
              m_theMachine.testRequest()
              ,
              createHeldHubShootSequence()
              ,
              m_swerveDrivetrain::isRobotInNeutralZone
            ))
        .onFalse(m_theMachine.intakeRequest());

    m_driverController.a()
        .onTrue(m_theMachine.idleRetractedRequest());

    m_driverController.x()
        .onTrue(m_theMachine.intakeRequest());

    m_driverController.povDown()
        .onTrue(m_theMachine.zeroRequest());

    m_driverController.povLeft()
        .onTrue(m_theMachine.idleDeployedRequest());

    m_driverController.leftBumper()
        .whileTrue(createHeldPassShootSequence())
        .onFalse(m_theMachine.intakeRequest());

    m_driverController.rightBumper()
      .whileTrue(createHeldHubShootSequence())
          .onFalse(m_theMachine.intakeRequest());

    m_driverController.povUp()
          .onTrue(m_theMachine.idleRequest());

    m_driverController.start()
        .onTrue(m_swerveDrivetrain.resetPoseWithMT1Command());

    m_driverController.back()
        .onTrue(m_theMachine.noneRequest());
     

    NamedCommands.registerCommand("AimAndShoot", 
          createAutoHubShootSequence());

    NamedCommands.registerCommand("MachineIntakeRequest", m_theMachine.intakeRequest());
    NamedCommands.registerCommand("MachineIdleDeployedRequest", m_theMachine.idleDeployedRequest());
    NamedCommands.registerCommand("MachineIdleRetractedRequest", m_theMachine.idleRetractedRequest());
    NamedCommands.registerCommand("MachineIdleRequest", m_theMachine.idleRequest());

    NamedCommands.registerCommand("SetStartPoseLeft", m_swerveDrivetrain.setStartPoseLeftCommand());
    NamedCommands.registerCommand("SetStartPoseMiddle", m_swerveDrivetrain.setStartPoseMiddleCommand());
    NamedCommands.registerCommand("SetStartPoseRight", m_swerveDrivetrain.setStartPoseRightCommand());

    NamedCommands.registerCommand("GoToTrench1", m_swerveDrivetrain.pathFindToTrench1());
    NamedCommands.registerCommand("GoToTrench2.2", m_swerveDrivetrain.pathFindToTrench2_2());
    NamedCommands.registerCommand("GoToStartPose", m_swerveDrivetrain.pathFindToStartPose1());

    NamedCommands.registerCommand("FollowTrenchIntake2_1", m_swerveDrivetrain.followTrench2_1());
    NamedCommands.registerCommand("FollowTrenchIntake2_2", m_swerveDrivetrain.followTrench2_2());

    NamedCommands.registerCommand("FollowTrenchIntake3_1", m_swerveDrivetrain.followTrench3_1());
    NamedCommands.registerCommand("FollowTrenchIntake3_2", m_swerveDrivetrain.followTrench3_2());


    NamedCommands.registerCommand("MoveToShoot8", m_swerveDrivetrain.moveToShoot8());
      
    }

  private Command createHeldHubShootSequence() {
    return Commands.either(
        m_theMachine.getReadyRequest()
            .andThen(m_theMachine.waitForShooter())
            .andThen(m_theMachine.shootRequest()),
        m_swerveDrivetrain.aimToHub().deadlineFor(
            m_theMachine.getReadyRequest()
                .andThen(m_theMachine.waitForShooter().alongWith(m_swerveDrivetrain.waitForAtAim()))
                .andThen(m_theMachine.shootRequest())),
        m_swerveDrivetrain::isAutoAimDisabled);
  }

  private Command createHeldPassShootSequence() {
    return Commands.either(
        m_theMachine.getReadyRequestPas()
            .andThen(m_theMachine.waitForShooter())
            .andThen(m_theMachine.testRequest()),
        m_swerveDrivetrain.aimToPass().deadlineFor(
            m_theMachine.getReadyRequestPas()
                .andThen(m_theMachine.waitForShooter().alongWith(m_swerveDrivetrain.waitForAtAim()))
                .andThen(m_theMachine.testRequest())),
        m_swerveDrivetrain::isAutoAimDisabled);
  }

  private Command createAutoHubShootSequence() {
    return Commands.either(
        m_theMachine.getReadyRequest()
            .andThen(m_theMachine.waitForShooter().withTimeout(3.0))
            .andThen(Commands.either(
                m_theMachine.shootRequest()
                    .andThen(new WaitCommand(4.0))
                    .andThen(m_theMachine.idleRetractedRequest()),
                m_theMachine.idleRetractedRequest(),
                m_shooterSubsystem::isShooterReadyToShoot)),
        Commands.deadline(
            m_theMachine.getReadyRequest()
                .andThen(m_theMachine.waitForShooter()
                    .alongWith(m_swerveDrivetrain.waitForAtAim())
                    .withTimeout(3.0))
                .andThen(Commands.either(
                    m_theMachine.shootRequest()
                        .andThen(new WaitCommand(4.0))
                        .andThen(m_theMachine.idleRetractedRequest()),
                    m_theMachine.idleRetractedRequest(),
                    () -> m_swerveDrivetrain.isAimed() && m_shooterSubsystem.isShooterReadyToShoot())),
            m_swerveDrivetrain.aimToHub()),
        m_swerveDrivetrain::isAutoAimDisabled);
  }

    
  private void configureSims() {
    FuelSim fuelSim = FuelSim.getInstance();
    HopperSim hopperSim = HopperSim.getInstance();
    ShooterSim shooterSim = ShooterSim.getInstance();
    MatchTrackerSim matchTracker = MatchTrackerSim.getInstance();

    SwerveFieldContactSim.getInstance().setSwerveDrivetrain(m_swerveDrivetrain);
    SwerveFieldContactSim.getInstance().setIntakeDeployedSupplier(() -> m_intakeSubsystem.isIntakeDeployed());

    fuelSim.spawnStartingFuel();

    fuelSim.registerRobot(
            Dimensions.BUMPER_WIDTH.in(Meters),
            Dimensions.BUMPER_LENGTH.in(Meters),
            Dimensions.BUMPER_HEIGHT.in(Meters),
            m_swerveDrivetrain::getPose,
            m_swerveDrivetrain::getFieldSpeeds);
    
    fuelSim.registerIntake(
            Dimensions.BUMPER_LENGTH.div(2).in(Meters),
            Dimensions.BUMPER_LENGTH.div(2).plus(Dimensions.HOPPER_EXTENSION_LENGTH).in(Meters),
            -Dimensions.BUMPER_WIDTH.div(2).in(Meters),
            Dimensions.BUMPER_WIDTH.div(2).in(Meters),
            () -> m_intakeSubsystem.isIntakeState(IntakeControlState.INTAKE) 
                        && hopperSim.isHopperAbleToIntake(),
            hopperSim::addFuelToHopper);

    fuelSim.start();

    hopperSim.setRobotPoseSupplier(m_swerveDrivetrain::getPose);
    hopperSim.setShouldRemoveFuelSupplier(() -> m_intakeSubsystem.isIntakeDeployed() 
                        && m_intakeSubsystem.isIntakeState(IntakeControlState.REVERSE));

    shooterSim.setShooterControlDataSupplier(m_shooterSubsystem::getCurrentControlData);
    shooterSim.setRobotPoseSupplier(m_swerveDrivetrain::getPose);
    shooterSim.setChassisSpeedsSupplier(m_swerveDrivetrain::getFieldSpeeds);
    shooterSim.setShouldShootSupplier(() -> (m_shooterSubsystem.isShooterState(ShooterControlState.SHOOT) || m_shooterSubsystem.isShooterState(ShooterControlState.TEST))
               && m_shooterSubsystem.shooterData.flywheelStateL == FlywheelState.AT_SPEED 
               && m_feederSubsystem.isFeederState(FeederControlState.FEED)
               && m_hopperSubsystem.isHopperState(HopperControlState.FEED));

    m_driverController.start().onTrue(matchTracker.startMatchCommand());

    FuelSim.BLUE_HUB.setHubActiveSupplier(matchTracker::isBlueHubActive);
    FuelSim.RED_HUB.setHubActiveSupplier(matchTracker::isRedHubActive);
  }
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  public void periodic() {

    if(DriverStation.isEnabled())
    {
      m_theMachine.stateMachine();
    }

    m_swerveDrivetrain.updateOfRobotPeriodic();

    m_theMachine.periodic();

    if (TelemetryConstants.SHOULD_SCHEDULER_COMMUNICATE) {
        // Telemetry for the Command Scheduler
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
     }


    //updateTelemetrySettings();

    }
}
