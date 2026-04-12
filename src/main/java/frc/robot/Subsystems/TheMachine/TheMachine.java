// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.TheMachine;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Constants.TheMachineConstants;
import frc.robot.Constants.States.HopperStates.HopperControlState;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Hopper.HopperSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Swerve.Utils.SwerveControlData;
import frc.robot.Subsystems.TheMachine.StateActions.TheMachineGetReadyAction;
import frc.robot.Subsystems.TheMachine.StateActions.TheMachineGetReadyActionPas;
import frc.robot.Subsystems.TheMachine.StateActions.TheMachineIdleAction;
import frc.robot.Subsystems.TheMachine.StateActions.TheMachineIdleDeployedAction;
import frc.robot.Subsystems.TheMachine.StateActions.TheMachineIdleRetractedAction;
import frc.robot.Subsystems.TheMachine.StateActions.TheMachineIntakeAction;
import frc.robot.Subsystems.TheMachine.StateActions.TheMachineReverseAction;
import frc.robot.Subsystems.TheMachine.StateActions.TheMachineShootAction;
import frc.robot.Subsystems.TheMachine.StateActions.TheMachineTestAction;
import frc.robot.Subsystems.TheMachine.StateActions.TheMachineZeroAction;
import frc.robot.Subsystems.TheMachine.StateRequests.TheMachineGetReadyRequest;
import frc.robot.Subsystems.TheMachine.StateRequests.TheMachineGetReadyRequestPas;
import frc.robot.Subsystems.TheMachine.StateRequests.TheMachineIdleDeployedRequest;
import frc.robot.Subsystems.TheMachine.StateRequests.TheMachineIdleRequest;
import frc.robot.Subsystems.TheMachine.StateRequests.TheMachineIdleRetractedRequest;
import frc.robot.Subsystems.TheMachine.StateRequests.TheMachineIntakeRequest;
import frc.robot.Subsystems.TheMachine.StateRequests.TheMachineNoneRequest;
import frc.robot.Subsystems.TheMachine.StateRequests.TheMachineReverseRequest;
import frc.robot.Subsystems.TheMachine.StateRequests.TheMachineShootRequest;
import frc.robot.Subsystems.TheMachine.StateRequests.TheMachineTestRequest;
import frc.robot.Subsystems.TheMachine.StateRequests.TheMachineZeroRequest;
import frc.robot.Subsystems.TheMachine.Utils.LEDController;
import frc.robot.Subsystems.TheMachine.Utils.TheMachineControlData;
import frc.robot.Utils.MatchTracker;

public class TheMachine {


  private TheMachineControlData theMachineData;

  private ShooterSubsystem shooterSubsystem;
  private FeederSubsystem feederSubsystem;
  private HopperSubsystem hopperSubsystem;
  private IntakeSubsystem intakeSubsystem;

  private Supplier<SwerveControlData> swerveDataSupplier;

  private Command theMachineZeroAction;
  private Command theMachineIdleDeployedAction;
  private Command theMachineIdleRetractedAction;
  private Command theMachineIntakeAction;
  private Command theMachineShootAction;
  private Command theMachineReverseAction;
  private Command theMachineTestAction;
  private Command theMachineIdleAction;
  private Command theMachineGetReadyAction;
  private Command theMachineGetReadyActionPas;

  private LEDController leftLed;

  private boolean intakeWithOffset = false;

  private MatchTracker matchTracker;

  public TheMachine(
    ShooterSubsystem shooterSubsystem,
    FeederSubsystem feederSubsystem,
    HopperSubsystem hopperSubsystem,
    IntakeSubsystem intakeSubsystem,
    Supplier<SwerveControlData> swerveDataSupplier
  ) 
  {
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.hopperSubsystem = hopperSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.swerveDataSupplier = swerveDataSupplier;

    theMachineData = new TheMachineControlData(shooterSubsystem.getShooterData(), 
                                              feederSubsystem.getFeederData(), 
                                              hopperSubsystem.getHopperData(), 
                                              intakeSubsystem.getIntakeData());


    theMachineZeroAction = TheMachineZeroAction.get(this);
    theMachineIdleDeployedAction = TheMachineIdleDeployedAction.get(this);
    theMachineIdleRetractedAction = TheMachineIdleRetractedAction.get(this);
    theMachineIntakeAction = TheMachineIntakeAction.get(this);
    theMachineShootAction = TheMachineShootAction.get(this);
    theMachineReverseAction = TheMachineReverseAction.get(this);
    theMachineTestAction = TheMachineTestAction.get(this);
    theMachineIdleAction = TheMachineIdleAction.get(this);
    theMachineGetReadyAction = TheMachineGetReadyAction.get(this);
    theMachineGetReadyActionPas = TheMachineGetReadyActionPas.get(this);

    leftLed = new LEDController(0, 30);

    matchTracker = new MatchTracker();
  }

  public TheMachineControlState getState() {
    return theMachineData.theMachineControlState;
  }

  public TheMachineControlState getPreviousState() {
    return theMachineData.previousTheMachineControlState;
  }

  public void setState(TheMachineControlState newState) {
    if (theMachineData.theMachineControlState != newState) {
      theMachineData.previousTheMachineControlState = theMachineData.theMachineControlState;
      theMachineData.theMachineControlState = newState;
    }
  }

  public boolean isState(TheMachineControlState state) {
    return theMachineData.theMachineControlState == state;
  }

  // Shooter State Requests

  public InstantCommand shooterZeroRequest() {
    return shooterSubsystem.zeroRequest();
  }

  public InstantCommand shooterRestRequest() {
    return shooterSubsystem.restRequest();
  }

  public InstantCommand shooterShootRequest() {
    return shooterSubsystem.shootRequest();
  }

  public InstantCommand shooterTestRequest() {
    return shooterSubsystem.testRequest();
  }

  // Feeder State Requests

  public InstantCommand feederZeroRequest() {
    return feederSubsystem.zeroRequest();
  }

  public InstantCommand feederFeedRequest() {
    return feederSubsystem.feedRequest();
  }

  public InstantCommand feederReverseRequest() {
    return feederSubsystem.reverseRequest();
  }

  public InstantCommand feederTestRequest() {
    return feederSubsystem.testRequest();
  }

  // Hopper State Requests

  public InstantCommand hopperZeroRequest() {
    return hopperSubsystem.zeroRequest();
  }

  public InstantCommand hopperFeedRequest() {
    return hopperSubsystem.feedRequest();
  }

  public InstantCommand hopperPushRequest() {
    return hopperSubsystem.pushRequest();
  }

  public InstantCommand hopperReverseRequest() {
    return hopperSubsystem.reverseRequest();
  }

  public InstantCommand hopperTestRequest() {
    return hopperSubsystem.testRequest();
  }

  // Intake State Requests

  public InstantCommand intakeCloseRequest() {
    return intakeSubsystem.closeRequest();
  }

  public InstantCommand intakeDeployRequest() {
    return intakeSubsystem.deployRequest();
  }

  public InstantCommand intakeIntakeRequest() {
    return intakeSubsystem.intakeRequest();
  }

  public InstantCommand intakeWithOffsetRequest() {
    return intakeSubsystem.intakeWithOffsetRequest();
  }

  public InstantCommand intakeFeedRequest() {
    return intakeSubsystem.feedRequest();
  }

  public InstantCommand intakeReverseRequest() {
    return intakeSubsystem.reverseRequest();
  }

  public InstantCommand intakeIdleBetweenRequest() {
    return intakeSubsystem.idleBetweenRequest();
  }

  public InstantCommand intakeTestRequest() {
    return intakeSubsystem.testRequest();
  }

  // Wait for Commands
  public WaitUntilCommand waitForShooter()
  {
    return shooterSubsystem.waitForShooterToBeReady();
  }

  public WaitUntilCommand waitForIntakeDeploy()
  {
    return intakeSubsystem.waitForIntakeToBeDeployed();
  }

  public WaitUntilCommand waitForIntakeRetract()
  {
    return intakeSubsystem.waitForIntakeToBeRetracted();
  }

  public Command zeroRequest(){
      return new TheMachineZeroRequest(this);
  } 

  public Command idleDeployedRequest(){
      return new TheMachineIdleDeployedRequest(this);
  }
  
  public Command idleRetractedRequest() {
      return new TheMachineIdleRetractedRequest(this);
  }

  public Command idleRequest() {
    return new TheMachineIdleRequest(this);
  }

  public Command intakeRequest() {
      return new TheMachineIntakeRequest(this);
  }

  public Command shootRequest() {
      return new TheMachineShootRequest(this);
  }

  public Command getReadyRequest(){
    return new TheMachineGetReadyRequest(this);
  }

  public Command getReadyRequestPas(){
    return new TheMachineGetReadyRequestPas(this);
  }

  public Command reverseRequest() {
      return new TheMachineReverseRequest(this);
  }

  public Command testRequest() {
      return new TheMachineTestRequest(this);
  }

  public Command noneRequest() {
    return new TheMachineNoneRequest(this);
  }

  private StructPublisher<Pose3d> hoodPosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Sim/HoodPose", Pose3d.struct).publish();

  private StructPublisher<Pose3d> funnelPosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Sim/FunnelPose", Pose3d.struct).publish();

  private StructPublisher<Pose3d> intakePosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Sim/IntakePose", Pose3d.struct).publish();

  public boolean checkHopperPrevFeed()
  {
    return hopperSubsystem.isPrevHopperState(HopperControlState.FEED);
  }

  public void calculateSubsytemPoses() {

    double hoodAngle = shooterSubsystem.getHoodAngleInRealLife();

    Pose3d hoodPose = TheMachineConstants.HOOD_RETRACTED_POSE
                            .rotateAround(TheMachineConstants.HOOD_RETRACTED_POSE.getTranslation(),new Rotation3d(0, Math.toRadians(hoodAngle*360), 0));

    double intakeArmAngle = intakeSubsystem.getArmAngleInRealLife();

    Pose3d intakePose = TheMachineConstants.INTAKE_DEPLOYED_POSE
                          .rotateAround(TheMachineConstants.INTAKE_DEPLOYED_POSE.getTranslation(), new Rotation3d(0, -Math.toRadians(intakeArmAngle*360), 0));

    Distance funnelExtension = Meters.of(0.0);

    if(intakeArmAngle < 30) funnelExtension = Meters.of(0.3125);
    else funnelExtension = Meters.of(0.3125).times(Math.cos(Math.toRadians(intakeArmAngle - 30)));
    
    Pose3d funnelPose = TheMachineConstants.FUNNEL_RETRACTED_POSE
                          .plus(new Transform3d(funnelExtension.in(Meters), 0, 0, new Rotation3d(0, 0, 0)));
    
    hoodPosePublisher.set(hoodPose);
    intakePosePublisher.set(intakePose);
    funnelPosePublisher.set(funnelPose);
  }

  private void scheduleIfNotRunning(Command action) {
    var scheduler = CommandScheduler.getInstance();
    if (!scheduler.isScheduled(action)) {
      scheduler.schedule(action);
    }
  }

  public void stateMachine() {
    switch (theMachineData.theMachineControlState) {
      case ZERO:            scheduleIfNotRunning(theMachineZeroAction); break;
      case IDLE_DEPLOYED:   scheduleIfNotRunning(theMachineIdleDeployedAction); break;
      case IDLE_RETRACTED:  scheduleIfNotRunning(theMachineIdleRetractedAction); break;
      case INTAKE:          scheduleIfNotRunning(theMachineIntakeAction); break;
      case SHOOT:           scheduleIfNotRunning(theMachineShootAction); break;
      case REVERSE:         scheduleIfNotRunning(theMachineReverseAction); break;
      case TEST:            scheduleIfNotRunning(theMachineTestAction); break;
      case IDLE:            scheduleIfNotRunning(theMachineIdleAction); break;
      case GET_READY:       scheduleIfNotRunning(theMachineGetReadyAction); break;
      case GET_READY_PAS:   scheduleIfNotRunning(theMachineGetReadyActionPas); break;
      case NONE:
        var scheduler = CommandScheduler.getInstance();
        scheduler.schedule(shooterSubsystem.zeroRequest());
        scheduler.schedule(feederSubsystem.zeroRequest());
        scheduler.schedule(hopperSubsystem.zeroRequest());
        scheduler.schedule(intakeSubsystem.closeRequest());
        break;
      default: break;
    }
  }

  public void updateLeds()
  {
    if(DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Blue).orElse(true))
    {
      leftLed.setMultiple2(0,0.5, 0, 0, 255);
    }
    else
    {
      leftLed.setMultiple2(0,0.5, 255, 0, 0);
    }

    leftLed.setMultiple2(0.5,1, 0, 255, 0);
  }

  public boolean isIntakeWithoutOffset()
  {
    return !intakeWithOffset;
  }

  public void changeIntakeMode()
  {
    intakeWithOffset = !intakeWithOffset;
  }

  public void periodic() {
    matchTracker.updateMatchTracker();
    
    if (TelemetryConstants.SHOULD_THEMACHINE_DATA_COMMUNICATE)
    {
      SmartDashboard.putData(theMachineData);
      SmartDashboard.putString("TheMachineControlState", theMachineData.theMachineControlState.toString());
      SmartDashboard.putString("PreviousTheMachineControlState", theMachineData.previousTheMachineControlState.toString());
    }

    if (TelemetryConstants.SHOULD_THEMACHINE_SIM_POSES_COMMUNICATE)
    {
      calculateSubsytemPoses();
    }
  }
}