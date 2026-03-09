// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Feeder;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Constants.States.FeederStates;
import frc.robot.Constants.States.FeederStates.*;
import frc.robot.Subsystems.Feeder.Hardware.FeederHardware;
import frc.robot.Subsystems.Feeder.Hardware.FeederRealHardware;
import frc.robot.Subsystems.Feeder.Hardware.FeederSimHardware;
import frc.robot.Subsystems.Feeder.StateActions.*;
import frc.robot.Subsystems.Feeder.StateRequests.*;
import frc.robot.Subsystems.Feeder.Utils.FeederControlData;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public class FeederSubsystem extends SubsystemBase {

  public final FeederHardware feederHardware;
  public final FeederControlData feederData;

  public final Command feederZeroAction;
  public final Command feederFeedAction;
  public final Command feederReverseAction;
  public final Command feederTestAction;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    if (Utils.isSimulation()) {
        feederHardware = new FeederSimHardware();
    } else {
        feederHardware = new FeederRealHardware();
    }

    feederData = new FeederControlData();

    feederZeroAction = new FeederZeroAction(this);
    feederFeedAction = new FeederFeedAction(this);
    feederReverseAction = new FeederReverseAction(this);
    feederTestAction = new FeederTestAction(this);
  }

  public void updateFeederData() {
    feederData.feederVelocity = feederHardware.getFeederVelocity();
    feederData.feederError = feederData.feederGoalVelocity - feederData.feederVelocity;
    
    feederData.feederRollerState = Math.abs(feederData.feederError) <= FeederConstants.FEEDER_ALLOWABLE_ERROR.in(RotationsPerSecond)
        ? FeederRollerState.AT_SPEED
        : FeederRollerState.REACHING_SPEED;
  }

  public void zero() {
    feederData.feederGoalVelocity = 0;
    feederHardware.feederStop();
  }

  public void feed() {
    feederData.feederGoalVelocity = FeederConstants.FEEDER_FEEDING_VELOCITY.in(RotationsPerSecond);
    feederHardware.setFeederSpeed(feederData.feederGoalVelocity);
  }

  public void reverse() {
    feederData.feederGoalVelocity = FeederConstants.FEEDER_REVERSE_VELOCITY.in(RotationsPerSecond);
    feederHardware.setFeederSpeed(feederData.feederGoalVelocity);
  }

  public void test() {
    feederHardware.testFeeder();
  }

  public boolean isFeederState(FeederControlState state) {
    return feederData.feederControlState == state;
  }

  public boolean isPrevFeederState(FeederControlState state) {
    return feederData.prevFeederControlState == state;
  }

  public boolean isFeederAtSpeed() {
    return feederData.feederRollerState == FeederRollerState.AT_SPEED;
  }

  public void setFeederState(FeederControlState newState) {
    feederData.prevFeederControlState = feederData.feederControlState;
    feederData.feederControlState = newState;
  }

  public InstantCommand zeroRequest() {
    return new FeederZeroRequest(this);
  }

  public InstantCommand feedRequest() {
    return new FeederFeedRequest(this);
  }

  public InstantCommand reverseRequest() {
    return new FeederReverseRequest(this);
  }

  public InstantCommand testRequest() {
    return new FeederTestRequest(this);
  }

  public FeederControlData getFeederData() {
    return feederData;
  }

  private void scheduleIfNotRunning(Command action) {
    var scheduler = CommandScheduler.getInstance();
    if (!scheduler.isScheduled(action)) scheduler.schedule(action);
  }

  public void stateMachine() {
    switch (feederData.feederControlState) {
      case ZERO:    scheduleIfNotRunning(feederZeroAction); break;
      case FEED:    scheduleIfNotRunning(feederFeedAction); break;
      case REVERSE: scheduleIfNotRunning(feederReverseAction); break;
      case TEST:    scheduleIfNotRunning(feederTestAction); break;
      default:      scheduleIfNotRunning(feederZeroAction); break;
    }
  }

  @Override
  public void periodic() {
    if (!Utils.isSimulation()) {
      feederHardware.update();
    }
    
    if (TelemetryConstants.SHOULD_FEEDER_HARDWARE_COMMUNICATE) {
        SmartDashboard.putData("Feeder Telemetry", feederHardware);
    }
    if (TelemetryConstants.SHOULD_FEEDER_CONTROL_COMMUNICATE) {
        SmartDashboard.putData("Feeder Control Data", feederData);
    }
    
    updateFeederData();
    stateMachine();
  }

  @Override
  public void simulationPeriodic() {
    if (Utils.isSimulation()) {
      feederHardware.update();
    }
  }
}
