package frc.robot.Subsystems.Hopper;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Constants.States.HopperStates;
import frc.robot.Subsystems.Hopper.Hardware.HopperHardware;
import frc.robot.Subsystems.Hopper.Hardware.HopperRealHardware;
import frc.robot.Subsystems.Hopper.Hardware.HopperSimHardware;
import frc.robot.Subsystems.Hopper.StateActions.*;
import frc.robot.Subsystems.Hopper.StateRequests.*;
import frc.robot.Subsystems.Hopper.Utils.HopperControlData;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public class HopperSubsystem extends SubsystemBase {

  public final HopperHardware hopperHardware;
  public final HopperControlData hopperData;

  public final Command hopperZeroAction;
  public final Command hopperFeedAction;
  public final Command hopperPushAction;
  public final Command hopperReverseAction;
  public final Command hopperTestAction;

  public HopperSubsystem() {
    if (Utils.isSimulation()) {
        hopperHardware = new HopperSimHardware();
    } else {
        hopperHardware = new HopperRealHardware();
    }

    hopperData = new HopperControlData();

    hopperZeroAction = new HopperZeroAction(this);
    hopperFeedAction = new HopperFeedAction(this);
    hopperPushAction = new HopperPushAction(this);
    hopperReverseAction = new HopperReverseAction(this);
    hopperTestAction = new HopperTestAction(this);
  }

  public void updateHopperData() {
    hopperData.hopperVelocity = hopperHardware.getHopperVelocity();
    hopperData.hopperError = hopperData.hopperGoalVelocity - hopperData.hopperVelocity;
    
    hopperData.hopperRollerState = Math.abs(hopperData.hopperError) <= HopperConstants.HOPPER_ALLOWABLE_ERROR.in(RotationsPerSecond)
        ? HopperStates.HopperRollerState.AT_SPEED
        : HopperStates.HopperRollerState.REACHING_SPEED;
  }

  public void zero() {
    hopperData.hopperGoalVelocity = 0;
    hopperHardware.hopperStop();
  }

  public void feed() {
    hopperData.hopperGoalVelocity = HopperConstants.HOPPER_FEEDING_VELOCITY.in(RotationsPerSecond);
    hopperHardware.setHopperSpeed(hopperData.hopperGoalVelocity);
  }

  public void push() {
    hopperData.hopperGoalVelocity = HopperConstants.HOPPER_PUSHING_VELOCITY.in(RotationsPerSecond);
    hopperHardware.setHopperSpeed(hopperData.hopperGoalVelocity);
  }

  public void reverse() {
    hopperData.hopperGoalVelocity = HopperConstants.HOPPER_REVERSE_VELOCITY.in(RotationsPerSecond);
    hopperHardware.setHopperSpeed(hopperData.hopperGoalVelocity);
  }

  public void test() {
    hopperHardware.testHopper();
  }

  public boolean isHopperState(HopperStates.HopperControlState state) {
    return hopperData.hopperControlState == state;
  }

  public boolean isPrevHopperState(HopperStates.HopperControlState state) {
    return hopperData.prevHopperControlState == state;
  }

  public boolean isHopperAtSpeed() {
    return hopperData.hopperRollerState == HopperStates.HopperRollerState.AT_SPEED;
  }

  public void setHopperState(HopperStates.HopperControlState newState) {
    hopperData.prevHopperControlState = hopperData.hopperControlState;
    hopperData.hopperControlState = newState;
  }

  public InstantCommand zeroRequest() {
    return new HopperZeroRequest(this);
  }

  public InstantCommand feedRequest() {
    return new HopperFeedRequest(this);
  }

  public InstantCommand pushRequest() {
    return new HopperPushRequest(this);
  }

  public InstantCommand reverseRequest() {
    return new HopperReverseRequest(this);
  }

  public InstantCommand testRequest() {
    return new HopperTestRequest(this);
  }

  public HopperControlData getHopperData() {
    return hopperData;
  }

  public void stateMachine() {
    switch (hopperData.hopperControlState) {
      case ZERO:
        if(!CommandScheduler.getInstance().isScheduled(hopperZeroAction)) {
          CommandScheduler.getInstance().schedule(hopperZeroAction);
        }
        break;
      case FEED:
        if(!CommandScheduler.getInstance().isScheduled(hopperFeedAction)) {
          CommandScheduler.getInstance().schedule(hopperFeedAction);
        }
        break;
      case PUSH:
        if(!CommandScheduler.getInstance().isScheduled(hopperPushAction)) {
          CommandScheduler.getInstance().schedule(hopperPushAction);
        }
        break;
      case REVERSE:
        if(!CommandScheduler.getInstance().isScheduled(hopperReverseAction)) {
          CommandScheduler.getInstance().schedule(hopperReverseAction);
        }
        break;
      case TEST:
        if(!CommandScheduler.getInstance().isScheduled(hopperTestAction)) {
          CommandScheduler.getInstance().schedule(hopperTestAction);
        }
        break;
      default:
        if(!CommandScheduler.getInstance().isScheduled(hopperZeroAction)) {
          CommandScheduler.getInstance().schedule(hopperZeroAction);
        }
        break;
    }
  }

  @Override
  public void periodic() {
    if (!Utils.isSimulation()) {
      hopperHardware.update();
    }
    
    if (TelemetryConstants.SHOULD_HOPPER_HARDWARE_COMMUNICATE) {
        SmartDashboard.putData("Hopper Telemetry", hopperHardware);
    }
    if (TelemetryConstants.SHOULD_HOPPER_CONTROL_COMMUNICATE) {
        SmartDashboard.putData("Hopper Control Data", hopperData);
    }

    stateMachine();
    updateHopperData();

  }

  @Override
  public void simulationPeriodic() {
    if (Utils.isSimulation()) {
      hopperHardware.update();
    }
  }
}
