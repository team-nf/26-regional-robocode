package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Constants.States.IntakeStates;
import frc.robot.Constants.States.IntakeStates.IntakeControlState;
import frc.robot.Constants.States.IntakeStates.IntakePositionState;
import frc.robot.Subsystems.Intake.Hardware.IntakeHardware;
import frc.robot.Subsystems.Intake.Hardware.IntakeRealHardware;
import frc.robot.Subsystems.Intake.Hardware.IntakeSimHardware;
import frc.robot.Subsystems.Intake.StateActions.*;
import frc.robot.Subsystems.Intake.StateRequests.*;
import frc.robot.Subsystems.Intake.Utils.IntakeControlData;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public class IntakeSubsystem extends SubsystemBase {

  public final IntakeHardware intakeHardware;
  public final IntakeControlData intakeData;

  public final Command intakeClosedAction;
  public final Command intakeDeployAction;
  public final Command intakeIntakeAction;
  public final Command intakeFeedAction;
  public final Command intakeReverseAction;
  public final Command intakeIdleBetweenAction;
  public final Command intakeTestAction;
  public final Command intakeWithOffsetAction;

  public IntakeSubsystem() {
    if (Utils.isSimulation()) {
        intakeHardware = new IntakeSimHardware();
    } else {
        intakeHardware = new IntakeRealHardware();
    }

    intakeData = new IntakeControlData();

    intakeClosedAction = new IntakeClosedAction(this);
    intakeDeployAction = new IntakeDeployAction(this);
    intakeIntakeAction = new IntakeIntakeAction(this);
    intakeFeedAction = new IntakeFeedAction(this);
    intakeReverseAction = new IntakeReverseAction(this);
    intakeIdleBetweenAction = new IntakeIdleBetweenAction(this);
    intakeTestAction = new IntakeTestAction(this);
    intakeWithOffsetAction = new IntakeWithOffsetAction(this);
  }

  public void updateIntakeData() {
    intakeData.intakeVelocity = intakeHardware.getIntakeVelocity();
    intakeData.intakeError = intakeData.intakeGoalVelocity - intakeData.intakeVelocity;
    
    intakeData.intakeRollerState = Math.abs(intakeData.intakeError) <= IntakeConstants.INTAKE_ALLOWABLE_ERROR.in(RotationsPerSecond)
        ? IntakeStates.IntakeRollerState.AT_SPEED
        : IntakeStates.IntakeRollerState.REACHING_SPEED;

    intakeData.intakeArmAngle = intakeHardware.getIntakeArmPosition();
    intakeData.intakeArmError = intakeData.intakeGoalArmAngle - intakeData.intakeArmAngle;

    if(intakeData.intakeArmAngle <= IntakeConstants.INTAKE_ARM_DEPLOYED_ANGLE.plus(IntakeConstants.INTAKE_ARM_ALLOWABLE_ERROR).in(Rotations))
    {
        intakeData.intakePositionState = IntakeStates.IntakePositionState.DEPLOYED;
    }
    else if(intakeData.intakeArmAngle >= IntakeConstants.INTAKE_ARM_RETRACTED_ANGLE.minus(IntakeConstants.INTAKE_ARM_ALLOWABLE_ERROR).in(Rotations))
    {
        intakeData.intakePositionState = IntakeStates.IntakePositionState.RETRACTED;
    }
    else
    {
        intakeData.intakePositionState = IntakeStates.IntakePositionState.BETWEEN;
    }
  }

  public IntakeStates.IntakePositionState getIntakePositionState() {
    return intakeData.intakePositionState;
  }

  public boolean isIntakeDeployed() {
    return intakeData.intakePositionState == IntakeStates.IntakePositionState.DEPLOYED;
  }

  public void close() {
    intakeData.intakeGoalVelocity = 0;
    intakeData.intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_RETRACTED_ANGLE.in(Rotations);
    intakeHardware.intakeStop();
    intakeHardware.intakeArmZero();
  }

  public void feed() {
    intakeData.intakeGoalVelocity = IntakeConstants.INTAKE_FEEDING_VELOCITY.in(RotationsPerSecond);
    intakeData.intakeGoalArmAngle = IntakeConstants.INTAKE_FEED_ANGLE.in(Rotations);
    intakeHardware.setIntakeArmPosition(intakeData.intakeGoalArmAngle);
    if(intakeData.intakePositionState == IntakeStates.IntakePositionState.BETWEEN)
        intakeHardware.setIntakeSpeed(intakeData.intakeGoalVelocity);
  }

  public void deploy() {
    intakeData.intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_DEPLOYED_ANGLE.in(Rotations);
    intakeData.intakeGoalVelocity = 0;
    intakeHardware.setIntakeArmPosition(intakeData.intakeGoalArmAngle);
    intakeHardware.intakeStop();

  }

  public void intake() {
    intakeData.intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_DEPLOYED_ANGLE.in(Rotations);
    intakeHardware.setIntakeArmPosition(intakeData.intakeGoalArmAngle);


   intakeData.intakeGoalVelocity = IntakeConstants.INTAKE_INTAKING_VELOCITY.in(RotationsPerSecond);
    if(intakeData.intakePositionState == IntakeStates.IntakePositionState.DEPLOYED)
    {
        intakeHardware.setIntakeSpeed(IntakeConstants.INTAKE_INTAKING_VELOCITY.in(RotationsPerSecond));
    }
  }

    public void intakeWithOffset() {
    intakeData.intakeGoalVelocity = IntakeConstants.INTAKE_INTAKING_VELOCITY.in(RotationsPerSecond);
    intakeData.intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_DEPLOYED_WITH_OFFSET_ANGLE.in(Rotations);
    intakeHardware.setIntakeArmPosition(intakeData.intakeGoalArmAngle);

    if(intakeData.intakePositionState == IntakeStates.IntakePositionState.DEPLOYED)
        intakeHardware.setIntakeSpeed(intakeData.intakeGoalVelocity);
  }


  public void reverse() {
    intakeData.intakeGoalVelocity = IntakeConstants.INTAKE_REVERSE_VELOCITY.in(RotationsPerSecond);
    intakeData.intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_DEPLOYED_ANGLE.in(Rotations);
    intakeHardware.setIntakeArmPosition(intakeData.intakeGoalArmAngle);
    if(intakeData.intakePositionState == IntakeStates.IntakePositionState.DEPLOYED)
        intakeHardware.setIntakeSpeed(intakeData.intakeGoalVelocity);
  }

  public void idleBetween() {
    intakeData.intakeGoalVelocity = IntakeConstants.INTAKE_FEEDING_VELOCITY.in(RotationsPerSecond);
    intakeData.intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_BETWEEN_ANGLE.in(Rotations);
    intakeHardware.setIntakeArmPosition(intakeData.intakeGoalArmAngle);
    intakeHardware.setIntakeSpeed(intakeData.intakeGoalVelocity);
  }

  public void test() {
    intakeHardware.testIntake();
    intakeHardware.testArmIntake();
  }

  public boolean isIntakeState(IntakeStates.IntakeControlState state) {
    return intakeData.intakeControlState == state;
  }

  public boolean isPrevIntakeState(IntakeStates.IntakeControlState state) {
    return intakeData.prevIntakeControlState == state;
  }

  public void setIntakeState(IntakeStates.IntakeControlState newState) {
    intakeData.prevIntakeControlState = intakeData.intakeControlState;
    intakeData.intakeControlState = newState;
  }

  public InstantCommand closeRequest() {
    return new IntakeCloseRequest(this);
  }

  public InstantCommand deployRequest() {
    return new IntakeDeployRequest(this);
  }

  public InstantCommand intakeRequest() {
    return new IntakeIntakeRequest(this);
  }

  public InstantCommand feedRequest() {
    return new IntakeFeedRequest(this);
  }

  public InstantCommand reverseRequest() {
    return new IntakeReverseRequest(this);
  }

  public InstantCommand testRequest() {
    return new IntakeTestRequest(this);
  }

  public InstantCommand idleBetweenRequest() {
    return new IntakeIdleBetweenRequest(this);
  }

  // Request with explicit arm offset (in rotations)
  public InstantCommand intakeWithOffsetRequest() {
    return new IntakeWithOffsetRequest(this);
  }

  public IntakeControlData getIntakeData() {
    return intakeData;
  }

  public WaitUntilCommand waitForIntakeToBeDeployed()
  {
    return new WaitUntilCommand(() -> {
        return intakeData.intakePositionState == IntakePositionState.DEPLOYED;
    }); 
  }

  public WaitUntilCommand waitForIntakeToBeRetracted()
  {
    return new WaitUntilCommand(() -> {
        return intakeData.intakePositionState == IntakePositionState.RETRACTED;
    });
  }

  public double getArmAngleInRealLife() {
    return intakeHardware.getIntakeArmPosition();
  }

  public void stateMachine() {
    switch (intakeData.intakeControlState) {
      case CLOSE:
              if(!CommandScheduler.getInstance().isScheduled(intakeClosedAction)) {
                CommandScheduler.getInstance().schedule(intakeClosedAction);
              }
        break;
      case DEPLOY:
        if(!CommandScheduler.getInstance().isScheduled(intakeDeployAction)) {
          CommandScheduler.getInstance().schedule(intakeDeployAction);
        }
        break;
      case INTAKE:
        if(!CommandScheduler.getInstance().isScheduled(intakeIntakeAction)) {
          CommandScheduler.getInstance().schedule(intakeIntakeAction);
        }
        break;
      case FEED:
        if(!CommandScheduler.getInstance().isScheduled(intakeFeedAction)) {
          CommandScheduler.getInstance().schedule(intakeFeedAction);
        }
        break;
      case REVERSE:
        if(!CommandScheduler.getInstance().isScheduled(intakeReverseAction)) {
          CommandScheduler.getInstance().schedule(intakeReverseAction);
        }
        break;
      case IDLE_BETWEEN:
        if(!CommandScheduler.getInstance().isScheduled(intakeIdleBetweenAction)) {
          CommandScheduler.getInstance().schedule(intakeIdleBetweenAction);
        }
        break;
      case TEST:
        if(!CommandScheduler.getInstance().isScheduled(intakeTestAction)) {
          CommandScheduler.getInstance().schedule(intakeTestAction);
        }
        break;
      case INTAKE_WITH_OFFSET:
        if(!CommandScheduler.getInstance().isScheduled(intakeWithOffsetAction)) {
          CommandScheduler.getInstance().schedule(intakeWithOffsetAction);
        }
        break;
      default:
        if(!CommandScheduler.getInstance().isScheduled(intakeClosedAction)) {
          CommandScheduler.getInstance().schedule(intakeClosedAction);
        }
        break;
    }   
  }

  @Override
  public void periodic() {
    if (!Utils.isSimulation()) {
      intakeHardware.update();
    }
    
    if (TelemetryConstants.SHOULD_INTAKE_HARDWARE_COMMUNICATE) {
        SmartDashboard.putData("Intake Telemetry", intakeHardware);
    }
    if (TelemetryConstants.SHOULD_INTAKE_CONTROL_COMMUNICATE) {
        SmartDashboard.putData("Intake Control Data", intakeData);
    }

    SmartDashboard.putBoolean("Conf/isIntakeIntake", isIntakeState(IntakeControlState.INTAKE));
    SmartDashboard.putBoolean("Conf/isIntakeIntakeW_Offset", isIntakeState(IntakeControlState.INTAKE_WITH_OFFSET));

    stateMachine();
    updateIntakeData();

  }

  @Override
  public void simulationPeriodic() {
    if (Utils.isSimulation()) {
      intakeHardware.update();
    }
  }
}
