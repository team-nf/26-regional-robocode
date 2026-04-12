package frc.robot.Subsystems.TheMachine.StateActions;

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of

// the WPILib BSD license file in the root directory of this project.
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsystems.TheMachine.TheMachine;

public class TheMachineIdleAction {

  public static Command get(TheMachine theMachine) {
    return new ParallelCommandGroup(
        theMachine.shooterRestRequest(),
        theMachine.feederReverseRequest(),
        theMachine.hopperZeroRequest(),
        theMachine.intakeIdleBetweenRequest()
    ).until(() -> theMachine.getState() != TheMachineControlState.IDLE);
  }
}