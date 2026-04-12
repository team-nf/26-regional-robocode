// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

public class States {

    public static class ShooterStates {
           public enum FlywheelState {
                ZERO,
                REACHING_SPEED,
                AT_SPEED
            }

            public enum HoodState {
                ZERO,
                MOVING_TO_POSITION,
                AT_POSITION
            }

            public enum ShooterControlState {
                ZERO,
                REST,
                SHOOT,
                TEST
            }
    }

    public static class FeederStates {
            public enum FeederControlState {
                ZERO,
                FEED,
                REVERSE,
                TEST
            }

            public enum FeederRollerState {
                ZERO,
                REACHING_SPEED,
                AT_SPEED
            }
    }

    public static class HopperStates {
            public enum HopperControlState {
                ZERO,
                FEED,
                PUSH,
                REVERSE,
                TEST
            }

            public enum HopperRollerState {
                ZERO,
                REACHING_SPEED,
                AT_SPEED
            }
    }

    public static class IntakeStates {
            public enum IntakeControlState {
                CLOSE,
                DEPLOY,
                INTAKE,
                INTAKE_WITH_OFFSET,
                REVERSE,
                FEED,
                IDLE_BETWEEN,
                TEST
            }

            public enum IntakeRollerState {
                ZERO,
                REACHING_SPEED,
                AT_SPEED
            }

            public enum IntakePositionState {
                DEPLOYED,
                RETRACTED,
                BETWEEN
            }
    }

    public static class SwerveStates {
            public enum SwerveState {
                TELEOP,
                AIMING,
                PATH_FOLLOWING,
            }
    }

    public static class TheMachineStates {
            public enum TheMachineControlState {
                ZERO,
                IDLE_RETRACTED, 
                IDLE_DEPLOYED,
                INTAKE,
                SHOOT,
                REVERSE,
                TEST,
                NONE,
                IDLE,
                GET_READY,
                GET_READY_PAS
            }
    }

}
