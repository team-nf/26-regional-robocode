// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utils.FuelSim;
import frc.robot.Utils.HopperSim;
import frc.robot.Utils.MatchTrackerSim;
import frc.robot.Utils.ShooterSim;
import frc.robot.Utils.SwerveFieldContactSim;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit()
  {
    PathfindingCommand.warmupCommand();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  private Notifier swerveSimNotifier;

  @Override
  public void simulationInit() {
    
    swerveSimNotifier = new Notifier(() -> {
      SwerveFieldContactSim.getInstance().handleSwerveSimFieldCollisions();
    });
    
    swerveSimNotifier.startPeriodic(SwerveFieldContactSim.getInstance().simLoopTimeSec);
    
  }

  @Override
  public void simulationPeriodic() {
    
    FuelSim.getInstance().updateSim();
    HopperSim.getInstance().updateSim();
    ShooterSim.getInstance().updateSim();
    MatchTrackerSim.getInstance().updateMatchTracker();
    
  }
}
