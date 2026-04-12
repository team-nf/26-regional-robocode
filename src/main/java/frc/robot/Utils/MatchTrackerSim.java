// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/** Add your docs here. */
public class MatchTrackerSim {

    private int teleopTimeSeconds = 140;
    private int autonomousTimeSeconds = 20;
    private int phaseDurationSeconds = 25;

    private boolean isAutonomous = true;
    private boolean isTeleop = false;
    private boolean isMatchStarted = false;

    private final double[] matchPeriods = {160,140,130,105,80,55,30,0};

    private boolean isPhaseBlue = false;
    private boolean isPhaseRed = false;
    private boolean isFirstPhaseBlue = false;

    private double blueScore = 0;
    private double redScore = 0;

    private double startTime = 0;
    private double matchTime = 0;

    private static MatchTrackerSim instance = null;

    public static MatchTrackerSim getInstance() {
        if (instance == null) {
            instance = new MatchTrackerSim();
        }
        return instance;
    }

    public void startMatch()
    {
        isMatchStarted = true;
        isAutonomous = true;
        isTeleop = false;

        startTime = System.currentTimeMillis() / 1000.0; // Convert to seconds

        FuelSim.getInstance().clearFuel();
        SwerveFieldContactSim.getInstance().reset();
        HopperSim.getInstance().reset();
        FuelSim.getInstance().spawnStartingFuel();
        FuelSim.BLUE_HUB.resetScore();
        FuelSim.RED_HUB.resetScore();
    }

    public Command startMatchCommand()
    {
        return new InstantCommand(this::startMatch);
    }

    public void updateMatchTracker()
    {
        double currentTime = (System.currentTimeMillis() / 1000.0) - startTime;
        if(isMatchStarted)
        {
            matchTime = matchPeriods[0] - currentTime; // Adjust for initial autonomous period

            if (matchTime > matchPeriods[1]) {
                isPhaseBlue = true;
                isPhaseRed = true;
            } else if (matchTime > matchPeriods[2]) {
                isPhaseBlue = true;
                isPhaseRed = true;

                if(blueScore > redScore)
                {
                    isFirstPhaseBlue = false;
                }
                else if(redScore > blueScore)
                {
                    isFirstPhaseBlue = true;
                }
                // Tie: keep previous isFirstPhaseBlue value
            } else if (matchTime > matchPeriods[3]) {
                isPhaseBlue = isFirstPhaseBlue;
                isPhaseRed = !isFirstPhaseBlue;
            } else if (matchTime > matchPeriods[4]) {
                isPhaseBlue = !isFirstPhaseBlue;
                isPhaseRed = isFirstPhaseBlue;
            } else if (matchTime > matchPeriods[5]) {
                isPhaseBlue = isFirstPhaseBlue;
                isPhaseRed = !isFirstPhaseBlue;
            } else if (matchTime > matchPeriods[6]) {
                isPhaseBlue = !isFirstPhaseBlue;
                isPhaseRed = isFirstPhaseBlue;
            } else if (matchTime > matchPeriods[7]) {
                isPhaseBlue = true;
                isPhaseRed = true;
            } else {
                // Match has ended
                isPhaseBlue = false;
                isPhaseRed = false;
                isMatchStarted = false;
            }

            isTeleop = DriverStation.isTeleop() && isMatchStarted;
            isAutonomous = DriverStation.isAutonomous() && isMatchStarted;

            blueScore = FuelSim.BLUE_HUB.getScore();
            redScore = FuelSim.RED_HUB.getScore();
        }

        publishMatchStatus();
    }

    public void publishMatchStatus()
    {
        SmartDashboard.putBoolean("Match/Is Autonomous", isAutonomous);
        SmartDashboard.putBoolean("Match/Is Teleop", isTeleop);
        SmartDashboard.putBoolean("Match/Is Phase Blue", isPhaseBlue);
        SmartDashboard.putBoolean("Match/Is Phase Red", isPhaseRed);
        SmartDashboard.putNumber("Match/Blue Score", blueScore);
        SmartDashboard.putNumber("Match/Red Score", redScore);
        SmartDashboard.putNumber("Match/Seconds", matchTime);
        SmartDashboard.putString("Match/MatchTime", String.format("%02d:%02d", (int)matchTime / 60, (int)matchTime % 60));
    }

    public boolean isBlueHubActive() {
        return isPhaseBlue;
    }

    public boolean isRedHubActive() {
        return isPhaseRed;
    }

}
