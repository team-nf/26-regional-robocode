// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class MatchTracker {

    private boolean isBlueHubActive = false;
    private boolean isRedHubActive = false;
    private String firstPhaseStatus = "";

    private double activePhaseDuration = 0;
    private double matchTime = 0;

    public void updateMatchTracker()
    {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            isBlueHubActive = false;
            isRedHubActive = false;
            activePhaseDuration = 0;
            firstPhaseStatus = "Empty";
            publishStatus();
            return;
        
        }

        if (DriverStation.isAutonomousEnabled())
        {
            isBlueHubActive = true;
            isRedHubActive = true;
            activePhaseDuration = 0;
            firstPhaseStatus = "Auto";
            publishStatus();
            return;
        }

        if(!DriverStation.isTeleopEnabled())
        {
            isBlueHubActive = false;
            isRedHubActive = false;
            activePhaseDuration = 0;
            firstPhaseStatus = "NotTeleop";
            publishStatus();
            return;
        }

        // We're teleop enabled, compute.
        matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            isBlueHubActive = true;
            isRedHubActive = true;
            activePhaseDuration = 0;
            firstPhaseStatus = "Empty";
            publishStatus();
            return;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            isBlueHubActive = true;
            isRedHubActive = true;
            firstPhaseStatus = "InvalidData";
            publishStatus();
            return;
            }
        }

        firstPhaseStatus = (redInactiveFirst ? "BlueFirst" : "RedFirst");

        if (matchTime > 130) {
            // Transition shift, hub is active.
            isBlueHubActive = true;
            isRedHubActive = true;
            activePhaseDuration = matchTime - 130;
            publishStatus();
            return;
        } else if (matchTime > 105) {
            // Shift 1
            isBlueHubActive = redInactiveFirst;
            isRedHubActive = !redInactiveFirst;
            activePhaseDuration = matchTime - 105;
            publishStatus();
            return;
        } else if (matchTime > 80) {
            // Shift 2
            isBlueHubActive = !redInactiveFirst;
            isRedHubActive = redInactiveFirst;
            activePhaseDuration = matchTime - 80;
            publishStatus();
            return;
        } else if (matchTime > 55) {
            // Shift 3
            isBlueHubActive = redInactiveFirst;
            isRedHubActive = !redInactiveFirst;
            activePhaseDuration = matchTime - 55;
            publishStatus();
            return;
        } else if (matchTime > 30) {
            // Shift 4
            isBlueHubActive = !redInactiveFirst;
            isRedHubActive = redInactiveFirst;
            activePhaseDuration = matchTime - 30;
            publishStatus();
            return;
        } else {
            // End game, hub always active.
            isBlueHubActive = true;
            isRedHubActive = true;
            activePhaseDuration = matchTime;
            publishStatus();
            return;
        } 
    }

    public void publishStatus()
    {
        SmartDashboard.putBoolean("Phase/IsBlueHubActive", isBlueHubActive);
        SmartDashboard.putBoolean("Phase/IsRedHubActive", isRedHubActive);
        SmartDashboard.putNumber("Phase/ActivePhaseDuration", activePhaseDuration);
        SmartDashboard.putNumber("Phase/GameTime", matchTime);
        SmartDashboard.putString("Phase/Status", firstPhaseStatus);
    }

}
