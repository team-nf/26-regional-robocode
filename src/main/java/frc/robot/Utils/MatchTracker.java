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
                isBlueHubActive = true;
                isRedHubActive = true;
                firstPhaseStatus = "InvalidData";
                publishStatus();
                return;
            }
        }

        firstPhaseStatus = (redInactiveFirst ? "BlueFirst" : "RedFirst");
        computePhaseFromMatchTime(redInactiveFirst);
        publishStatus();
    }

    private void computePhaseFromMatchTime(boolean redInactiveFirst) {
        if (matchTime > 130) {
            isBlueHubActive = true;
            isRedHubActive = true;
            activePhaseDuration = matchTime - 130;
        } else if (matchTime > 105) {
            isBlueHubActive = redInactiveFirst;
            isRedHubActive = !redInactiveFirst;
            activePhaseDuration = matchTime - 105;
        } else if (matchTime > 80) {
            isBlueHubActive = !redInactiveFirst;
            isRedHubActive = redInactiveFirst;
            activePhaseDuration = matchTime - 80;
        } else if (matchTime > 55) {
            isBlueHubActive = redInactiveFirst;
            isRedHubActive = !redInactiveFirst;
            activePhaseDuration = matchTime - 55;
        } else if (matchTime > 30) {
            isBlueHubActive = !redInactiveFirst;
            isRedHubActive = redInactiveFirst;
            activePhaseDuration = matchTime - 30;
        } else {
            isBlueHubActive = true;
            isRedHubActive = true;
            activePhaseDuration = matchTime;
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
