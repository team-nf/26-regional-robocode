package frc.robot.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.States.SwerveStates.SwerveState;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;

public class Localization {
    private static final Matrix<N3, N1> MT2_STD_DEVS = VecBuilder.fill(0.5, 0.5, 9999999);

    private final CommandSwerveDrivetrain drivetrain;

    private boolean leftEnabled = true;
    private boolean rightEnabled = true;
    private boolean disabledLocoEnabled = true;

    public Localization(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        SmartDashboard.putBoolean("Conf/LL-Left_Enabled", true);
        SmartDashboard.putBoolean("Conf/LL-Right_Enabled", true);
        SmartDashboard.putBoolean("Conf/DisabledLocoEnabled", true);
    }

    public void updateConfig() {
        leftEnabled = SmartDashboard.getBoolean("Conf/LL-Left_Enabled", true);
        rightEnabled = SmartDashboard.getBoolean("Conf/LL-Right_Enabled", true);
        disabledLocoEnabled = SmartDashboard.getBoolean("Conf/DisabledLocoEnabled", true);
    }

    private boolean shouldRejectMeasurement(LimelightHelpers.PoseEstimate estimate, double maxDist) {
        if (estimate == null) return true;
        if (estimate.tagCount < 1) return true;
        if (estimate.avgTagDist > maxDist) return true;
        return false;
    }

    public void addVisionMeasurementMT2() {
        double robotYaw = drivetrain.getGyroHeading();
        LimelightHelpers.SetRobotOrientation("limelight-left", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        LimelightHelpers.PoseEstimate limelightMeasurementLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
        LimelightHelpers.PoseEstimate limelightMeasurementRight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");

        if (drivetrain.swerveDataSupplier().get().swerveControlState == SwerveState.AIMING && DriverStation.isAutonomous()) return;
        if (Math.abs(drivetrain.getGyroRate()) > 360) return;

        if (!shouldRejectMeasurement(limelightMeasurementLeft, 3.5) && leftEnabled) {
            drivetrain.setVisionMeasurementStdDevs(MT2_STD_DEVS);
            drivetrain.addVisionMeasurement(
                limelightMeasurementLeft.pose,
                limelightMeasurementLeft.timestampSeconds
            );
        }

        if (!shouldRejectMeasurement(limelightMeasurementRight, 3.5) && rightEnabled) {
            drivetrain.setVisionMeasurementStdDevs(MT2_STD_DEVS);
            drivetrain.addVisionMeasurement(
                limelightMeasurementRight.pose,
                limelightMeasurementRight.timestampSeconds
            );
        }
    }

    public void disabledPeriodic() {
        updateConfig();
        if (disabledLocoEnabled) {
            resetWithMT1();
        }
    }

    public void enabledPeriodic() {
        addVisionMeasurementMT2();
    }

    public void resetWithMT1() {
        double robotYaw = drivetrain.getGyroHeading();
        LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        LimelightHelpers.PoseEstimate limelightMeasurementRight = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");

        if (limelightMeasurementRight != null && limelightMeasurementRight.tagCount > 0) {
            if (limelightMeasurementRight.avgTagDist < 3.0) {
                drivetrain.resetPose(limelightMeasurementRight.pose);
            }
        }
    }
}
