package frc.robot.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.States.SwerveStates.SwerveState;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Utils.LimelightHelpers;


public class Localization {
    private static final Matrix<N3, N1> MT1_STD_DEVS = VecBuilder.fill(0.5, 0.5, 9999999);

    CommandSwerveDrivetrain drivetrain;

    private boolean isMode1Set = false;
    private boolean isLLReady = false;

    public Localization(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        SmartDashboard.putBoolean("Conf/LL-Left_Enabled", true);
        SmartDashboard.putBoolean("Conf/LL-Right_Enabled", true);
        SmartDashboard.putBoolean("Conf/DisabledLocoEnabled", true);
    }

    public void addVisionMeasurementMT1() {
    double robotYaw = drivetrain.getGyroHeading();
    LimelightHelpers.SetRobotOrientation("limelight-left", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    LimelightHelpers.PoseEstimate limelightMeasurementLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
    LimelightHelpers.PoseEstimate limelightMeasurementRight = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");

    boolean doRejectUpdate = false;
    boolean doRejectLeft = false;
    boolean doRejectRight = false;

 if(drivetrain.swerveDataSupplier().get().swerveControlState == SwerveState.AIMING && DriverStation.isAutonomous()) doRejectUpdate = true;


    if(Math.abs(drivetrain.getGyroRate()) > 360)
    {
        doRejectUpdate = true;
    }

    if(limelightMeasurementLeft != null)
    {
        if(limelightMeasurementLeft.tagCount < 1)
            {
                doRejectLeft = true;
            }

        if(limelightMeasurementLeft.avgTagDist > 3.5)
            {
                doRejectLeft = true;
            }
    }
    else doRejectLeft = true;

    if(limelightMeasurementRight != null)
    {
    if(limelightMeasurementRight.tagCount < 1)
        {
            doRejectRight = true;
        }
        if(limelightMeasurementRight.avgTagDist > 3.5)
            {
                doRejectRight = true;
            }
    }
    else doRejectRight = true;


    if(!doRejectUpdate)
    {
        if(!doRejectLeft && SmartDashboard.getBoolean("Conf/LL-Left_Enabled", true))
        {
            drivetrain.setVisionMeasurementStdDevs(MT1_STD_DEVS);

            drivetrain.addVisionMeasurement(
            limelightMeasurementLeft.pose,
            limelightMeasurementLeft.timestampSeconds
            );
        }

        if(!doRejectRight && SmartDashboard.getBoolean("Conf/LL-Right_Enabled", true))
        {
            drivetrain.setVisionMeasurementStdDevs(MT1_STD_DEVS);
            drivetrain.addVisionMeasurement(
            limelightMeasurementRight.pose,
            limelightMeasurementRight.timestampSeconds
            );
        }
    }
    }

    public void disabledPeriodic()
    {
        isMode1Set = true;

        if (SmartDashboard.getBoolean("Conf/DisabledLocoEnabled", true)) {
            resetWithMT1();
        }
    }

    public void enabledPeriodic()
    {
        if(!isLLReady && isMode1Set)
        {
            isLLReady = true;
        }

        if(isLLReady) addVisionMeasurementMT1();
    }

    public void resetWithMT1()
    {
        double robotYaw = drivetrain.getGyroHeading();
        LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        LimelightHelpers.PoseEstimate limelightMeasurementRight = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");

        if(limelightMeasurementRight != null && limelightMeasurementRight.tagCount > 0)
        {
            if (limelightMeasurementRight.avgTagDist < 3.0) {
                drivetrain.resetPose(limelightMeasurementRight.pose);
            }
        }

    }
}
