package frc.robot.Utils;

import com.ctre.phoenix6.StatusSignal.SignalMeasurement;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States.SwerveStates.SwerveState;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.LimelightHelpers.PoseEstimate;


public class Localization {
    private static final Matrix<N3, N1> MT2_STD_DEVS = VecBuilder.fill(0.6, 0.6, 9999999);
    private static final Matrix<N3, N1> MT1_STD_DEVS = VecBuilder.fill(0.5, 0.5, 9999999);

    CommandSwerveDrivetrain drivetrain;
    LimelightHelpers.PoseEstimate currentPoseEstimateFinal;
    boolean useMT2;

    private boolean isMode1Set = false;
    private boolean isLLReady = false;

    public Localization(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.useMT2 = true;

        // Switch to internal IMU with external assist when enabled
        //LimelightHelpers.SetIMUAssistAlpha("limelight-left", 0.001);  // Adjust correction strength

        // Switch to internal IMU with external assist when enabled
        //LimelightHelpers.SetIMUAssistAlpha("limelight-right", 0.001);  // Adjust correction strength

        SmartDashboard.putBoolean("Conf/LL-Left_Enabled", true);
        SmartDashboard.putBoolean("Conf/LL-Right_Enabled", true);
        SmartDashboard.putBoolean("Conf/DisabledLocoEnabled", true);

    }

    public void addVisionMeasurement() {
        // First, tell Limelight your robot's current orientation
    double robotYaw = drivetrain.getGyroHeading();
    LimelightHelpers.SetRobotOrientation("limelight-left", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    LimelightHelpers.PoseEstimate limelightMeasurementLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
    LimelightHelpers.PoseEstimate limelightMeasurementRight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
    
    boolean doRejectUpdate = false;
    boolean doRejectLeft = false;
    boolean doRejectRight = false;
    
    if(Math.abs(drivetrain.getGyroRate()) > 360)
    {
        doRejectUpdate = true;
    }


    //if(drivetrain.swerveDataSupplier().get().swerveControlState == SwerveState.AIMING) doRejectUpdate = true;

    if(limelightMeasurementLeft != null)
    {
    if(limelightMeasurementLeft.tagCount == 0)
        {
            doRejectLeft = true;
        }
    }
    else doRejectLeft = true;

    if(limelightMeasurementRight != null)
    {
    if(limelightMeasurementRight.tagCount == 0)
        {
            doRejectRight = true;
        }
    }
    else doRejectRight = true;


    if(!doRejectUpdate)
    {
        if(!doRejectLeft && SmartDashboard.getBoolean("Conf/LL-Left_Enabled", true))
        {
            drivetrain.setVisionMeasurementStdDevs(MT2_STD_DEVS);

            drivetrain.addVisionMeasurement(
            limelightMeasurementLeft.pose,
            limelightMeasurementLeft.timestampSeconds
            );
        }

        if(!doRejectRight && SmartDashboard.getBoolean("Conf/LL-Right_Enabled", true))
        {
            drivetrain.setVisionMeasurementStdDevs(MT2_STD_DEVS);
            drivetrain.addVisionMeasurement(
            limelightMeasurementRight.pose,
            limelightMeasurementRight.timestampSeconds
            );
        }
    }
    }

    public void addVisionMeasurementMT1() {
        // First, tell Limelight your robot's current orientation
    double robotYaw = drivetrain.getGyroHeading();
    LimelightHelpers.SetRobotOrientation("limelight-left", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
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


    public void setImuMode1()
    {
        double robotYaw = drivetrain.getGyroHeading();


        LimelightHelpers.SetRobotOrientation("limelight-left", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetIMUMode("limelight-left", 1);
  
        
        LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetIMUMode("limelight-right", 1);
    }

    public void setImuMode2()
    {
        double robotYaw = drivetrain.getGyroHeading();

        LimelightHelpers.SetRobotOrientation("limelight-left", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetIMUMode("limelight-left", 2);
  
        
        LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetIMUMode("limelight-right", 2);
    }

    public void setImuMode0()
    {
        double robotYaw = drivetrain.getGyroHeading();


        LimelightHelpers.SetRobotOrientation("limelight-left", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetIMUMode("limelight-left", 0);
  
        
        LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetIMUMode("limelight-right", 0);
    }

    public boolean hasAnEstimate() {
        return (currentPoseEstimateFinal != null);
    }
    public double getLimelightTagHeading() {
        if (hasAnEstimate()) {return currentPoseEstimateFinal.pose.getRotation().getDegrees();}
        return 0;
    }

    public Pose2d getLimelightPose() {
        if (hasAnEstimate()) {return currentPoseEstimateFinal.pose;}
        return new Pose2d();
    }

    public LimelightHelpers.PoseEstimate getLimelightPoseEstimate() {
        if (hasAnEstimate()) {return currentPoseEstimateFinal;}
        return new LimelightHelpers.PoseEstimate();
    }

    public Command useMT2() {
        return new InstantCommand(()->{useMT2 = true;});
    }

    
    public Command useMT() {
        return new InstantCommand(()->{useMT2 = false;});
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
            //setImuMode2();
            //setImuMode0();
            isLLReady = true;
        }

        if(isLLReady) addVisionMeasurementMT1();
    }

    public void resetWithMT1()
    {
        double robotYaw = drivetrain.getGyroHeading();
        LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        //setImuMode1();

        LimelightHelpers.PoseEstimate limelightMeasurementRight = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");

        if(limelightMeasurementRight != null && limelightMeasurementRight.tagCount > 0)
        {
            if (limelightMeasurementRight.avgTagDist < 3.0) {
                drivetrain.resetPose(limelightMeasurementRight.pose);
            }
        }

    }
}