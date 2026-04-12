// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Utils;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Subsystems.Swerve.Utils.SwerveControlData;

/** Add your docs here. */
public class ShooterCalculator {

    Supplier<SwerveControlData> swerveDataSupplier;

    public ShooterCalculator(Supplier<SwerveControlData> swerveDataSupplier) {
        this.swerveDataSupplier = swerveDataSupplier;
    }

    public double calculateFlywheelSpeed(double distanceToTargetMeters, double launchAngleDegrees, double initialHeightMeters, double targetHeightMeters) {
        // Placeholder implementation
        // Replace with actual physics calculations to determine the required wheel speed
        double requiredSpeed = Math.sqrt(9.81 * distanceToTargetMeters); // Simplified example
        return requiredSpeed;
    }

    public double calculateHoodAngle(double distanceToTargetMeters, double initialHeightMeters, double targetHeightMeters) {
        // Placeholder implementation
        // Replace with actual calculations to determine the required hood angle
        double angle = Math.toDegrees(Math.atan2(targetHeightMeters - initialHeightMeters, distanceToTargetMeters));
        return angle/360;
    }


    public double calculateFlywheelSpeedFromCurrentPose()
    {

        double wheelSpeed = flywheelRPMFormula(swerveDataSupplier.get().distanceToHub);

        //wheelSpeed /= ShooterConstants.SHOOTER_VELOCITY_TRANSFER_COEFFICIENT;

        //SmartDashboard.putNumber("WheelSpeedCalculated", wheelSpeed);


        //if(SmartDashboard.getBoolean("UseManualShooterValues", false))
        //{
          //  wheelSpeed = SmartDashboard.getNumber("ManualFlywheelRpm", wheelSpeed);
        //}
        //SmartDashboard.putNumber("Conf/CalcRPM", TelemetryConstants.roundTelemetry(wheelSpeed));

        wheelSpeed /= 60;



        wheelSpeed = Math.max(ShooterConstants.MIN_FLYWHEEL_SPEED.in(RotationsPerSecond), Math.min(wheelSpeed, ShooterConstants.MAX_FLYWHEEL_SPEED.in(RotationsPerSecond))); // Clamp between min and max wheel speeds

        return wheelSpeed;
        //return RotationsPerSecond.of(35);
    }

        public double calculatePassSpeedFromCurrentPose()
    {
        double wheelSpeed = pasRPMFormula(swerveDataSupplier.get().xDistanceToHub);

        //wheelSpeed /= ShooterConstants.SHOOTER_VELOCITY_TRANSFER_COEFFICIENT;

        //SmartDashboard.putNumber("WheelSpeedCalculated", wheelSpeed);


        //if(SmartDashboard.getBoolean("UseManualShooterValues", false))
        //{
          //  wheelSpeed = SmartDashboard.getNumber("ManualFlywheelRpm", wheelSpeed);
        //}
        //SmartDashboard.putNumber("Conf/CalcPassRPM", TelemetryConstants.roundTelemetry(wheelSpeed));

        wheelSpeed /= 60;



        wheelSpeed = Math.max(ShooterConstants.MIN_FLYWHEEL_SPEED.in(RotationsPerSecond), Math.min(wheelSpeed, ShooterConstants.MAX_FLYWHEEL_SPEED.in(RotationsPerSecond))); // Clamp between min and max wheel speeds

        return wheelSpeed;
        //return RotationsPerSecond.of(35);
    }

    public double calculateHoodAngleFromCurrentPose()
    {
        double hoodAngle = hoodAngleFormula(swerveDataSupplier.get().distanceToHub);

        //SmartDashboard.putNumber("Conf/CalcHoodAngle", hoodAngle);

       // if(SmartDashboard.getBoolean("UseManualShooterValues", false))
        //{
         //   hoodAngle = SmartDashboard.getNumber("ManualHoodAngle", hoodAngle);
        //}

        double offsetDeg = ShooterConstants.HOOD_ANGLE_OFFSET.in(Degrees);
        double maxDeg = offsetDeg + ShooterConstants.MAX_HOOD_ANGLE.in(Degrees);
        hoodAngle = Math.max(offsetDeg, Math.min(hoodAngle, maxDeg)); // Clamp between min and max hood angles

        //System.out.println("Calculated Hood Angle: " + hoodAngle + "Distance: " + distance);
        //return Degrees.of(hoodAngle);
        return (hoodAngle - offsetDeg) / 360;
    }

        public double calculateRestFlywheelSpeedFromCurrentPose()
    {
        // Placeholder implementation
        // Replace with actual logic to calculate shooter wheel speed based on robot's current pose
        return ShooterConstants.FLYWHEEL_REST_SPEED.in(RotationsPerSecond);//.div((3.0))
    }

    public double calculateRestHoodAngleFromCurrentPose()
    {
        // Placeholder implementation
        // Replace with actual logic to calculate hood angle based on robot's current pose
        return ShooterConstants.MIN_HOOD_ANGLE.in(Rotations);//.div((3.0))
    }


    
    public double hoodAngleFormulaOLD(double x)
    {
        double a = -0.18;
        double b = 2.55;
        double c = -5.59;
        double d = 20.11;

        return a*Math.pow(x, 3) + b*Math.pow(x, 2) + c*x + d;
    } 

    public double hoodAngleFormula(double x)
    {
        double a = 0.0523475;
        double b = -0.693153;
        double c = 2.97895;
        double d = -3.31205;
        double f = -4.6815;
        double g = 25.40816;

        
        if (x < 2.25) {
            return 18;
        }
        else
        {
            return (((((a * x + b) * x + c) * x + d) * x + f) * x + g);

        }
    } 

     public static final double flywheelRPMFormula(double x)
    {
        double a = -29.72883;
        double b = 473.27393;
        double c = -2886.63609;
        double d = 8444.7507;
        double f = -11605.2592;
        double g = 7475.15141;

        if(x < 1.7)
        {
            return 1500;
        }

        double y = (((((a * x + b) * x + c) * x + d) * x + f) * x + g);

        if(x > 5)
        {
            y = 2631;
            y += 20*(x-5);
        }

        return y/ShooterConstants.SHOOTER_VELOCITY_TRANSFER_COEFFICIENT;
    }


    
    public static final double pasRPMFormula(double x)
    {
        double a = 350;
        double b = 1500;

        double y = a*x + b;

        return y;
    }

}
