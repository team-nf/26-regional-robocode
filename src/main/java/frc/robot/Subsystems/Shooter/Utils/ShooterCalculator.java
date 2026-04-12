// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Utils;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.*;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Swerve.Utils.SwerveControlData;

public class ShooterCalculator {

    Supplier<SwerveControlData> swerveDataSupplier;

    public ShooterCalculator(Supplier<SwerveControlData> swerveDataSupplier) {
        this.swerveDataSupplier = swerveDataSupplier;
    }

    public double calculateFlywheelSpeedFromCurrentPose()
    {

        double wheelSpeed = flywheelRPMFormula(swerveDataSupplier.get().distanceToHub);
        wheelSpeed /= 60;
        wheelSpeed = Math.max(ShooterConstants.MIN_FLYWHEEL_SPEED.in(RotationsPerSecond), Math.min(wheelSpeed, ShooterConstants.MAX_FLYWHEEL_SPEED.in(RotationsPerSecond)));
        return wheelSpeed;
    }

    public double calculatePassSpeedFromCurrentPose()
    {
        double wheelSpeed = pasRPMFormula(swerveDataSupplier.get().xDistanceToHub);
        wheelSpeed /= 60;
        wheelSpeed = Math.max(ShooterConstants.MIN_FLYWHEEL_SPEED.in(RotationsPerSecond), Math.min(wheelSpeed, ShooterConstants.MAX_FLYWHEEL_SPEED.in(RotationsPerSecond)));
        return wheelSpeed;
    }

    public double calculateHoodAngleFromCurrentPose()
    {
        double hoodAngle = hoodAngleFormula(swerveDataSupplier.get().distanceToHub);

        double offsetDeg = ShooterConstants.HOOD_ANGLE_OFFSET.in(Degrees);
        double maxDeg = offsetDeg + ShooterConstants.MAX_HOOD_ANGLE.in(Degrees);
        hoodAngle = Math.max(offsetDeg, Math.min(hoodAngle, maxDeg));

        return (hoodAngle - offsetDeg) / 360;
    }

    public double calculateRestFlywheelSpeedFromCurrentPose()
    {
        return ShooterConstants.FLYWHEEL_REST_SPEED.in(RotationsPerSecond);
    }

    public double calculateRestHoodAngleFromCurrentPose()
    {
        return ShooterConstants.MIN_HOOD_ANGLE.in(Rotations);
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
