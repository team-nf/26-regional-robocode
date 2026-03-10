// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.PoseConstants;
import frc.robot.constants.ShooterConstants;

public class ShooterCalculator {

    private final Supplier<Pose2d> poseSupplier;

    public ShooterCalculator(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    private Translation2d getHubTranslation() {
        return Boolean.TRUE.equals(Container.isBlue)
            ? PoseConstants.BLUE_HUB_AIM_POSE.getTranslation()
            : PoseConstants.RED_HUB_AIM_POSE.getTranslation();
    }

    private double getDistanceToHub() {
        return getHubTranslation().getDistance(poseSupplier.get().getTranslation());
    }

    private double getXDistanceToHub() {
        return Math.abs(getHubTranslation().getX() - poseSupplier.get().getTranslation().getX());
    }

    public double calculateFlywheelSpeedFromCurrentPose() {
        double wheelSpeed = flywheelRPMFormula(getDistanceToHub());
        wheelSpeed /= 60;
        wheelSpeed = Math.max(
            ShooterConstants.MIN_FLYWHEEL_SPEED.in(RotationsPerSecond),
            Math.min(wheelSpeed, ShooterConstants.MAX_FLYWHEEL_SPEED.in(RotationsPerSecond))
        );
        return wheelSpeed;
    }

    public double calculatePassSpeedFromCurrentPose() {
        double wheelSpeed = passRPMFormula(getXDistanceToHub());
        wheelSpeed /= 60;
        wheelSpeed = Math.max(
            ShooterConstants.MIN_FLYWHEEL_SPEED.in(RotationsPerSecond),
            Math.min(wheelSpeed, ShooterConstants.MAX_FLYWHEEL_SPEED.in(RotationsPerSecond))
        );
        return wheelSpeed;
    }

    public double calculateHoodAngleFromCurrentPose() {
        double hoodAngle = hoodAngleFormula(getDistanceToHub());

        double offsetDeg = ShooterConstants.HOOD_ANGLE_OFFSET.in(Degrees);
        double maxDeg = offsetDeg + ShooterConstants.MAX_HOOD_ANGLE.in(Degrees);

        hoodAngle = Math.max(offsetDeg, Math.min(hoodAngle, maxDeg));

        return (hoodAngle - offsetDeg) / 360;
    }

    public double calculateRestFlywheelSpeed() {
        return ShooterConstants.FLYWHEEL_REST_SPEED.in(RotationsPerSecond);
    }

    public double calculateRestHoodAngle() {
        return ShooterConstants.MIN_HOOD_ANGLE.in(Rotations);
    }

    public double calculatePassHoodAngle() {
        return ShooterConstants.PASS_HOOD_ANGLE.in(Rotations);
    }

    public double hoodAngleFormula(double x) {
        double a = 0.0523475;
        double b = -0.693153;
        double c = 2.97895;
        double d = -3.31205;
        double f = -4.6815;
        double g = 25.40816;

        if (x < 2.25) {
            return 18;
        } else {
            return (((((a * x + b) * x + c) * x + d) * x + f) * x + g);
        }
    }

    public static double flywheelRPMFormula(double x) {
        double a = -29.72883;
        double b = 473.27393;
        double c = -2886.63609;
        double d = 8444.7507;
        double f = -11605.2592;
        double g = 7475.15141;

        if (x < 1.7) {
            return 1500;
        }

        double y = (((((a * x + b) * x + c) * x + d) * x + f) * x + g);

        if (x > 5) {
            y = 2631;
            y += 20 * (x - 5);
        }

        return y / ShooterConstants.SHOOTER_VELOCITY_TRANSFER_COEFFICIENT;
    }

    public static double passRPMFormula(double x) {
        double a = 350;
        double b = 1500;

        double y = a * x + b;

        return y;
    }
}
