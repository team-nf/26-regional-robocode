package frc.robot.Utils;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.Constants.Dimensions;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;

public class SwerveFieldContactSim {
    public double simLoopTimeSec = 0.001; 
    private int pointNumberForEdge = 8;

    private CommandSwerveDrivetrain m_swerveDrivetrain = null;
    private Supplier<Boolean> isIntakeDeployed = null;

    private static SwerveFieldContactSim instance;

    public void setSwerveDrivetrain(CommandSwerveDrivetrain swerveDrivetrain) {
        this.m_swerveDrivetrain = swerveDrivetrain;
        currentSimPose = m_swerveDrivetrain.getInitialStartPose();
        prevSimPose = m_swerveDrivetrain.getInitialStartPose();
    }

    public void setIntakeDeployedSupplier(Supplier<Boolean> isIntakeDeployed) {
        this.isIntakeDeployed = isIntakeDeployed;
    }

    public static SwerveFieldContactSim getInstance() {
        if (instance == null) {
            instance = new SwerveFieldContactSim();
        }
        return instance;
    }

    private Pose2d currentSimPose;
    private Pose2d prevSimPose;


    private SwerveFieldContactSim() {
    } 

    public CommandSwerveDrivetrain getSwerveDrivetrain() {
        return m_swerveDrivetrain;
    }

    public void handleSwerveSimFieldCollisions() {
            currentSimPose = m_swerveDrivetrain.getState().Pose;

            // Handle collisions with field boundaries
            double fieldXMin = 0.1;
            double fieldXMax = 16.33;
            double fieldYMin = 0.1;
            double fieldYMax = 8.15;

            double frontCollisionDistance = Dimensions.BUMPER_COLLISION_DISTANCE.in(Meters) / 2.0;
            double backCollisionDistance = Dimensions.BUMPER_COLLISION_DISTANCE.in(Meters) / 2.0;

            double frontOffset = 0.0;

            if(isIntakeDeployed != null && isIntakeDeployed.get()) {
                frontOffset = 0.2; // add hopper extension length
            }
            
            Pose2d frontLeftCorner = currentSimPose.transformBy(
                new Transform2d(
                    frontCollisionDistance * Math.cos(Math.PI/4) + frontOffset,
                    frontCollisionDistance * Math.sin(Math.PI/4),
                    Rotation2d.kZero
                )
            );

            Pose2d frontRightCorner = currentSimPose.transformBy(
                new Transform2d(
                    frontCollisionDistance * Math.cos(- Math.PI/4) + frontOffset,
                    frontCollisionDistance * Math.sin(- Math.PI/4),
                    Rotation2d.kZero
                )
            );

            Pose2d backLeftCorner = currentSimPose.transformBy(
                new Transform2d(
                    backCollisionDistance * Math.cos(3*Math.PI/4),
                    backCollisionDistance * Math.sin(3*Math.PI/4),
                    Rotation2d.kZero
                )
            );

            Pose2d backRightCorner = currentSimPose.transformBy(
                new Transform2d(
                    backCollisionDistance * Math.cos(- 3*Math.PI/4),
                    backCollisionDistance * Math.sin(- 3*Math.PI/4),
                    Rotation2d.kZero
                )
            );

            boolean[] collisionResults = checkFieldCollisionWithDirections(new Pose2d[] {
                frontLeftCorner,
                frontRightCorner,
                backLeftCorner,
                backRightCorner
            });

            double correctedX = currentSimPose.getX();
            double correctedY = currentSimPose.getY();
            Rotation2d correctedRotation2d = currentSimPose.getRotation();

            if (collisionResults[0] && correctedX < prevSimPose.getX()) correctedX = prevSimPose.getX();

            if (collisionResults[1] && correctedX > prevSimPose.getX()) correctedX = prevSimPose.getX();

            if (collisionResults[2] && correctedY < prevSimPose.getY()) correctedY = prevSimPose.getY();

            if (collisionResults[3] && correctedY > prevSimPose.getY()) correctedY = prevSimPose.getY();

            if (collisionResults[4] && correctedRotation2d.getDegrees() > prevSimPose.getRotation().getDegrees()) 
                correctedRotation2d = prevSimPose.getRotation();

            if (collisionResults[5] && correctedRotation2d.getDegrees() < prevSimPose.getRotation().getDegrees()) 
                correctedRotation2d = prevSimPose.getRotation();

            boolean collisionDetected = false;

            for (boolean b : collisionResults) {
                if (b) {
                    collisionDetected = true;
                    break;
                }
            }
            
            if (collisionDetected) {
                m_swerveDrivetrain.resetPose(new Pose2d(correctedX, correctedY, correctedRotation2d));
            }
            else prevSimPose = currentSimPose;
            
    }

    private StructArrayPublisher<Pose2d> cornerPosePublisher 
                = NetworkTableInstance.getDefault().getStructArrayTopic("Sim/Swerve/CornerPoses", Pose2d.struct).publish();
    
    private StructArrayPublisher<Pose2d> edgePosePublisher 
                = NetworkTableInstance.getDefault().getStructArrayTopic("Sim/Swerve/EdgePoses", Pose2d.struct).publish();

    public boolean[] checkFieldCollisionWithDirections(Pose2d[] robotCorners)
    {
        boolean x_plus_collided = false;
        boolean x_minus_collided = false;
        boolean y_plus_collided = false;
        boolean y_minus_collided = false;

        boolean ccw_rotation_collided = false;
        boolean cw_rotation_collided = false;

        boolean[] cornerCollisionCheck = {false, false, false, false};

        for (int i = 0; i < robotCorners.length; i++) {

            Pose2d point = robotCorners[i];

            double x = point.getX();
            double y = point.getY();

            boolean cornerXPlus = false;
            boolean cornerXMinus = false;
            boolean cornerYPlus = false;
            boolean cornerYMinus = false;

            for (double[] collisionYRange : Dimensions.getCollisionValuesForY(x)) {
                    if (collisionYRange[0] == -1.0 && collisionYRange[1] == -1.0) {
                        continue;
                    }
                    if (y > collisionYRange[0] && y < collisionYRange[1]) {
                        if ((y - collisionYRange[0]) < (collisionYRange[1] - y)) {
                            cornerYPlus = true;
                        } else {
                            cornerYMinus = true;
                        }
                    }
            }

            for (double[] collisionXRange : Dimensions.getCollisionValuesForX(y)) {
                    if (collisionXRange[0] == -1.0 && collisionXRange[1] == -1.0) {
                        continue;
                    }
                    if (x > collisionXRange[0] && x < collisionXRange[1]) {
                        if ((x - collisionXRange[0]) < (collisionXRange[1] - x)) {
                            cornerXPlus = true;
                        } else {
                            cornerXMinus = true;
                        }
                    }
            }
            if (cornerXPlus) x_plus_collided = true;
            if (cornerXMinus) x_minus_collided = true;
            if (cornerYPlus) y_plus_collided = true;
            if (cornerYMinus) y_minus_collided = true;
            cornerCollisionCheck[i] = cornerXMinus || cornerXPlus || cornerYMinus || cornerYPlus;
        }

        if(cornerCollisionCheck[0] && !cornerCollisionCheck[1] && !cornerCollisionCheck[2]) {
            ccw_rotation_collided = true;
        }

        if(cornerCollisionCheck[1] && !cornerCollisionCheck[0] && !cornerCollisionCheck[3]) {
            cw_rotation_collided = true;
        }

        if(cornerCollisionCheck[2] && !cornerCollisionCheck[3] && !cornerCollisionCheck[0]) {
            cw_rotation_collided = true;
        }

        if(cornerCollisionCheck[3] && !cornerCollisionCheck[2] && !cornerCollisionCheck[1]) {
            ccw_rotation_collided = true;
        }

        Pose2d[] robotCornersOrdered = new Pose2d[] {
            robotCorners[0],
            robotCorners[1],
            robotCorners[3],
            robotCorners[2]
        };

        Pose2d[] edgePoints = new Pose2d[pointNumberForEdge * robotCornersOrdered.length];

        for (int i = 0; i < robotCornersOrdered.length; i++) {
            Pose2d cornerA = robotCornersOrdered[i];
            Pose2d cornerB = robotCornersOrdered[(i + 1) % robotCornersOrdered.length];

            for (int j = 1; j <= pointNumberForEdge; j++) {
                double t = j / (pointNumberForEdge + 1.0);
                double x = cornerA.getX() + t * (cornerB.getX() - cornerA.getX());
                double y = cornerA.getY() + t * (cornerB.getY() - cornerA.getY());
                edgePoints[i * pointNumberForEdge + (j - 1)] = new Pose2d(x, y, cornerA.getRotation());
            }
        }

        for (Pose2d point : edgePoints) {
            double x = point.getX();
            double y = point.getY();

            for (double[] collisionYRange : Dimensions.getCollisionValuesForY(x)) {
                    if (collisionYRange[0] == -1.0 && collisionYRange[1] == -1.0) {
                        continue;
                    }
                    if (y > collisionYRange[0] && y < collisionYRange[1]) {
                        if ((y - collisionYRange[0]) < (collisionYRange[1] - y)) {
                            y_plus_collided = true;
                        } else {
                            y_minus_collided = true;
                        }
                    }
            }

            for (double[] collisionXRange : Dimensions.getCollisionValuesForX(y)) {
                    if (collisionXRange[0] == -1.0 && collisionXRange[1] == -1.0) {
                        continue;
                    }
                    if (x > collisionXRange[0] && x < collisionXRange[1]) {
                        if ((x - collisionXRange[0]) < (collisionXRange[1] - x)) {
                            x_plus_collided = true;
                        } else {
                            x_minus_collided = true;
                        }
                    }
            }
        }

        cornerPosePublisher.set(robotCorners);
        edgePosePublisher.set(edgePoints);

        return new boolean[] {x_minus_collided, x_plus_collided, y_minus_collided, y_plus_collided, ccw_rotation_collided, cw_rotation_collided};
    }

    public void reset() {
        if (m_swerveDrivetrain == null) return;
        currentSimPose = m_swerveDrivetrain.getInitialStartPose();
        prevSimPose = m_swerveDrivetrain.getInitialStartPose();
        m_swerveDrivetrain.resetPose(currentSimPose);
    }

    public Pose2d getPose() {
        return currentSimPose;
    }
}