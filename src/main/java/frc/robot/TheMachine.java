package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TheMachineConstants;
import frc.robot.constants.States.TheMachineStates.TheMachineState;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TheMachine {

    private final ShooterSubsystem shooterSubsystem;
    private final HopperSubsystem hopperSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final FeederSubsystem feederSubsystem;

    private TheMachineState state = TheMachineState.IDLE_RETRACTED;

    private boolean intakeWithOffset = false;
    
    public TheMachine(ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem, IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.feederSubsystem = feederSubsystem;
    }

    public void zero() {
        shooterSubsystem.zero();
        feederSubsystem.zero();
        hopperSubsystem.zero();
        intakeSubsystem.close();
        state = TheMachineState.ZERO;
    }

    public void idleRetracted() {
        shooterSubsystem.rest();
        feederSubsystem.reverse();
        hopperSubsystem.zero();
        intakeSubsystem.close();
        state = TheMachineState.IDLE_RETRACTED;
    }

    public void idleDeployed() {
        shooterSubsystem.rest();
        feederSubsystem.reverse();
        hopperSubsystem.zero();
        intakeSubsystem.deploy();
        state = TheMachineState.IDLE_DEPLOYED;
    }

    public void idle() {
        shooterSubsystem.rest();
        feederSubsystem.reverse();
        hopperSubsystem.zero();
        intakeSubsystem.idleBetween();
        state = TheMachineState.IDLE;
    }

    public void intake() {
        shooterSubsystem.rest();
        feederSubsystem.reverse();
        hopperSubsystem.zero();
        if (intakeWithOffset) {
            intakeSubsystem.intakeWithOffset();
        } else {
            intakeSubsystem.intake();
        }
        state = TheMachineState.INTAKE;
    }

    public void getReady() {
        shooterSubsystem.shootFromPose();
        feederSubsystem.reverse();
        hopperSubsystem.reverse();
        intakeSubsystem.deploy();
        state = TheMachineState.GET_READY;
    }

    public void shoot() {
        shooterSubsystem.shootFromPose();
        feederSubsystem.feed();
        hopperSubsystem.feed();
        intakeSubsystem.feed();
        state = TheMachineState.SHOOT;
    }

    public void getReadyPass() {
        shooterSubsystem.pass();
        feederSubsystem.reverse();
        hopperSubsystem.reverse();
        intakeSubsystem.deploy();
        state = TheMachineState.GET_READY_PASS;
    }

    public void pass() {
        shooterSubsystem.pass();
        feederSubsystem.feed();
        hopperSubsystem.feed();
        intakeSubsystem.feed();
        state = TheMachineState.PASS;
    }

    public void reverse() {
        shooterSubsystem.zero();
        feederSubsystem.reverse();
        hopperSubsystem.reverse();
        intakeSubsystem.reverse();
        state = TheMachineState.REVERSE;
    }

    public void test() {
        shooterSubsystem.test();
        feederSubsystem.feed();
        hopperSubsystem.feed();
        intakeSubsystem.feed();
        state = TheMachineState.TEST;
    }

    public void changeIntakeMode() {
        intakeWithOffset = !intakeWithOffset;
    }

    public boolean isIntakeWithOffset() {
        return intakeWithOffset;
    }

    public boolean isState(TheMachineState state) {
        return this.state == state;
    }

    public boolean isShooterReady()
    {
        return shooterSubsystem.isReadyToShoot();
    }

private StructPublisher<Pose3d> hoodPosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Sim/HoodPose", Pose3d.struct).publish();

  private StructPublisher<Pose3d> funnelPosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Sim/FunnelPose", Pose3d.struct).publish();

  private StructPublisher<Pose3d> intakePosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Sim/IntakePose", Pose3d.struct).publish();


  public void calculateSubsytemPoses() {

    double hoodAngle = shooterSubsystem.getHoodPosition();

    Pose3d hoodPose = TheMachineConstants.HOOD_RETRACTED_POSE
                            .rotateAround(TheMachineConstants.HOOD_RETRACTED_POSE.getTranslation(),new Rotation3d(0, Math.toRadians(hoodAngle*360), 0));

    double intakeArmAngle = intakeSubsystem.getIntakeArmPosition();

    Pose3d intakePose = TheMachineConstants.INTAKE_DEPLOYED_POSE
                          .rotateAround(TheMachineConstants.INTAKE_DEPLOYED_POSE.getTranslation(), new Rotation3d(0, -Math.toRadians(intakeArmAngle*360 + 60), 0));

    Distance funnelExtension = Meters.of(0.0);

    if(intakeArmAngle*360 < 30) funnelExtension = Meters.of(0.3125);
    else funnelExtension = Meters.of(0.3125).times(Math.cos(Math.toRadians(intakeArmAngle*360 - 30)));
    
    Pose3d funnelPose = TheMachineConstants.FUNNEL_RETRACTED_POSE
                          .plus(new Transform3d(funnelExtension.in(Meters), 0, 0, new Rotation3d(0, 0, 0)));
    
    hoodPosePublisher.set(hoodPose);
    intakePosePublisher.set(intakePose);
    funnelPosePublisher.set(funnelPose);
  }

  public SubsystemBase[] getSubsystems() {
    return new SubsystemBase[] {shooterSubsystem, feederSubsystem, hopperSubsystem, intakeSubsystem};
  }
}
