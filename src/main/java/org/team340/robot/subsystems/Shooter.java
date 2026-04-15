package org.team340.robot.subsystems;

import static org.team340.robot.util.ShootParams.shooterVelocityMap;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

/**
 * The robot's shooter.
 */
@Logged
public final class Shooter extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("shooter");

    private static final TunableDouble atVelocityEpsilon = tunables.value("atVelocityEpsilon", 4.0);

    private final TalonFX lead;
    private final TalonFX follow;

    private final StatusSignal<Double> closedLoopError;

    private final VelocityVoltage velocityControl;
    private final Follower followControl;

    public Shooter() {
        this.lead = new TalonFX(RobotMap.SHOOTER_LEAD_MOTOR, RobotMap.CANBus);
        this.follow = new TalonFX(RobotMap.SHOOTER_FOLLOW_MOTOR, RobotMap.CANBus);

        configureMotors();

        closedLoopError = lead.getClosedLoopError();

        PhoenixUtil.run(() -> BaseStatusSignal.setUpdateFrequencyForAll(100, closedLoopError));
        PhoenixUtil.run(() -> BaseStatusSignal.setUpdateFrequencyForAll(500, lead.getMotorVoltage()));
        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                lead.getVelocity(),
                lead.getClosedLoopReference()
            )
        );
        PhoenixUtil.run(() -> ParentDevice.optimizeBusUtilizationForAll(4, lead, follow));

        velocityControl = new VelocityVoltage(0.0);
        velocityControl.EnableFOC = true;
        velocityControl.UpdateFreqHz = 0.0;

        followControl = new Follower(lead.getDeviceID(), MotorAlignmentValue.OppositeOfLeader);

        tunables.add("leadMotor", lead);
        tunables.add("followMotor", follow);
    }

    @Override
    public void periodic() {
        closedLoopError.refresh();
        follow.setControl(followControl);
    }

    /**
     * shooter's closed loop error is less than allowed error.
     * @return true if less
     */
    public boolean atVelocity() {
        return Math.abs(closedLoopError.getValueAsDouble()) <= atVelocityEpsilon.get();
    }

    /**
     * Run the shooter to target a specific distance based on a preset interpolating map.
     * @param distance The distance to target in meters.
     */
    public Command targetDistance(final DoubleSupplier distance) {
        return runVelocity(() -> shooterVelocityMap.get(distance.getAsDouble())).withName("Shooter.targetDistance()");
    }

    /**
     * Internal method to run the shooter at a specified velocity.
     * @param velocity The velocity in rotations/second at the rotor (gearing not included).
     */
    private Command runVelocity(final DoubleSupplier velocity) {
        return commandBuilder("Shooter.runVelocity()")
            .onExecute(() -> {
                velocityControl.withVelocity(velocity.getAsDouble());
                lead.setControl(velocityControl);
            })
            .onEnd(() -> {
                lead.stopMotor();
            });
    }

    private void configureMotors() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 0.35;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.129;
        config.Slot0.kA = 0.0;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        PhoenixUtil.run(() -> lead.clearStickyFaults());
        PhoenixUtil.run(() -> lead.getConfigurator().apply(config));

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> follow.clearStickyFaults());
        PhoenixUtil.run(() -> follow.getConfigurator().apply(config));
    }
}
