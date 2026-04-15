package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;

/**
 * The robot's uptake.
 */
@Logged
public final class Uptake extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("uptake");

    private static enum State {
        FEED(78.0),
        BARF(-75.0);

        public final TunableDouble speed;

        private State(final double speed) {
            this.speed = tunables.value("speed/" + name(), speed);
        }
    }

    private final TalonFX lead;
    private final TalonFX follow;

    private final VelocityTorqueCurrentFOC velocityControl;
    private final Follower followControl;

    public Uptake() {
        this.lead = new TalonFX(RobotMap.UPTAKE_LEAD_MOTOR, RobotMap.CANBus);
        this.follow = new TalonFX(RobotMap.UPTAKE_FOLLOW_MOTOR, RobotMap.CANBus);

        configureMotors();

        PhoenixUtil.run(() -> lead.getTorqueCurrent().setUpdateFrequency(500));
        PhoenixUtil.run(() -> BaseStatusSignal.setUpdateFrequencyForAll(50, lead.getVelocity()));
        PhoenixUtil.run(() -> ParentDevice.optimizeBusUtilizationForAll(4, lead, follow));

        velocityControl = new VelocityTorqueCurrentFOC(0.0);
        velocityControl.UpdateFreqHz = 0.0;

        followControl = new Follower(lead.getDeviceID(), MotorAlignmentValue.Aligned);

        tunables.add("leadMotor", lead);
        tunables.add("followMotor", follow);

        // Enum warmup
        State.FEED.speed.get();
    }

    @Override
    public void periodic() {
        follow.setControl(followControl);
    }

    /**
     * Feeds the shooter.
     * @return feed command
     */
    public Command feed() {
        return runState(State.FEED).withName("Uptake.feed()");
    }

    /**
     * Barfs back into the hopper.
     * @return barf command
     */
    public Command barf() {
        return runState(State.BARF).withName("Uptake.barf()");
    }

    /**
     * Internal method to run the motors as configured for the specified state.
     * @param state The uptake state to target.
     * @return runState command
     */
    private Command runState(final State state) {
        return commandBuilder("Uptake.run()")
            .onExecute(() -> {
                velocityControl.withVelocity(state.speed.get());
                lead.setControl(velocityControl);
            })
            .onEnd(() -> {
                lead.stopMotor();
            });
    }

    private void configureMotors() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 120.0;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 11.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 8.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> lead.clearStickyFaults());
        PhoenixUtil.run(() -> lead.getConfigurator().apply(config));

        PhoenixUtil.run(() -> follow.clearStickyFaults());
        PhoenixUtil.run(() -> follow.getConfigurator().apply(config));
    }
}
