package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
 * The robot's floor roller indexer.
 */
@Logged
public final class Indexer extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("indexer");

    private static enum State {
        FEED(78.0),
        BARF(-75.0),
        CREEP(-10.0);

        public final TunableDouble speed;

        private State(final double speed) {
            this.speed = tunables.value("speed/" + name(), speed);
        }
    }

    private final TalonFX motor;

    private final VelocityTorqueCurrentFOC velocityControl;

    public Indexer() {
        this.motor = new TalonFX(RobotMap.INDEXER_MOTOR, RobotMap.CANBus);

        configureMotor();

        PhoenixUtil.run(() -> motor.getTorqueCurrent().setUpdateFrequency(500));
        PhoenixUtil.run(() -> BaseStatusSignal.setUpdateFrequencyForAll(50, motor.getVelocity()));
        PhoenixUtil.run(() -> ParentDevice.optimizeBusUtilizationForAll(4, motor));

        velocityControl = new VelocityTorqueCurrentFOC(0.0);
        velocityControl.UpdateFreqHz = 0.0;

        tunables.add("motor", motor);

        // Enum warmup
        State.FEED.speed.get();
    }

    /**
     * Feeds the shooter.
     * @return feed command
     */
    public Command feed() {
        return runState(State.FEED).withName("Indexer.feed()");
    }

    /**
     * Barfs back into the hopper.
     * @return barf command
     */
    public Command barf() {
        return runState(State.BARF).withName("Indexer.barf()");
    }

    /**
     * Slowly reverses to prevent the note from feeding prematurely.
     * @return creep command
     */
    public Command creep() {
        return runState(State.CREEP).withName("Indexer.creep()");
    }

    /**
     * Internal method to run the motor as configured for the specified state.
     * @param state The indexer state to target.
     * @return runState command
     */
    private Command runState(final State state) {
        return commandBuilder("Indexer.run()")
            .onExecute(() -> {
                velocityControl.withVelocity(state.speed.get());
                motor.setControl(velocityControl);
            })
            .onEnd(() -> {
                motor.stopMotor();
            });
    }

    private void configureMotor() {
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

        PhoenixUtil.run(() -> motor.clearStickyFaults());
        PhoenixUtil.run(() -> motor.getConfigurator().apply(config));
    }
}
