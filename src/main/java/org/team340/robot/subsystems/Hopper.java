package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
 * The robot's hopper, which channels fuel into the intake.
 */
@Logged
public final class Hopper extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("hopper");

    private static enum State {
        EXTEND(0.25),
        RETRACT(-0.1);

        public final TunableDouble position;

        private State(final double position) {
            this.position = tunables.value("positions/" + name(), position);
        }
    }

    private final TalonFX motor;

    private final MotionMagicVoltage positionControl;

    public Hopper() {
        this.motor = new TalonFX(RobotMap.HOPPER_MOTOR, RobotMap.CANBus);

        configureMotor();

        PhoenixUtil.run(() -> BaseStatusSignal.setUpdateFrequencyForAll(50, motor.getPosition()));
        PhoenixUtil.run(() -> ParentDevice.optimizeBusUtilizationForAll(4, motor));

        positionControl = new MotionMagicVoltage(0.0);
        positionControl.EnableFOC = true;
        positionControl.UpdateFreqHz = 0.0;

        tunables.add("motor", motor);

        // Enum warmup
        State.EXTEND.position.get();
    }

    /**
     * Extends the hopper down to channel fuel into the intake.
     * @return extend command
     */
    public Command extend() {
        return runState(State.EXTEND).withName("Hopper.extend()");
    }

    /**
     * Retracts the hopper to the stowed position.
     * @return retract command
     */
    public Command retract() {
        return runState(State.RETRACT).withName("Hopper.retract()");
    }

    private Command runState(final State state) {
        return commandBuilder("Hopper.runState()")
            .onExecute(() -> {
                positionControl.withPosition(state.position.get());
                motor.setControl(positionControl);
            })
            .onEnd(motor::stopMotor);
    }

    private void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimit = 20.0;

        config.MotionMagic.MotionMagicCruiseVelocity = 2.0;
        config.MotionMagic.MotionMagicAcceleration = 8.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = 60.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.12;
        config.Slot0.kA = 0.0;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.3;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.15;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> motor.clearStickyFaults());
        PhoenixUtil.run(() -> motor.getConfigurator().apply(config));
    }
}
