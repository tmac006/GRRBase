package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Hopper;
import org.team340.robot.subsystems.Indexer;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Shooter;
import org.team340.robot.subsystems.Swerve;
import org.team340.robot.subsystems.Uptake;

/**
 * The Routines class contains command compositions that require
 * multiple subsystems, such as sequences or parallel command groups.
 */
public final class Routines {

    private static final TunableTable tunables = Tunables.getNested("routines");

    private static final TunableDouble staticShootDistance = tunables.value("staticShootDistance", 2.0);

    private final Hopper hopper;
    private final Indexer indexer;
    private final Intake intake;
    private final Shooter shooter;
    private final Swerve swerve;
    private final Uptake uptake;

    public Routines(Robot robot) {
        hopper = robot.hopper;
        indexer = robot.indexer;
        intake = robot.intake;
        shooter = robot.shooter;
        swerve = robot.swerve;
        uptake = robot.uptake;
    }

    /**
     * Deploys the intake and extends the hopper to channel fuel.
     */
    public Command intake() {
        return parallel(hopper.extend(), intake.intake()).withName("Routines.intake()");
    }

    /**
     * Barfs fuel out of the intake.
     */
    public Command barf() {
        return parallel(
            sequence(waitSeconds(0.25), indexer.barf().asProxy(), uptake.barf().asProxy()),
            intake.barf()
        ).withName("Routines.barf()");
    }

    /**
     * Finishes barfing fuel by purging then stowing.
     */
    public Command finishBarf() {
        return sequence(intake.purge().until(intake::isStowed), intake.stow()).withName("Routines.finishBarf()");
    }

    /**
     * Spins up the shooter, holds the note back with a slow reverse creep on the
     * indexer and uptake, then feeds once the shooter is at velocity.
     */
    public Command shoot() {
        return parallel(
            shooter.targetDistance(swerve::targetDistance),
            sequence(
                parallel(indexer.creep(), uptake.creep()).until(shooter::atVelocity),
                parallel(indexer.feed(), uptake.feed())
            )
        ).withName("Routines.shoot()");
    }

    /**
     * Shoots at the hub from a fixed distance, as a backup.
     */
    public Command staticShoot() {
        return parallel(
            shooter.targetDistance(staticShootDistance),
            parallel(indexer.feed(), uptake.feed())
        ).withName("Routines.staticShoot()");
    }

    /**
     * Runs all subsystems to check functionality.
     */
    public Command testSequence() {
        Timer timer = new Timer();

        final double SHOOT_TIME = 4.0;
        final double MAX_SHOOT_DISTANCE = 12.0;
        DoubleSupplier targetDistance = () -> timer.get() * (MAX_SHOOT_DISTANCE / SHOOT_TIME);

        return sequence(
            intake.intake().asProxy().withTimeout(2.0),
            parallel(indexer.feed(), uptake.feed(), intake.agitate()).asProxy().withTimeout(2.0),
            parallel(intake.stow(), shooter.targetDistance(targetDistance))
                .beforeStarting(timer::restart)
                .asProxy()
                .withTimeout(SHOOT_TIME)
        ).withName("Routines.testSequence()");
    }
}
