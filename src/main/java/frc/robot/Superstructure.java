package frc.robot;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants.IntakeSpeed;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.superstructure.HoodSubsystem;
import frc.robot.subsystems.superstructure.IntakeSubsystem;
import frc.robot.subsystems.superstructure.ShooterSubsystem;
import frc.robot.subsystems.superstructure.TurretSubsystem;

import static edu.wpi.first.units.Units.Millimeter;
import static frc.robot.Constants.SuperstructureConstants.*;

public class Superstructure {
    private SwerveSubsystem swerveSubsystem;
    private HoodSubsystem hoodSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private TurretSubsystem turretSubsystem;

    private LaserCan intakeLC;
    private Trigger ballDetectTrigger = new Trigger(this::ballDetect).debounce(INTAKE_LC_DEBOUNCE);

    public Superstructure(
        SwerveSubsystem swerveSubsystem, 
        HoodSubsystem hoodSubsystem, 
        ShooterSubsystem shooterSubsystem, 
        IntakeSubsystem intakeSubsystem, 
        TurretSubsystem turretSubsystem
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.turretSubsystem = turretSubsystem;

        this.intakeLC = new LaserCan(INTAKE_LC_ID);

        try {
            intakeLC.setRangingMode(LaserCan.RangingMode.SHORT);
            intakeLC.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4)); // prev numbers that worked(8, 8, 4, 4)
            intakeLC.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public boolean ballDetect(){
        if (RobotBase.isSimulation()) {
            return false;
        }

        Measurement measurement = intakeLC.getMeasurement();
        if (measurement == null) {
            return false;
        }

        return  measurement.distance_mm < INTAKE_LC_TRIGGER_DISTANCE.in(Millimeter);
    }

    public Command intake() {
        return Commands.sequence(
            Commands.either(
                intakeSubsystem.setPresetCommand(IntakeSpeed.SHOOT),
                Commands.none(),
                () -> ballDetectTrigger.getAsBoolean()
            ),
            Commands.waitUntil(ballDetectTrigger.negate()),
            intakeSubsystem.setPresetCommand(IntakeSpeed.SLOW),
            Commands.waitUntil(ballDetectTrigger).withTimeout(INTAKE_TIMEOUT),
            intakeSubsystem.setPresetCommand(IntakeSpeed.STOP)
        );
    }
}
