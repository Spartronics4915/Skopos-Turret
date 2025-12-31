package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.superstructure.HoodSubsystem;
import frc.robot.subsystems.superstructure.IntakeSubsystem;
import frc.robot.subsystems.superstructure.ShooterSubsystem;
import frc.robot.utilities.ModeSwitchHandler;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.IO.*;

public class RobotContainer {

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final TurretSubsystem turretSubsystem = new TurretSubsystem();
    
    private final CommandXboxController driverController = new CommandXboxController(DRIVE_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
    public DriveCommand driveCommand = new DriveCommand(driverController, swerveSubsystem);

    public RobotContainer() {
        configureBindings();

        ModeSwitchHandler.EnableModeSwitchHandler(
            hoodSubsystem,
            shooterSubsystem,
            intakeSubsystem
        );
    }

    private void configureBindings() {
        swerveSubsystem.setDefaultCommand(driveCommand);

        ChassisSpeeds driverNudgeUp = new ChassisSpeeds(0.25, 0, 0);
        ChassisSpeeds driverNudgeDown = new ChassisSpeeds(-0.25, 0, 0);
        ChassisSpeeds driverNudgeLeft = new ChassisSpeeds(0, 0.25, 0);
        ChassisSpeeds driverNudgeRight = new ChassisSpeeds(0, -0.25, 0);

        driverController.povUp().whileTrue(
            Commands.run(() -> {
                swerveSubsystem.drive(driverNudgeUp);
            })
        );

        driverController.povDown().whileTrue(
            Commands.run(() -> {
                swerveSubsystem.drive(driverNudgeDown);
            })
        );

        driverController.povLeft().whileTrue(
            Commands.run(() -> {
                swerveSubsystem.drive(driverNudgeLeft);
            })
        );

        driverController.povRight().whileTrue(
            Commands.run(() -> {
                swerveSubsystem.drive(driverNudgeRight);
            })
        );

        driverController.b().onTrue(
            Commands.runOnce(() -> {
                IS_FIELD_RELATIVE = !IS_FIELD_RELATIVE;
            })
        );

        driverController.a().onTrue(
            Commands.runOnce(() -> {
                TELEOP_HEADING_OFFSET = swerveSubsystem.getPose().getRotation();
            })
        );

        driverController.x().whileTrue(
            Commands.runOnce(() -> {

            })
        );

        //#endregion

        //#region Operator Controller Bindings

        operatorController.povUp().whileTrue(
            Commands.run(() ->
                hoodSubsystem.incrementAngle(Rotation2d.fromDegrees(-HOOD_STEP))
            )
        );

        operatorController.povDown().whileTrue(
            Commands.run(() ->
                hoodSubsystem.incrementAngle(Rotation2d.fromDegrees(HOOD_STEP))
            )
        );

        operatorController.povLeft().whileTrue(
            Commands.run(() ->
                turretSubsystem.incrementAngle(Rotation2d.fromDegrees(-TURRET_STEP))
            )
        );

        operatorController.povRight().whileTrue(
            Commands.run(() ->
                turretSubsystem.incrementAngle(Rotation2d.fromDegrees(TURRET_STEP))
            )
        );

        operatorController.leftTrigger(0.01).whileTrue(
            Commands.run(() -> {
                double triggerRaw = operatorController.getLeftTriggerAxis();
                double trigger = applyResponseCurve(MathUtil.applyDeadband(triggerRaw, TRIGGER_DEADBAND));
                intakeSubsystem.setSpeed(trigger * 10);
            }).finallyDo(
                () -> intakeSubsystem.setSpeed(0)
            )
        );

        operatorController.rightTrigger(0.01).whileTrue(
            Commands.run(() -> {
                double triggerRaw = operatorController.getRightTriggerAxis();
                double trigger = applyResponseCurve(MathUtil.applyDeadband(triggerRaw, TRIGGER_DEADBAND));
                shooterSubsystem.setSetpoint(trigger * 80);
            }).finallyDo(
                () -> shooterSubsystem.setSetpoint(0)
            )
        );

        //#endregion

        SmartDashboard.putData("Reset Dynamics", Commands.sequence(
            hoodSubsystem.setSetpointCommand(Rotation2d.fromDegrees(0)),
            turretSubsystem.setSetpointCommand(Rotation2d.fromDegrees(0))
        ));
    }
}
