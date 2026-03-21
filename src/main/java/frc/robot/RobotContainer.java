// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.autoShooter;
import frc.robot.commands.runShooter;
import frc.robot.commands.spinToWin;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intake = new Intake(16,12);
    public final Shooter shoot = new Shooter(11,10,14, 15,18, 17);

    private final SlewRateLimiter filterX = new SlewRateLimiter(MaxSpeed/5);
    private final SlewRateLimiter filterY = new SlewRateLimiter(MaxSpeed/5);


    public RobotContainer() {
        configureBindings();
        registerAutoCommands();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left) filterY.calculate
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        //driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driver.rightTrigger().toggleOnTrue(new runShooter(shoot, driver, intake));
        driver.leftTrigger().toggleOnTrue(new spinToWin(drivetrain, ()-> -driver.getLeftY() * MaxSpeed/5,()-> -driver.getLeftX() * MaxSpeed/5));
        driver.x().onTrue(new InstantCommand(() -> shoot.incSpeed(true)));
        driver.y().onFalse(new InstantCommand(() -> shoot.decSpeed(true)));

        operator.x().whileTrue(new InstantCommand(() -> intake.setWheelSpeed(-0.8)));
        operator.x().onFalse(new InstantCommand(() -> intake.setWheelSpeed(0)));
        operator.y().onTrue(new InstantCommand(() -> intake.wristExtend()));
        //operator.b().onTrue(new InstantCommand(()-> intake.wristRetract()));
        operator.rightBumper().whileTrue(Commands.run(()-> intake.wristShake()));

        // joystick.x().whileTrue(new InstantCommand(() -> shoot.moveHood(-1)));
        // joystick.x().onFalse(new InstantCommand(() -> shoot.moveHoo(0)));
        // joystick.y().whileTrue(new InstantCommand(() -> shoot.moveHood(1)));
        // joystick.y().onFalse(new InstantCommand(() -> shoot.moveHood(0)));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void registerAutoCommands() {
        NamedCommands.registerCommand("intakeOut", new InstantCommand(() -> intake.wristExtend()));
        NamedCommands.registerCommand("spinToWin", new spinToWin(drivetrain, ()-> 0, ()-> 0));
        NamedCommands.registerCommand("autoShoot", new autoShooter(shoot, driver, intake));
        

    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Blue");
    }

    public void putElastic(){
        SmartDashboard.putNumber("analog 1", shoot.getVoltage1());
        SmartDashboard.putNumber("analog 2", shoot.getVoltage2());
        SmartDashboard.putNumber("targetRPM", shoot.getRPS());
        SmartDashboard.putNumber("distance", shoot.getDistance());
        SmartDashboard.putNumber("shoot speed", shoot.getSSpeed());
        SmartDashboard.putNumber("Intake position", intake.getIntakePosition());
        SmartDashboard.putNumber("Shoot Inc", shoot.getSpeed());

    }
}

