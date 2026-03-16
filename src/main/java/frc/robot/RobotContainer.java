// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.alignmentMode;
import frc.robot.commands.runShooter;
import frc.robot.commands.shootSys;
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

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-operator.getLeftY() * MaxSpeed/5) // Drive forward with negative Y (forward)
                    .withVelocityY(-operator.getLeftX() * MaxSpeed/5) // Drive left with negative X (left)
                    .withRotationalRate(-operator.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        //intake
        operator.x().whileTrue(new InstantCommand(() -> intake.setWheelSpeed(-0.8)));
        operator.x().onFalse(new InstantCommand(() -> intake.setWheelSpeed(0)));
        operator.y().onTrue(new InstantCommand(() -> intake.wristExtend()));

        //joystick.b().onTrue(new InstantCommand(() -> intake.wristRetract()));

        // joystick.x().whileTrue(new InstantCommand(() -> shoot.moveHood(-1)));
        // joystick.x().onFalse(new InstantCommand(() -> shoot.moveHoo(0)));
        // joystick.y().whileTrue(new InstantCommand(() -> shoot.moveHood(1)));
        // joystick.y().onFalse(new InstantCommand(() -> shoot.moveHood(0)));

        //joystick.x().whileTrue(new InstantCommand(() -> shoot.shoot(-1,1)));
        //joystick.x().whileFalse(new InstantCommand(() -> shoot.shoodt(0,0)));

        operator.leftTrigger().toggleOnTrue(new spinToWin(drivetrain, ()-> -operator.getLeftY() * MaxSpeed/5,()-> -operator.getLeftX() * MaxSpeed/5));
        
        //joystick.leftTrigger().onTrue(new alignmentMode(drivetrain, shoot));
        // joystick.a().onTrue(new InstantCommand(() -> shoot.changeHoodMode()));
        // joystick.b().onTrue(new InstantCommand(() -> shoot.hoodActivate()));

        // Reset the field-centric heading on left bumper press.
        operator.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);

        operator.rightTrigger().toggleOnTrue(new runShooter(shoot, operator, intake));
        // joystick.rightTrigger().onTrue(new InstantCommand(() -> shoot.shoot()));
        // joystick.rightTrigger().onFalse(new InstantCommand(() -> shoot.shoot()));

    }        // joystick.back().and(joystick.start()).onTrue(drivetrain.)


    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)

                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }

    public void putElastic(){
        SmartDashboard.putNumber("hood mode", shoot.getHoodMode());
        SmartDashboard.putNumber("analog 1", shoot.getVoltage1());
        SmartDashboard.putNumber("analog 2", shoot.getVoltage2());
        SmartDashboard.putNumber("targetRPM", shoot.getRPS());
        SmartDashboard.putNumber("distance", shoot.getDistance());
        SmartDashboard.putNumber("shoot speed", shoot.getSpeed());
    }

}
