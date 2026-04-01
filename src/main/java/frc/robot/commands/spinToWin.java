package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class spinToWin extends Command {
  // private final CommandXboxController joystick = new CommandXboxController(0);
  private CommandSwerveDrivetrain swerve;
  private Shooter shoot;
  private Telemetry telemetry;
  private double k;
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
  private DoubleSupplier xVelocity, yVelocity;
  // --- Motion Compensation Constants ---
private static final double kP_TURN = 0.25;
private static final double kI_TURN = 0.0;
private static final double kD_TURN = 0.001;

private static final double kV_Y = 0.05;   // lateral velocity compensation
private static final double kW_OMEGA = 0.02; // rotational velocity compensation

private final PIDController turnPID = new PIDController(kP_TURN, kI_TURN, kD_TURN);

  /** Creates a new command. */
  public spinToWin(CommandSwerveDrivetrain s, DoubleSupplier xVelocity, DoubleSupplier yVelocity, Telemetry t) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = s;
    telemetry = t;
    this.xVelocity = xVelocity;
    this.yVelocity = yVelocity;

    addRequirements(s);
  }
  public double rotate(){
    if (Math.abs(LimelightHelpers.getTX("limelight-bigboy"))>2 ){
      return -(Math.copySign(1.0, LimelightHelpers.getTX("limelight-bigboy")));
    }else{
      return 0;
    }
    
    // double value = -LimelightHelpers.getTX("limelight-bigboy");
    // if(Math.abs(value) < 0.5) value = 0;
    // return value;
  }
  

public double rotate2() {
    double tx = LimelightHelpers.getTX("limelight-bigboy");

    // If no tag, don't rotate
    if (!LimelightHelpers.getTV("limelight-bigboy")) {
        return 0;
    }
    swerve.getPigeon2().getYaw().getValue();

    // Continuous smooth rotation
    double output = turnPID.calculate(tx, 0);

    // Clamp to reasonable speed
    return MathUtil.clamp(output, -1.0, 1.0);
}

public double rotate3() {
    // 1. Read Limelight TX
    double tx = LimelightHelpers.getTX("limelight-bigboy");

    // 2. Get the current swerve drive state
    SwerveDriveState state = swerve.getState(); // your drivetrain already exposes this

    // 3. Compute lateral velocity compensation
    double yVelRPS = telemetry.getYVelocityMotorRPS(state);
    double lateralComp = kV_Y * yVelRPS;

    // 4. Compute rotational velocity compensation
    double omega = telemetry.getOmegaRadPerSec(state);
    double omegaComp = kW_OMEGA * omega;

    // 5. Base rotation from TX PID (target = 0 because shooter + goal centered)
    double pidRotation = turnPID.calculate(tx, 0.0);

    // 6. Final rotation command
    double rotation = pidRotation + lateralComp + omegaComp;

    // Optional clamp
    return MathUtil.clamp(rotation, -1.0, 1.0);
}



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight-bigboy",0);
    //new shootSys();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = xVelocity.getAsDouble();
    double y = yVelocity.getAsDouble();
    if(Math.abs(x) < .09) x = 0;
    if(Math.abs(y) < .09) y = 0;

    swerve.setControl(drive.withVelocityX(x).withVelocityY(y).withRotationalRate(rotate2()));
    }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
