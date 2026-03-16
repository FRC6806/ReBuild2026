package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
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
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
  private DoubleSupplier xVelocity, yVelocity;
  /** Creates a new command. */
  public spinToWin(CommandSwerveDrivetrain s, DoubleSupplier xVelocity, DoubleSupplier yVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = s;
    this.xVelocity = xVelocity;
    this.yVelocity = yVelocity;

    addRequirements(s);
  }
  public double rotate(){
    if (Math.abs(LimelightHelpers.getTX("limelight-bigboy"))>2 && (Math.abs(xVelocity.getAsDouble()) > 0.1 || Math.abs(yVelocity.getAsDouble()) > 0.1)){
      return -(Math.copySign(1.0, LimelightHelpers.getTX("limelight-bigboy")));
    }else{
      return 0;
    }
    // double value = -LimelightHelpers.getTX("limelight-bigboy");
    // if(Math.abs(value) < 0.5) value = 0;
    // return value;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight-bigboy",1);
    //new shootSys();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = xVelocity.getAsDouble();
    double y = yVelocity.getAsDouble();
    if(Math.abs(x) < .09) x = 0;
    if(Math.abs(y) < .09) y = 0;

    swerve.setControl(drive.withVelocityX(x).withVelocityY(y).withRotationalRate(rotate()));
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
