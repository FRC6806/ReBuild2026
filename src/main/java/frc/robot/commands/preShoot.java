// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class preShoot extends Command {
  /** Creates a new shootFeed. */
  Shooter shooter1;
  CommandXboxController joystick;
  public preShoot(Shooter s) {
    shooter1=s;
    joystick = new CommandXboxController(0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter1.pSetSpeed(.75);
    shooter1.fSetSpeed(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (joystick.rightTrigger().getAsBoolean()){
      shooter1.sSetSpeed(0);
      shooter1.fSetSpeed(0);
      shooter1.pSetSpeed(0);
      return true;
    }else{
      return false;
    }
  }
}
