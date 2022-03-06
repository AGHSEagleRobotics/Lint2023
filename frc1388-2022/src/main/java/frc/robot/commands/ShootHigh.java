// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem.FeederFunctions;

//TODO can make high and low one command
public class ShootHigh extends CommandBase {
  private final ShooterFeederSubsystem m_shooterSubsystem;
  private final TransitionSubsystem m_transitionSubsystem;
  /** Creates a new ShooterCommands. */
  public ShootHigh(ShooterFeederSubsystem shooterSubsystem, TransitionSubsystem transitionSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_transitionSubsystem = transitionSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, transitionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setShooterEnabled(true);
    m_shooterSubsystem.setTargetRPM(ShooterConstants.SHOOTER_RPM_HIGHGOAL);

  }

  // For shooterSpeedIsReady, we need to start feeder when shooter speed is stable for around 1/5 of a second:

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // check with subystem to see if shooter ready
    if (m_shooterSubsystem.shooterSpeedIsReady()) {
      //run feeder motor
      m_shooterSubsystem.setFeederFunction(FeederFunctions.FORWARD);
      m_transitionSubsystem.setTransitionSpeed(TransitionConstants.TRANSITION_SPEED_FORWARD_FAST);
    } else {
      m_shooterSubsystem.setFeederFunction(FeederFunctions.OFF);
      m_transitionSubsystem.setTransitionSpeed(TransitionConstants.TRANSITION_SPEED_FORWARD_SLOW);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Turn off feeder motor
    m_shooterSubsystem.setFeederFunction(FeederFunctions.OFF);
    //Turn off shooter motor
    m_shooterSubsystem.setShooterEnabled(false);
    m_transitionSubsystem.setTransitionSpeed(TransitionConstants.TRANSITION_SPEED_FORWARD_SLOW);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}