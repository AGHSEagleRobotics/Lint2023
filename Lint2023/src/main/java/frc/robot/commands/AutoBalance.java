// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoBalance extends CommandBase {
  private static final Logger log = LogManager.getLogger(AutoBalance.class);

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private final double m_turnSpeed;
  private final double m_turnAngleSet;

  private double m_angle1 = 0;
  private double m_angle2 = 0;
  private double m_angle3 = 0;
  private int m_tickCounter = 0;

  private final PIDController m_pidController = new PIDController(DriveTrainConstants.BALANCE_P_VALUE, 0, 0);

  /** Creates a new AutoTurn. */
  public AutoBalance(DriveTrainSubsystem driveTrainSubsystem, double turnSpeed, double turnAngleSet) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_turnSpeed = turnSpeed;
    m_turnAngleSet = turnAngleSet;
    //System.out.println("*****************TURNCONSTUCTOR****************************************TURNCONSTRUCTOR*******************");
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubsystem);

    m_pidController.setTolerance(DriveTrainConstants.BALANCE_P_VALUE);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.info("m_turnSpeed={}\tm_turnAngleSet={}",m_turnSpeed,m_turnAngleSet);

    // m_driveTrainSubsystem.resetGyro();
    m_driveTrainSubsystem.setDeadbandZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //  double turnSpeed;
    m_tickCounter++;

    double pSpeed;
    double angle = m_driveTrainSubsystem.getGyroAngle();
    double averageAngle = Math.abs((m_angle1 + m_angle2 + m_angle3) / 3);
    System.out.println("Angle1: "+ m_angle1 + "   Angle2: " + m_angle2 + "   Angle3: " + m_angle3);
    System.out.println("Average Angle: "+ averageAngle);

    if( averageAngle - Math.abs(angle) > 1){
    //if( Math.abs(angle) < 13.5 ){
      pSpeed = 0;
    }
    else {
          //double pSpeed = angle * DriveTrainConstants.BALANCE_P_VALUE;
      pSpeed = Math.pow(((Math.abs(angle))/15), 3);
      System.out.println("(abs(angle)/15)^7 :  " + pSpeed);
      pSpeed = Math.copySign(pSpeed, angle);
      System.out.println("pSpeed copySign :  " + pSpeed);
      pSpeed = MathUtil.clamp(pSpeed, -0.45, 0.45);
      System.out.println("pSpeed clamped :  " + pSpeed);
      m_driveTrainSubsystem.tankDrive(pSpeed, pSpeed);
    
    }
    
    /* TODO test constant code
    double constantSpeed = DriveTrainConstants.BALANCE_CONSTANT;
    m_driveTrainSubsystem.tankDrive(constantSpeed, constantSpeed);
    */
    if( m_tickCounter == 1){ m_angle1 = angle; }
    if( m_tickCounter == 2){ m_angle2 = angle; }
    if( m_tickCounter == 3){ m_angle3 = angle; m_tickCounter=0; }

    System.out.println("Angle: "+angle + "/t Speed: " + pSpeed);

    // turnSpeed = m_pidController.calculate(angle, m_turnAngleSet);
   // turnSpeed = MathUtil.clamp(turnSpeed, -m_turnSpeed, m_turnSpeed);

    // log.debug("Angle: {} \tturnSpeed: {} \tTurnSetPoint: {}", angle, turnSpeed, m_turnAngleSet);
    //System.out.println("Angle: "+angle+"\tturnSpeed: "+turnSpeed+"\tTurnSetPoint"+m_turnAngleSet);

   // m_driveTrainSubsystem.curvatureDrive(0, turnSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pidController.reset();
    m_driveTrainSubsystem.curvatureDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = (m_pidController.atSetpoint());
    // boolean finished = (m_pidController.atSetpoint());
    // return finished;
    return false;
  }
}
