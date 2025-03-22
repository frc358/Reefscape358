// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.time.format.TextStyle;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.NetworkTableInstance;


public class Robot extends TimedRobot {


  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer; 
  public static boolean obtainedAlliance = false;
  public boolean m_LimelightHasValidTarget = false;
  public static double tx;
  public static double ty;

  public Robot() {
    m_robotContainer = new RobotContainer();
    //CameraServer.startAutomaticCapture();
    SignalLogger.enableAutoLogging(false);
  }

  @Override
  public void robotPeriodic() {

    if (!obtainedAlliance && DriverStation.isDSAttached() && DriverStation.isFMSAttached() && DriverStation.getAlliance().isPresent()){
      obtainedAlliance = true;
    }
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    //update boolean value before putting it on dashboard
    m_LimelightHasValidTarget = LimelightHelpers.getTV("limelight"); 
    SmartDashboard.putBoolean("AprilTag Target", m_LimelightHasValidTarget);
    tx = LimelightHelpers.getTX("limelight");
    ty = LimelightHelpers.getTY("limelight");
    //SmartDashboard.putNumber("April Tag ID", NetworkTableInstance.getDefault().getTable("limelight").getEntry("<tid>").getDouble(kDefaultPeriod));
  }

  @Override
  public void disabledInit() {
    System.gc();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}


  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

public static boolean isRedAlliance(){
  if (RobotBase.isReal()){
    return DriverStation.getAlliance().filter(value -> value == DriverStation.Alliance.Red).isPresent();
  }
  else return false; //False = blue, true = red
}

}