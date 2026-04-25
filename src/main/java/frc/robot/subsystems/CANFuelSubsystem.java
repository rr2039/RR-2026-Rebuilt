// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final SparkClosedLoopController launcherCLController;
  private final SparkClosedLoopController feederCLController;

  /** Creates a new CANBallSubsystem. */
  @SuppressWarnings("removal")
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederConfig.closedLoop
    .p(0.3)
    .i(0)
    .d(0.1)
    .outputRange(-0.8, 0.8)
    .feedForward
    .kV(12.0 / 5767);;
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feederCLController = feederRoller.getClosedLoopController();


    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherConfig.closedLoop
    .p(0.3)
    .i(0)
    .d(0.1)
    .outputRange(-1, 1)
    .feedForward
    .kV(12.0 / 5767);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherCLController = intakeLauncherRoller.getClosedLoopController();

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double vel) {
    launcherCLController.setSetpoint(vel, ControlType.kVelocity);
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double vel) {
    feederCLController.setSetpoint(vel, ControlType.kVelocity);

  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    feederCLController.setSetpoint(0, ControlType.kVelocity);
    intakeLauncherRoller.set(0);
    launcherCLController.setSetpoint(0, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
