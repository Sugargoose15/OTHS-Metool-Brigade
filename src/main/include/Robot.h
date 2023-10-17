// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <string>
#include "ctre/Phoenix.h"
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc2/command/PIDCommand.h>
#include <frc/Timer.h>

#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>

#include <frc/motorcontrol/VictorSP.h>
#include <frc/motorcontrol/Talon.h>
#include <CANVenom.h>
#include <frc/motorcontrol/MotorControllerGroup.h>

#include <frc/Encoder.h>
#include <frc/AnalogEncoder.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/smartdashboard/SendableChooser.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
  bool debounce;

  	struct Controls
	{
		double x;
		double y;
		bool trigger;
		double stick_left;
		double stick_right;
		double trigger_left;
		double trigger_right;
		bool bumper_right;
		bool align;
		bool wave;
	};
    Controls PowerLevel = Controls{
		.x = 0.7,
		.y = 0.7,
		.trigger = false,
		.stick_left = -0.3,
		.stick_right = -0.3,
		.trigger_left = 1,
		.trigger_right = -1,
		.bumper_right = false,
		.align = false,
		.wave = false};

  Controls GetControls()
	{
		return Controls{
			.x = joystick.GetRawAxis(4) * PowerLevel.x,
			.y = joystick.GetRawAxis(1) * PowerLevel.y,
			.trigger = joystick.GetRawButton(1),
			.stick_left = controller.GetRawAxis(1) * PowerLevel.stick_left,
			.stick_right = controller.GetRawAxis(5) * PowerLevel.stick_right,
			.trigger_left = controller.GetRawAxis(2) * PowerLevel.trigger_left,
			.trigger_right = controller.GetRawAxis(3) * PowerLevel.trigger_right,
			.bumper_right = controller.GetRawButton(6),
			.wave = controller.GetRawButton(0)};
	}

	frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};

  	TalonSRX motor_right_front_CTRE{18};
	TalonSRX motor_right_back_CTRE{14};
	TalonSRX motor_left_back_CTRE{16};
	TalonSRX motor_left_front_CTRE{15};

	WPI_TalonSRX motor_right_front{18};
	WPI_TalonSRX motor_right_back{14};
	WPI_TalonSRX motor_left_back{16};
	WPI_TalonSRX motor_left_front{15};

  	frc::MotorControllerGroup group_right{motor_right_back, motor_right_front};
	frc::MotorControllerGroup group_left{motor_left_back, motor_left_front};
	frc::DifferentialDrive drive{group_left, group_right};

 	frc::CANVenom motor_spool{2};
	frc::CANVenom motor_arm{1};

  	frc::Joystick controller{0};
	frc::Joystick joystick{1};

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  frc::Timer m_timer;
};
