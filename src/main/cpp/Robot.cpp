// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "ctre/Phoenix.h"
#include <CANVenom.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "cameraserver/CameraServer.h"
#include <frc/drive/DifferentialDrive.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/Encoder.h>
#include <frc/TimedRobot.h>
#include <cmath>
#include <fmt/core.h>
#include <frc/Timer.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::CameraServer::StartAutomaticCapture();

}

void Robot::RobotPeriodic() {}

 
void Robot::AutonomousInit() {
  m_timer.Restart();
  
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
    
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
    double autospeed = 0.35;
    if (m_timer.Get() < 1_s) {
      drive.ArcadeDrive(0.05, autospeed, false);
    } else if (m_timer.Get() < 2_s) {
      drive.ArcadeDrive(autospeed, 0.0, false);
    } else {
      drive.ArcadeDrive(0.0, 0.0, false);
    }
    
  }
}

void Robot::TeleopInit() {
bool debounce = false;
}

void Robot::TeleopPeriodic() {
  // set encoder position
  motor_arm.SetPosition(0);
  // Fetch Controls
  Robot::Controls controls = Robot::GetControls();
  
  //Drive Code
  drive.ArcadeDrive(controls.x, controls.y, true);
  //move arm
  motor_arm.SetCommand(frc::CANVenom::kProportional,controls.stick_left);
  //wave arm (A)
  double wavespeed = .5;
  double waverange = 1;
  frc::SmartDashboard::PutNumber("armposition", motor_arm.GetPosition());
  if (controls.wave == true) {
    motor_arm.SetPosition(0);
    if (motor_arm.GetPosition() == waverange) {
      motor_arm.SetCommand(frc::CANVenom::kProportional,wavespeed);
    }
  }
  

  double parameter = 0.05; //raise value to stop dancing
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double tx = table->GetNumber("tx", 0.0);
  double ta = table->GetNumber("ta", 0.0);
  frc::SmartDashboard::PutNumber("ta", ta);
  //double bonus = (ta>3)?0:(-(2/15)*ta + 0.4); <- thanks Lukas
  // Align to target (april tag)
  double align = joystick.GetRawAxis(3);
  double hk = align -.2;
  double sadmaxpower = hk/1.6; //maximum power on motors during seek and destroy 
  double bonus = tx/70; //turning speed for s&d
  double seekturn = .2;
  if (bonus > sadmaxpower) {
    bonus = sadmaxpower;
  } else if (bonus < -sadmaxpower) {
    bonus = -sadmaxpower;
  } //clamps bonus ^
  double fwdbonus = sadmaxpower - abs(bonus);//puts rest of max power to forward movement
   //right trigger
   
  if (align > 0.5) {
    if ((ta == 0)&&(debounce == false)) {
    drive.ArcadeDrive(seekturn, 0, false);
    }
    else if ((ta < 3)&&(ta != 0)) { // stops when close
      drive.ArcadeDrive(bonus, -fwdbonus, false);
      debounce = true;
    }
  } else {
    debounce = false;
  }
  frc::SmartDashboard::PutNumber("debounce", debounce);
  

  // Fetch Controls
  
  //Drive Code
    //drive.ArcadeDrive(controls.x, controls.y, true);

  /*if (controls.stick_right > 0.2){
    motor_spool.SetCommand(frc::CANVenom::kProportional, 0);
    } 
    else {
      motor_spool.SetCommand(frc::CANVenom::kProportional,controls.stick_right);

    }
   // if (motor_arm.GetPosition()>-7.5){
   // motor_arm.SetCommand(frc::CANVenom::kProportional, controls.stick_right);
   // } 
   // else if (motor_arm.GetPosition() < -7.5){
   //   motor_arm.SetCommand(frc::CANVenom::kProportional,controls.stick_right * -0.25);

}*/
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
