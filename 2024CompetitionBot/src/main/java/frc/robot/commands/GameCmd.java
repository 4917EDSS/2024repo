// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Robot;
import frc.robot.subsystems.ArduinoSub;

public class GameCmd extends Command {
  private final CommandPS4Controller m_driverController;
  private final ArduinoSub m_arduinoSub;

  private Instant time;

  private int length = 1;
  private int xWorm[] = new int[24];
  private int yWorm[] = new int[24];
  private int direction = 0;

  private int foodX = 0;
  private int foodY = 0;

  public GameCmd(CommandPS4Controller controller, ArduinoSub arduinoSub) {
    m_driverController = controller;
    m_arduinoSub = arduinoSub;

    addRequirements(arduinoSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initWorm();
    length = 1;
    time = Instant.now();
    foodX = (int) (Math.random() * 3.0);
    foodY = (int) (Math.random() * 5.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.inTestMode) { // Only run this in test mode
      //m_arduinoSub.clearPixels();
      if(Duration.between(time, Instant.now()).toMillis() > 200) {
        getInput();
        m_arduinoSub.clearPixels();
        updateWorm();
        drawFood();
        drawWorm();
        checkFood();
        m_arduinoSub.updatePixels();

        time = Instant.now();
      }
    }
  }

  void initWorm() {
    for(int i = 1; i < 24; i++) {
      xWorm[i] = -1;
      yWorm[i] = -1;
    }
  }

  void updateWorm() {
    switch(direction) {
      case 0:
        xWorm[0]++;
        break;

      case 1:
        yWorm[0]++;
        break;

      case 2:
        xWorm[0]--;
        break;
      case 3:
        yWorm[0]--;
        break;
    }
    if(xWorm[0] < 0)
      xWorm[0] = 3;
    if(xWorm[0] > 3)
      xWorm[0] = 0;

    if(yWorm[0] < 0)
      yWorm[0] = 5;
    if(yWorm[0] > 5)
      yWorm[0] = 0;
    for(int i = length - 1; i > 0; i--) {
      xWorm[i] = xWorm[i - 1];
      yWorm[i] = yWorm[i - 1];
    }
  }

  boolean checkDead() {
    for(int i = 2; i < length; i++) {
      if(xWorm[i] == xWorm[0] && yWorm[i] == yWorm[0])
        return true;
    }
    return false;
  }

  void drawWorm() {

    for(int i = 0; i < length; i++) {
      if(i < 2) {
        m_arduinoSub.writePixel(xWorm[i], yWorm[i], 0, 255, 255);
      } else {
        m_arduinoSub.writePixel(xWorm[i], yWorm[i], 0, 255, 0);
      }

    }
  }

  void drawFood() {
    m_arduinoSub.writePixel(foodX, foodY, 255, 0, 0);
  }

  void checkFood() {
    if(xWorm[0] == foodX && yWorm[0] == foodY) {
      length++;
      foodX = (int) (Math.random() * 3.0);
      foodY = (int) (Math.random() * 5.0);
    }
  }

  void getInput() {
    if(m_driverController.povRight().getAsBoolean() && direction != 2) {
      direction = 0;
    } else if(m_driverController.povDown().getAsBoolean() && direction != 3) {
      direction = 1;
    } else if(m_driverController.povLeft().getAsBoolean() && direction != 0) {
      direction = 2;
    } else if(m_driverController.povUp().getAsBoolean() && direction != 1) {
      direction = 3;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arduinoSub.updateLED(0, 255, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!Robot.inTestMode) // Only run this in test mode
      return true;
    return m_driverController.cross().getAsBoolean() || checkDead();
  }
}
