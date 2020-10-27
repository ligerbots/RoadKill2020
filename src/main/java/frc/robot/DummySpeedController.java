package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;

public class DummySpeedController implements SpeedController{
  double s;
  @Override
  public void pidWrite(double output) {

  }

  @Override
  public void set(double speed) {
    s=speed;
  }

  @Override
  public double get() {
    return s;
  }

  @Override
  public void setInverted(boolean isInverted) {

  }

  @Override
  public boolean getInverted() {
    return false;
  }

  @Override
  public void disable() {

  }

  @Override
  public void stopMotor() {

  }
}