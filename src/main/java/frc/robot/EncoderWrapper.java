package frc.robot;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class EncoderWrapper {

    BaseMotorController motorController;
    private int oldTicks;
    private int currentTicks;
    private double oldTime = 0;
    private double time = 0;
    private Double distancePerPulse;

    public EncoderWrapper(BaseMotorController motorController) {
        this.motorController = motorController;
    }

    public void setDistancePerPulse (double distancePerPulse) {
        this.distancePerPulse = distancePerPulse;
    }

    public int getRawDistance() {
        return motorController.getSelectedSensorPosition();
    }

    public double getDistance() {
        if (distancePerPulse == null) throw new NullPointerException("Distance Per Pulse");
        return motorController.getSelectedSensorPosition() * distancePerPulse;
    }

    public double getRate () {
        if (distancePerPulse == null) throw new NullPointerException("Distance Per Pulse");
        return (distancePerPulse * (currentTicks - oldTicks)) / (time - oldTime);
    }

    public void reset () {
        motorController.setSelectedSensorPosition(0);
    }

    public void update() {
        oldTime = time;
        time = (double)System.nanoTime() / 1_000_000_000.0;
        oldTicks = currentTicks;
        currentTicks = getRawDistance();
    }
}