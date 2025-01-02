package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CalibrateServo {
    public Servo servo;
    public double servoPos, increment;
    Telemetry tele;
    public CalibrateServo(Servo servo, double increment) {
        this.servo = servo;
        this.increment = increment;
    }

    public CalibrateServo(Servo servo) {
        this.servo = servo;
        this.increment = 0.03;
    }

    public double calibrate(boolean up, boolean down) {
        servoPos = servo.getPosition();
        if (up) servo.setPosition(servoPos + increment);
        if (down) servo.setPosition(servoPos - increment);
        return servo.getPosition();
    }
}