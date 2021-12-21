package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_cutie{
    public Servo servo = null;

    public static double SERVO_RELEASE = 0.93;
    public static double SERVO_CLOSE = 0.1;

    public servo_cutie(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "servoCutie");
        drept();
    }

    public void setServoPositions(double pos1) {
        if(pos1 > -1.0) {
            servo.setPosition(pos1);
        }
    }

    public void drept() { setServoPositions(SERVO_RELEASE);}
    public void unghi() {
        setServoPositions(SERVO_CLOSE);
    }
}