package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_test {
    public Servo servo = null;

    public static double SERVO_RELEASEE = 0.66;
    public static double SERVO_POZ1 = 0.59;
    public static double SERVO_POZ2 = 0.50;
    public static double SERVO_CLOSEE = 0.39;

    public servo_test(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "servoTest");
        close();
    }

    public void setServoPositions(double pos1) {
        if(pos1 > -1.0) {
            servo.setPosition(pos1);
        }
    }

    public void open() { setServoPositions(SERVO_RELEASEE); }
    public void cerc1(){ setServoPositions(SERVO_POZ1);}
    public void cerc2(){ setServoPositions(SERVO_POZ2);}
    public void close() {
        setServoPositions(SERVO_CLOSEE);
    }
}