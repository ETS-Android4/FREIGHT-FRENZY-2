package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_cutie {
    public Servo servo = null;

    public static double SERVO_RELEASEE = 0.8;
    public static double SERVO_CLOSEE = 0.4;

    public servo_cutie(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "servoCutie");
        jos();
    }

    public void setServoPositions(double pos1) {
        if(pos1 > -1.0) {
            servo.setPosition(pos1);
        }
    }

    public void jos() { setServoPositions(SERVO_RELEASEE); }
    public void sus() {
        setServoPositions(SERVO_CLOSEE);
    }
}