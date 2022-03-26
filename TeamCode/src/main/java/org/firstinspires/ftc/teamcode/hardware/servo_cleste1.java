package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_cleste1 {
    public Servo servo = null;

    public static double POS_DESC = 0.71;
    public static double POS_HUB = 0.42;
    public static double POS_SEMI = 0.37;
    public static double POS_INCH = 0.33;

    public servo_cleste1(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "servoCleste1");
        close();
    }

    public void setServoPositions(double pos1) {
        if(pos1 > -1.0) {
            servo.setPosition(pos1);
        }
    }

    public void open() { setServoPositions(POS_DESC);}
    public void semi() { setServoPositions(POS_SEMI);}
    public void hub() { setServoPositions(POS_HUB);}
    public void close() { setServoPositions(POS_INCH);}

}