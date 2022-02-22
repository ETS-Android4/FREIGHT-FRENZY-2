package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_cleste2 {
    public Servo servo = null;

    public static double POS_DESC = 0.7;
    public static double POS_SEMI = 0.15;
    public static double POS_INCH = 0.1;

    public servo_cleste2(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "servoCleste2");
        close();
    }

    public void setServoPositions(double pos1) {
        if(pos1 > -1.0) {
            servo.setPosition(pos1);
        }
    }

    public void open() { setServoPositions(POS_DESC);}
    public void semi() { setServoPositions(POS_SEMI);}
    public void close() { setServoPositions(POS_INCH);}

}