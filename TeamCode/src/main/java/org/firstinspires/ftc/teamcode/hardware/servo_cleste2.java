package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_cleste2 {
    public Servo servo = null;

    public static double POS_DESC = 0.8;
    public static double POS_INCH = 0.5;

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
    public void close() { setServoPositions(POS_INCH);}

}