package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_odo {
    public Servo servo = null;

    public static double POS_JOS = 0.82;
    public static double POS_SUS = 0.48;

    public servo_odo(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "servoOdometrie");
        jos();
    }

    public void setServoPositions(double pos1) {
        if(pos1 > -1.0) {
            servo.setPosition(pos1);
        }
    }

    public void jos() { setServoPositions(POS_JOS);}
    public void sus() { setServoPositions(POS_SUS);}

}