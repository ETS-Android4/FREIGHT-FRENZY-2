/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.init_robot;
import org.firstinspires.ftc.teamcode.hardware.servo_brat;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste1;
import org.firstinspires.ftc.teamcode.hardware.servo_cleste2;

@Config
@TeleOp
//@Disabled
public class drive1 extends LinearOpMode {

    init_robot conserva = new init_robot();


    private double root2 = Math.sqrt(2.0);
    private boolean slow_mode = false;
    private boolean reverse_intake = false;

    public boolean ok = false;
    public boolean ok_intake = false;
    public boolean okk_intake = true;

    public static double outtake_velo = 2000;
    //public static double outtake_dist = 1950;
    public static double outtake_sus = 1200;
    public static double outtake_mijl = 760;
    public static double outtake_jos = 550;

    public static double viteza = 250;
    public static double a = 1;
    public static double cnt = 100;

    public static double down_pos = 5;
    public static double p = 2.5;
    public static double i = 1;
    public static double d = 0;
    public static double f = 13;
    public static double pp = 10;
    public DcMotorEx outtake = null;

    public double intake_speed = 0.58;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void runOpMode() {


        someRandomShit();
        Gamepad gp1 = gamepad1;
        Gamepad gp2 = gamepad2;

        servo_brat brat = new servo_brat(hardwareMap);
        servo_cleste1 cleste1 = new servo_cleste1(hardwareMap);
        servo_cleste2 cleste2 = new servo_cleste2(hardwareMap);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {

            /* gamepad 1 */

            /* Drive */

            double direction = Math.atan2(gp1.left_stick_x, gp1.left_stick_y) - Math.PI/2;
            double rotation = -gp1.right_stick_x;
            double speed = Math.sqrt(gp1.left_stick_x*gp1.left_stick_x + gp1.left_stick_y*gp1.left_stick_y);

            if(gp1.right_bumper){
                slow_mode = true;
            }
            if(gp1.left_bumper){
                slow_mode = false;
            }
            if(slow_mode){
                setDrivePowers(direction, 0.43 * Math.pow(speed, 3.0), 0.35 * Math.pow(rotation, 3.0));
            }else{
                setDrivePowers(direction, Math.pow(speed, 3.0),0.6 * Math.pow(rotation, 3.0));
            }

            if(gp2.dpad_up){
                outtake.setTargetPosition((int)outtake_sus);
                outtake.setVelocity(outtake_velo);
                sleep(550);
                brat.sus();
            }
            if(gp2.dpad_right){
                outtake.setTargetPosition((int)outtake_mijl);
                outtake.setVelocity(outtake_velo);
                sleep(550);
                brat.second();
            }
            if(gp2.dpad_down){
                outtake.setTargetPosition((int)outtake_jos);
                outtake.setVelocity(outtake_velo);
                sleep(550);
                brat.first();
            }
            if(gp2.dpad_left){
                outtake.setTargetPosition(950);
                outtake.setVelocity(outtake_velo);
                sleep(400);
                ok = false;
                brat.jos();
                cleste1.close();
                cleste2.close();
                sleep(700);
                outtake.setTargetPosition(150);
            }

            if(gp2.x)
            {
                cleste1.open();
                cleste2.open();
            }


            if(gp2.left_trigger > 0.1 && gp2.right_trigger > 0.1){
                cleste1.close();
                cleste2.open();
                outtake.setTargetPosition((int)down_pos);
                conserva.intake1.setVelocity(-1500*Math.min(gp1.right_trigger, intake_speed));

                ok_intake = true;
            }
            else if(gp2.right_trigger > 0.1) {
                cleste1.close();
                cleste2.open();
                outtake.setTargetPosition((int)down_pos);
                outtake.setVelocity(outtake_velo);
                conserva.intake1.setVelocity(1500*Math.min(gp2.right_trigger, intake_speed));

                ok_intake = true;
            }
            else if(ok_intake){
                cleste1.close();
                cleste2.close();
                conserva.intake1.setVelocity(0);
                sleep(350);
                outtake.setTargetPosition(200);
                outtake.setVelocity(outtake_velo);
                ok_intake = false;
            }

            if(gp1.b){
                conserva.intake1.setVelocity(-440);
                conserva.intake2.setVelocity(440);
                sleep(1430);
                conserva.intake1.setVelocity(-1400);
                conserva.intake2.setVelocity(1400);
                sleep(250);
                conserva.intake1.setVelocity(0);
                conserva.intake2.setVelocity(0);
            }
            if(gp1.x){
                viteza = 250;
                a = 1;
                while(a<350){
                    conserva.intake1.setVelocity(-1*(int)(viteza+(a*a/cnt)));
                    conserva.intake2.setVelocity((int)(viteza+(a*a/cnt)));
                    a += 1;
                }
                conserva.intake1.setVelocity(0);
                conserva.intake2.setVelocity(0);
            }


            /* Telemetry */
            telemetry.addData("slow_mode", slow_mode);
            telemetry.update();
        }

    }


    /* Mecanum drive */
    public void setDrivePowers(double direction, double speed, double rotateSpeed){
        double directionRads = direction;

        double sin = Math.sin(Math.PI/4 - directionRads);
        double cos = Math.cos(Math.PI/4 - directionRads);

        conserva.lf.setPower(-root2 * speed * sin + rotateSpeed);
        conserva.rf.setPower(-root2 * speed * cos - rotateSpeed);
        conserva.lr.setPower(-root2 * speed * cos + rotateSpeed);
        conserva.rr.setPower(-root2 * speed * sin - rotateSpeed);
    }

    public void someRandomShit(){

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setVelocityPIDFCoefficients(p, i, d, f);
        outtake.setPositionPIDFCoefficients(pp);
        outtake.setTargetPosition(5);
        outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake.setPower(0.0);
        outtake.setTargetPositionTolerance(2);

        conserva.init(hardwareMap);
    }


}