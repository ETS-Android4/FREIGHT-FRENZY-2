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

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private double root2 = Math.sqrt(2.0);
    private boolean slow_mode = false;
    private boolean reverse_intake = false;

    public boolean ok = false;
    public boolean ok_intake = false;

    public static double outtake_velo = 2000;
    public static double outtake_dist = 1950;
    public static double down_pos = 5;
    public static double p = 2.5;
    public static double i = 1;
    public static double d = 0;
    public static double f = 13;
    public static double pp = 10;
    public DcMotorEx outtake = null;

    public double intake_speed = 0.5;



    @Override
    public void runOpMode() {

        someRandomShit();
        Gamepad gp1 = gamepad1;
        //Gamepad gp2 = gamepad2;

        servo_brat brat = new servo_brat(hardwareMap);
        servo_cleste1 cleste1 = new servo_cleste1(hardwareMap);
        servo_cleste2 cleste2 = new servo_cleste2(hardwareMap);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {

            /* gamepad 1 */

            /* Drive */
			
            double direction = Math.atan2(-gp1.left_stick_y, gp1.left_stick_x) - Math.PI/2;
            double rotation = -gp1.right_stick_x;
            double speed = Math.sqrt(gp1.left_stick_x*gp1.left_stick_x + gp1.left_stick_y*gp1.left_stick_y);

            if(gp1.dpad_down){
                slow_mode = true;
            }
            if(gp1.dpad_up){
                slow_mode = false;
            }
            if(slow_mode){
                setDrivePowers(direction, 0.38 * Math.pow(speed, 3.0), 0.35 * Math.pow(rotation, 3.0));
            }else{
                setDrivePowers(direction, Math.pow(speed, 3.0),0.7 * Math.pow(rotation, 3.0));
            }

            if(gp1.right_bumper){
                outtake.setTargetPosition((int)outtake_dist);
                outtake.setVelocity(outtake_velo);
            }
            if(gp1.left_bumper){
                ok = false;
                brat.jos();
                cleste1.close();
                cleste2.close();
                sleep(1000);
                outtake.setTargetPosition(200);
            }

            if(gp1.a && outtake.getCurrentPosition() > 1000 && !ok)
            {
                ok = true;
                brat.sus();
            }
            if(gp1.b)
            {
                cleste1.open();
                cleste2.open();
            }



            if(gp1.right_trigger > 0.1) {
                cleste1.open();
                cleste2.close();
                outtake.setTargetPosition((int)down_pos);
                conserva.intake1.setVelocity(1500*Math.min(gp1.right_trigger, intake_speed));

                //conserva.intake2.setVelocity(1500*Math.min(gp1.right_trigger, intake_speed));
                ok_intake = true;
            }
            else if(ok_intake){
                cleste1.close();
                outtake.setTargetPosition(200);
                conserva.intake1.setVelocity(-400);

                //conserva.intake2.setVelocity(-400);
                ok_intake = false;
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