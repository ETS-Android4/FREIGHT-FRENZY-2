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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
//@Disabled
public class intake_test extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public DcMotorEx intake = null;
    public double motor_ticks = 103.8;
    public double ticks = 0;
    public double ticks2 = 0;
    public static double viteza = 0.5;

    @Override
    public void runOpMode() {

        /* Carousel PID */


        intake = hardwareMap.get(DcMotorEx.class, "intake1");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setPower(0.0);


        /* Gamepads */

        Gamepad gp2 = gamepad2;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {
            /* gamepad 2 */

            /* Intake */


            /*
            if(gp2.right_trigger > 0.1) {
                intake.setVelocity(gp2.right_trigger*1000);
           }
            else{
                ticks = intake.getCurrentPosition();
                ticks = ticks % motor_ticks;
                if(ticks<1.5 || ticks > (motor_ticks-3.5))
                    intake.setPower(0.0);
                else
                    intake.setPower(0.035);
            }

             */



            if(gp2.right_trigger > 0.1) {
                intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intake.setVelocity(1500*Math.min(gp2.right_trigger, viteza));
            }
            else if (intake.getCurrentPosition() % motor_ticks > 3 && intake.getCurrentPosition() % motor_ticks < motor_ticks-3){
                ticks = intake.getCurrentPosition();
                ticks2 = ticks % motor_ticks;
                intake.setTargetPosition((int)(ticks + (motor_ticks-ticks2)));
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setVelocity(200);
            }

            dashboardTelemetry.addData("position", intake.getCurrentPosition());
            dashboardTelemetry.update();

        }

    }
}