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

package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class init_robot {
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor lf = null; // Left front motor
    public DcMotor rf = null; // Right front motor
    public DcMotor lr = null; // Left rear motor
    public DcMotor rr = null; // Right rear motor
    public DcMotorEx intake1 = null; // Intake motor
    public DcMotorEx intake2 = null; // Intake motor
    public DcMotorEx carusel = null; // Intake motor

    HardwareMap hwMap = null;

    public init_robot() {}

    public void init(HardwareMap param_hwMap) {
        hwMap = param_hwMap;


        /* Movement motors */
        lf = hwMap.get(DcMotor.class, "leftFront");
        lr = hwMap.get(DcMotor.class, "leftRear");
        rf = hwMap.get(DcMotor.class, "rightFront");
        rr = hwMap.get(DcMotor.class, "rightRear");

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setDirection(DcMotor.Direction.REVERSE);

        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setDirection(DcMotor.Direction.FORWARD);

        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setDirection(DcMotor.Direction.REVERSE);

        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setDirection(DcMotor.Direction.FORWARD);

        lf.setPower(0.0);
        rf.setPower(0.0);
        lf.setPower(0.0);
        lr.setPower(0.0);



        /* Intake motor 1 */
        intake1 = hwMap.get(DcMotorEx.class, "intake1");

        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake1.setPower(0.0);


        /* Intake motor 2 */
        intake2 = hwMap.get(DcMotorEx.class, "intake2");

        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setDirection(DcMotor.Direction.REVERSE);
        intake2.setPower(0.0);


        carusel = hwMap.get(DcMotorEx.class, "carusel");

        carusel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carusel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carusel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carusel.setDirection(DcMotor.Direction.REVERSE);
        carusel.setPower(0.0);



    }

}
