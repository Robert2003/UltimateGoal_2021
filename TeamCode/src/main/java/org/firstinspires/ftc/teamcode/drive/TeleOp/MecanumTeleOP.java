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

package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="MecanumTeleOP", group="Iterative Opmode")
//@Disabled
public class MecanumTeleOP extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;

    private Servo servo;
    private Servo wobbleServo;

    private DcMotor intake1 = null;
    private DcMotor intake2 = null;
    private DcMotor flyWheel = null;
    private DcMotor wobbleArm = null;

    double rearLeftPower;
    double rearRightPower;
    double frontLeftPower;
    double frontRightPower;

    double flyWheelPower;

    double drive;
    double strafe;
    double rotate;

    double powerLatchingUp;
    double powerLatchingDown;

    boolean supress2 = false;

    private BNO055IMU imu;

    @Override
    public void init()
    {
        leftFront = hardwareMap.get(DcMotor.class, "fl_motor");
        rightFront = hardwareMap.get(DcMotor.class, "fr_motor");
        leftRear = hardwareMap.get(DcMotor.class, "bl_motor");
        rightRear = hardwareMap.get(DcMotor.class, "br_motor");

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        flyWheel = hardwareMap.get(DcMotor.class, "flyWheel");
        wobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");

        servo = hardwareMap.get(Servo.class, "servo");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        servo.setPosition(0.5);

        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotor.Direction.REVERSE);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    @Override
    public void init_loop()
    {
    }

    @Override
    public void start()
    {
        runtime.reset();
    }

    @Override
    public void loop()
    {
        /*Acceleration angle = imu.getGravity();*/
        telemetry.addData("x1", leftFront.getCurrentPosition());
        telemetry.addData("x2", rightFront.getCurrentPosition());
        telemetry.addData("x3", rightRear.getCurrentPosition());
        telemetry.addData("x4", leftRear.getCurrentPosition());
        telemetry.addData("wobble", wobbleArm.getCurrentPosition());
        telemetry.update();

        if (gamepad2.y) {
            intake1.setPower(1);
            intake2.setPower(1);
        } else if (gamepad2.a) {
            intake1.setPower(-1);
            intake2.setPower(-1);
        } else if (gamepad2.x) {
            intake1.setPower(0);
            intake2.setPower(0);
        }

        /*
        flyWheelPower = gamepad2.right_stick_y;
        if(Math.abs(flyWheel.getPower()-flyWheelPower) > 0.05)
            if(Math.abs(flyWheelPower) > 0.75)
                flyWheel.setPower(0.75);
            else
                flyWheel.setPower(flyWheelPower);
*/
        powerLatchingUp = gamepad2.left_trigger;
        powerLatchingDown = -gamepad2.right_trigger;

        if((powerLatchingUp==0 && powerLatchingDown==0))
        {
            wobbleArm.setTargetPosition(wobbleArm.getCurrentPosition());
            wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleArm.setPower(1);
        }
        else
            wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(powerLatchingDown!=0 || powerLatchingUp!=0)
            wobbleArm.setPower(powerLatchingUp+powerLatchingDown);

        if(gamepad2.dpad_left)
            wobbleServo.setPosition(1);
        if(gamepad2.dpad_right)
            wobbleServo.setPosition(0);
        if(gamepad2.dpad_down)
            wobbleServo.setPosition(0.5);

        if(gamepad2.right_bumper)
            flyWheel.setPower(0.85f);
        else
            flyWheel.setPower(0f);

        drive = gamepad1.right_stick_y;
        strafe = -gamepad1.right_stick_x;
        rotate = -gamepad1.right_trigger + gamepad1.left_trigger;

        rearLeftPower = -strafe + drive + rotate;
        frontLeftPower = strafe + drive + rotate;
        rearRightPower = strafe + drive - rotate;
        frontRightPower = -strafe + drive - rotate;

        rearLeftPower = Range.clip(rearLeftPower, -1.0, 1.0);
        rearRightPower = Range.clip(rearRightPower, -1.0, 1.0);
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);

        if (gamepad1.right_bumper) {
            rearLeftPower *= 0.3;
            frontLeftPower *= 0.3;
            rearRightPower *= 0.3;
            frontRightPower *= 0.3;
        }


        if(gamepad1.left_bumper)
            supress2 = !supress2;

        if(supress2)
        {
            rearLeftPower *= 0.6;
            frontLeftPower *= 0.6;
            rearRightPower *= 0.6;
            frontRightPower *= 0.6;
        }

        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftRear.setPower(rearLeftPower);
        rightRear.setPower(rearRightPower);

        if(gamepad2.left_bumper)
        {
            if(runtime.seconds() > 1.6)
            {
                runtime.reset();
                servo.setPosition(0);
            }
            else if(runtime.seconds() > 0.8)
                servo.setPosition(0.5);
        }
        else
            servo.setPosition(0.5);
        /*
        if(gamepad2.dpad_left)
        {
            servo.setPosition(0.5);
        }
        if(gamepad2.dpad_right)
        {
            servo.setPosition(1);
        }
         */
    }

    @Override
    public void stop()
    {

    }

}
