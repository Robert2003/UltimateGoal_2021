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

import static java.lang.Math.cos;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

@TeleOp(name="MecanumTeleOP", group="Iterative Opmode")
//@Disabled
public class MecanumTeleOP extends OpMode
{
    private RobotConstants robot = new RobotConstants();
    private ElapsedTime runtime = new ElapsedTime();

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
        robot.init(hardwareMap);
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
        telemetry.addData("flyWheel velocity", robot.flyWheel.getVelocity());
        telemetry.addData("position", robot.wobbleArm.getCurrentPosition());
        telemetry.update();

        if (gamepad2.y) {
            robot.intake1.setPower(1);
            robot.intake2.setPower(1);
        } else if (gamepad2.a) {
            robot.intake1.setPower(-1);
            robot.intake2.setPower(-1);
        } else if (gamepad2.x) {
            robot.intake1.setPower(0);
            robot.intake2.setPower(0);
        }

        powerLatchingUp = gamepad2.left_trigger;
        powerLatchingDown = -gamepad2.right_trigger;

        /*if((powerLatchingUp==0 && powerLatchingDown==0))
        {
            robot.wobbleArm.setTargetPosition(robot.wobbleArm.getCurrentPosition());
            robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleArm.setPower(1);
        }
        else
            robot.wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(powerLatchingDown!=0 || powerLatchingUp!=0)
            robot.wobbleArm.setPower((powerLatchingUp+powerLatchingDown)*0.5);
*/
        if(gamepad2.left_stick_button)
        {
            robot.wobbleArm.setTargetPosition(-610);
            robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleArm.setPower(0.3);
        }
        else if(gamepad2.right_stick_button)
        {
            robot.wobbleArm.setTargetPosition(-300);
            robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleArm.setPower(0.6);
        }
        else if(Math.abs(robot.wobbleArm.getTargetPosition()-robot.wobbleArm.getCurrentPosition()) < 15)
        {
            robot.wobbleArm.setTargetPosition(robot.wobbleArm.getCurrentPosition());
            robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleArm.setPower(1);
        }

        if(gamepad2.dpad_left)
            robot.wobbleServo.setPosition(1);
        if(gamepad2.dpad_right)
            robot.wobbleServo.setPosition(0);
        if(gamepad2.dpad_down)
            robot.wobbleServo.setPosition(0.5);

        if(gamepad2.right_bumper)
            robot.flyWheel.setPower(0.7f);
        else
            robot.flyWheel.setPower(0f);

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

        robot.leftFront.setPower(frontLeftPower);
        robot.rightFront.setPower(frontRightPower);
        robot.leftRear.setPower(rearLeftPower);
        robot.rightRear.setPower(rearRightPower);

        if(gamepad2.left_bumper)
        {
            if(runtime.seconds() > 1.6)
            {
                runtime.reset();
                robot.servo.setPosition(0);
            }
            else if(runtime.seconds() > 0.8)
                robot.servo.setPosition(0.5);
        }
        else
            robot.servo.setPosition(0.5);
    }

    @Override
    public void stop()
    {

    }

    public double speed(double X, double Y)
    {
        double alfa = 37.0;
        double deltaY = Y-0.15;
        double numarator = 4.9*X*X;
        double numitor = (tan(toRadians(alfa))*X-deltaY)*cos(toRadians(alfa))*cos(toRadians(alfa));
        double cat = numitor/numarator;
        return sqrt(cat);
    }
}
