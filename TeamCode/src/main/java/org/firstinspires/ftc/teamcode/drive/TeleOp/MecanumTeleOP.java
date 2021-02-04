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

    private DcMotor intake = null;
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

    private BNO055IMU imu;

    @Override
    public void init()
    {
        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear_drive");

        intake = hardwareMap.get(DcMotor.class, "intake");
        flyWheel = hardwareMap.get(DcMotor.class, "flyWheel");
        wobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

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
        Acceleration angle = imu.getGravity();
        telemetry.addData("first", angle.xAccel);
        telemetry.addData("second", angle.yAccel);
        telemetry.addData("third", angle.zAccel);
        telemetry.update();

        if (gamepad2.y) {
            intake.setPower(1);
        } else if (gamepad2.a) {
            intake.setPower(-1);
        } else if (gamepad2.x) {
            intake.setPower(0);
        }

        wobbleArm.setPower(gamepad2.left_stick_y*0.4);

        /*
        flyWheelPower = gamepad2.right_stick_y;
        if(Math.abs(flyWheel.getPower()-flyWheelPower) > 0.05)
            if(Math.abs(flyWheelPower) > 0.75)
                flyWheel.setPower(0.75);
            else
                flyWheel.setPower(flyWheelPower);
         */

        if(gamepad2.right_bumper) flyWheel.setPower(0.85f);
        else flyWheel.setPower(0f);

        drive = -gamepad1.right_stick_y;
        strafe = -gamepad1.right_stick_x;
        rotate = gamepad1.right_trigger - gamepad1.left_trigger;

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

        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftRear.setPower(rearLeftPower);
        rightRear.setPower(rearRightPower);
    }

    @Override
    public void stop()
    {

    }

}
