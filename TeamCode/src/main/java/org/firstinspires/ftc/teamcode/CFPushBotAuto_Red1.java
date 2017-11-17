package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ControlFreaks.CFPushBotHardware;


/**
 * Created by adevries on 11/6/2015.
 */
@Autonomous(name="Red1", group="Red")
//@Disabled
public class CFPushBotAuto_Red1 extends LinearOpMode
{
    /* Declare OpMode members. */
    CFPushBotHardware robot   = new CFPushBotHardware();

    boolean v_useGyro = false;
    boolean v_turnSlow = true;
    int v_lifter_step = 500;
    //--------------------------------------------------------------------------
    //
    // v_state
    //
    /**
     * This class member remembers which state is currently active.  When the
     * start method is called, the state will be initialized (0).  When the loop
     * starts, the state will change from initialize to state_1.  When state_1
     * actions are complete, the state will change to state_2.  This implements
     * a state machine for the loop method.
     * 1680 ticks in a 60 moters
     * 560 ticks in a 20 moters
     * 1120 ticks in a 40 moter
     */
    private int v_state = 0;

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during auto-operation.
     * The state machine uses a class member and encoder input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        int color = -1;
        float jewelKnockDistance=0;
        robot.init(this);
        robot.drive_power_override(.3F,.3F, .3F);
        robot.turn_power_override(.3F, .3F);
        robot.redled_on();
        //robot.redled_on();
        robot.led7seg_timer_init(30);
        waitForStart();
        // run until the end of the match (driver presses STOP)
        robot.led7seg_timer_start(30);
        while (opModeIsActive()) {
            robot.hardware_loop();

            switch (v_state) {
                //
                // Synchronize the state machine and hardware.
                //
                case 0:
                    //
                    // drive Forward  inches
                    //
                    //robot.led7seg_timer_start(30);
                    robot.blockgrabber_close();
                    robot.jewel_lower();
                    robot.sensor_color_enable(true);
                    robot.sensor_color_led(true);

                    v_state++;
                    break;
                case 1:
                    //enable color sensor
                    robot.timewait(2);
                    // Transition to the next state when this method is called again.
                    v_state++;
                    break;
                case 2:
                    if(robot.timewait_Complete()){
                        robot.lifter_step(v_lifter_step);
                        robot.timewait(5);
                        v_state++;
                    }
                    break;
                case 3:
                    color = robot.sensor_color_GreatestColor();
                    if (color == 0 || color == 2)
                    {
                        robot.set_message("Color Read " + color);
                        v_state++;
                    }else{
                        if(robot.timewait_Complete()){
                            robot.set_message("Color Timeout ");
                            v_state = v_state+ 3;
                        }
                    }
                    break;
                case 4:
                    if (color == 0) {
                        jewelKnockDistance = -2;
                        robot.drive_inches(jewelKnockDistance, v_useGyro);
                        robot.timewait(2);
                        v_state++;
                    }else if(color==2){
                        jewelKnockDistance = 2;
                        robot.drive_inches(jewelKnockDistance,  v_useGyro);
                        robot.timewait(2);
                        v_state++;
                    }
                    break;
                case 5:
                    if(robot.drive_inches_complete()){
                        v_state++;
                    }
                    break;
                case 6:
                    robot.jewel_raise();
                    robot.timewait(2);
                    v_state++;
                    break;
                case 7:
                    if(robot.timewait_Complete()) {
                        v_state++;
                    }
                    break;
                case 8:
//                    if (color == 0) {
//                        robot.drive_inches(-3,v_useGyro);
//                    }else if(color==2){
//                        robot.drive_inches(3, v_useGyro);
//                    }
                    v_state++;
                    //hi

                    break;
                case 9:
//                    if(robot.drive_inches_complete()){
                    v_state++;
//                    }
                    break;
                case 10:
                    robot.drive_inches(15 - jewelKnockDistance,v_useGyro);
                    v_state++;
                    break;
                case 11:
                    if(robot.drive_inches_complete())
                    {
                        v_state++;
                    }
                    break;
                case 12:
                    robot.turn_degrees(45,false,v_useGyro);
                    robot.timewait(2);
                    v_state++;
                    break;
                case 13:
                    if (robot.turn_complete()| robot.timewait_Complete())
                    {
                        v_state++;
                    }
                    break;
                case 14:
                    robot.drive_inches(10,v_useGyro);
                    robot.timewait(2);
                    v_state++;
                    break;
                case 15:
                    if (robot.drive_inches_complete() || robot.timewait_Complete())
                    {
                        v_state++;
                    }
                    break;
                case 16:
                    robot.blockgrabber_open();
                    v_state++;
                    break;
                case 17:
                    if(robot.timewait_Complete()){
                        v_state++;
                    }
                    break;
                case 18:
                    robot.drive_inches(-2,v_useGyro);
                    robot.timewait(1);
                    v_state++;
                    break;
                case 19:
                    if(robot.drive_inches_complete() || robot.timewait_Complete()){
                        v_state++;
                    }
                    break;
                case 20:
                    robot.lifter_step(-v_lifter_step);
                    robot.set_message("the robot worked thank connor ");
                    //robot.play_jingle_bells();
                    v_state++;
                    break;

                default:
                    //
                    // The autonomous actions have been accomplished (i.e. the state has
                    // transitioned into its final state.
                    //
                    break;
            }



        } // loop
    }




}