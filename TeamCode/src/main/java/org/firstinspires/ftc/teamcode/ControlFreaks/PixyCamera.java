package org.firstinspires.ftc.teamcode.ControlFreaks;

//import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.CstArray;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;

/**
 * Created by adevries on 11/13/2017.
 */
public class PixyCamera {
    static final String logId = "Pixy:";       // Tag identifier in FtcRobotController.LogCat
    private String v_deviceName = "Not Set";
    private byte I2CAddress = 0x54;
    private Wire v_pixy;

    // Communication/misc parameters
    private static final int PIXY_START_WORD = 0xaa55;
    private static final int PIXY_START_WORD_CC = 0xaa56;
    private static final int PIXY_START_WORDX = 0x55aa;
    private static final int PIXY_MAX_SIGNATURE = 7;
    private static final byte PIXY_SERVO_SYNC = (byte) 0xff;
    private static final byte PIXY_CAM_BRIGHTNESS_SYNC = (byte) 0xfe;
    private static final byte PIXY_LED_SYNC = (byte) 0xfd;
    // Pixy x-y position values
    private static final int PIXY_MIN_X  = 0;
    private static final int PIXY_MAX_X = 319;
    private static final int PIXY_MIN_Y =  0;
    private static final int PIXY_MAX_Y = 199;

    // RC-servo values
    private static final int PIXY_RCS_MIN_POS = 0;
    private static final int PIXY_RCS_MAX_POS =  1000;
    private static final int PIXY_RCS_CENTER_POS = ((PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS)/2);

    public enum BlockType {
        NORMAL_BLOCK,
        CC_BLOCK
    }

    private BlockType g_blockType;
    private boolean v_pixy_enabled = false;

    public PixyCamera(HardwareMap hardwareMap, String deviceName, byte RealI2CAddress) throws Exception {
        try {
            I2CAddress = RealI2CAddress;
            v_deviceName = deviceName;
            v_pixy = new Wire(hardwareMap, deviceName, I2CAddress * 2);
            init();
        } catch (Exception p_exeception) {
            debugLogException("Error Creating Wire", p_exeception);
            v_pixy = null;
            throw p_exeception;
        }
    }

    public PixyCamera(HardwareMap hardwareMap, String deviceName) throws Exception

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.
        try {
            v_deviceName = deviceName;
            v_pixy = new Wire(hardwareMap, deviceName, I2CAddress * 2);
            init();

        } catch (Exception p_exeception) {
            debugLogException("Error Creating Wire", p_exeception);
            v_pixy = null;
        }

    }

    private void init() throws Exception {
        try {
            if (v_pixy != null) {
//                v_ledseg.write(0x21, 0);
//                v_ledseg.write(0x81, 0);
//                v_ledseg.beginWrite(0x00);
//                v_ledseg.write(numbertable[0]);
//                v_ledseg.write(0x00);
//                v_ledseg.write(numbertable[0]);
//                v_ledseg.write(0x00);
//                v_ledseg.write(0x2);
//                v_ledseg.write(0x00);
//                v_ledseg.write(numbertable[0]);
//                v_ledseg.write(0x00);
//                v_ledseg.write(numbertable[0]);
//                v_ledseg.write(0x00);
//                v_ledseg.endWrite();
                v_pixy_enabled = true;
            }
        } catch (Exception p_exeception) {
            debugLogException("Error Init", p_exeception);
            throw p_exeception;
        }
    }


    /**
     * put this in your loop so its called over and over in the loop
     */
    boolean readingData = false;
    Block[] Block;
    public void loop() {
//        if(v_pixy_enabled){
//            if(readingData == false) {
//                v_pixy.beginWrite(0x00);
//                v_pixy.endWrite();
//                readingData = true;
//                v_pixy.getResponse();
//            }else{
//
//            }
//        }
    }
/*
int setServos(uint16_t s0, uint16_t s1)
{
  uint8_t outBuf[6];

  outBuf[0] = 0x00;
  outBuf[1] = PIXY_SERVO_SYNC;
  *(uint16_t *)(outBuf + 2) = s0;
  *(uint16_t *)(outBuf + 4) = s1;

  return send(outBuf, 6);
}

 */

    public boolean set_servos(int s0, int s1){
        if(v_pixy!=null) {
            v_pixy.beginWrite(0x00);
            v_pixy.write(PIXY_SERVO_SYNC);
            v_pixy.write(s0); //two bytes here may need shift
            v_pixy.write(s1);
            v_pixy.endWrite();
            return true;
        }else{
            return false;
        }
    }

    public boolean set_led(byte red, byte green, byte blue){
        if(v_pixy!=null) {
            v_pixy.beginWrite(0x00);
            v_pixy.write(PIXY_LED_SYNC);
            v_pixy.write(red);
            v_pixy.write(green);
            v_pixy.write(blue);
            v_pixy.endWrite();
            return true;
        }else{
            return false;
        }
    }

    public boolean set_brightness(byte brightness) {
        if(v_pixy!=null) {
            v_pixy.beginWrite(0x00);
            v_pixy.write(PIXY_CAM_BRIGHTNESS_SYNC);
            v_pixy.write(brightness);
            v_pixy.endWrite();
            return true;
        }else{
            return false;
        }
    }



    public boolean isEnabled(){
        return v_pixy_enabled;
    }
    public boolean enabled(boolean enable){
        try{
            if (v_pixy_enabled != enable) {
                v_pixy_enabled = enable;
            }
            return true;
        }catch (Exception p_exeception)
        {
            debugLogException("Error Init", p_exeception);
            return false;
        }
    }

    public void stop() {
        if(v_pixy != null) {
            v_pixy.close();
        }
    }
    void debugLogException( String msg, Exception ex){

        String debugMessage = logId + msg;
        if (ex != null) {
            String errMsg = ex.getLocalizedMessage();
            if (errMsg != null) {
                debugMessage = debugMessage + errMsg;
            }else{
                debugMessage = debugMessage + " error. is null";
            }
        }else{
            debugMessage = debugMessage + " error is null";
        }
        android.util.Log.d("pixyCamera",debugMessage);
        //DbgLog.msg(debugMessage);
        //telemetry.addData(line, debugMessage);
    }
    void debugPrint( String msg){

        String debugMessage = logId + msg;
        android.util.Log.d("pixyCamera",debugMessage);
        //DbgLog.msg(debugMessage);
        //telemetry.addData(line, debugMessage);
    }

    public class Block
    {
        // print block structure!
        public String print()
        {
            int i, j;
            String sig = "";
            int d;
            Boolean flag;
            if (signature>PIXY_MAX_SIGNATURE) // color code! (CC)
            {
                // convert signature number to an octal string
                for (i=12, j=0, flag=false; i>=0; i-=3)
                {
                    d = (signature>>i)&0x07;
                    if (d>0 && !flag)
                        flag = true;
                    if (flag)
                        sig = sig +  d + '0';
                }

                return "CC block! sig: " + signature + " x:" + x + " y:" + y + " width:" + width +  " height:" + height + " angle:" + angle;
            }
            else // regular block.  Note, angle is always zero, so no need to print
                return "sig: " + signature + " x:" + x + " y:" + y + " width:" + width +  "height:" + height;

        }
        public int signature;
        public int x;
        public int y;
        public int width;
        public int height;
        public int angle;
    };

    private boolean  skipStart = false;
    private BlockType blockType;
    private int blockCount = 0;

    private boolean getStart()
    {
        int w, lastw;

        lastw = 0xffff;

        while(true)
        {
            w = v_pixy.readLH();
            if (w==0 && lastw==0)
            {
                //delayMicroseconds(10);
                return false;
            }
            else if (w==PIXY_START_WORD && lastw==PIXY_START_WORD)
            {
                blockType = BlockType.NORMAL_BLOCK;
                return true;
            }
            else if (w==PIXY_START_WORD_CC && lastw==PIXY_START_WORD)
            {
                blockType = BlockType.CC_BLOCK;
                return true;
            }
            else if (w==PIXY_START_WORDX)
            {
                debugPrint("reorder");
                v_pixy.read();
            }
            lastw = w;
        }
    }


    public Block getLargestBlock() {
        if (v_pixy != null) {
//            v_pixy.beginWrite(0x50);
//            v_pixy.endWrite();
            v_pixy.requestFrom(0x50, 1);
            if (v_pixy.responseCount() > 0) {
                Block block = new Block();
                block.signature = v_pixy.readLH();
                block.x = v_pixy.read();
                block.y = v_pixy.read();
                block.width = v_pixy.read();
                block.height = v_pixy.read();
                return block;
            }
        }
        return null;
    }

    public Block[] getBlocks(int maxBlocks) {
        if(v_pixy != null) {
            v_pixy.beginWrite(0x00);
            v_pixy.endWrite();
            int i;
            int w, checksum, sum;
            ArrayList<Block> blocks;
            blocks = new ArrayList<Block>();
            if (!skipStart) {
                if (getStart() == false)
                    return null;
            } else
                skipStart = false;

            for (blockCount = 0; blockCount < maxBlocks; ) {
                checksum = v_pixy.readLH();
                if (checksum == PIXY_START_WORD) // we've reached the beginning of the next frame
                {
                    skipStart = true;
                    blockType = BlockType.NORMAL_BLOCK;
                    //Serial.println("skip");
                    return blocks.toArray(Block);
                } else if (checksum == PIXY_START_WORD_CC) {
                    skipStart = true;
                    blockType = BlockType.CC_BLOCK;
                    return blocks.toArray(Block);
                } else if (checksum == 0) {
                    return blocks.toArray(Block);
                }
                Block block = new Block();
                for (i = 0, sum = 0; i < 6; i++) {
                    if (blockType == BlockType.NORMAL_BLOCK && i >= 5) // skip
                    {
                        block.angle = 0;
                        break;
                    }
                    w = v_pixy.readLH();
                    sum += w;
                    switch (i) {
                        case 0:
                            block.signature = w;
                            break;
                        case 1:
                            block.x = w;
                            break;
                        case 2:
                            block.y = w;
                            break;
                        case 3:
                            block.width = w;
                            break;
                        case 4:
                            block.height = w;
                            break;
                        case 5:
                            block.angle = w;
                            break;
                    }

                    if (checksum == sum) {
                        blockCount++;
                        blocks.add(block);
                    } else {
                        debugPrint("cs error");
                    }

                    w = v_pixy.readLH();
                    if (w == PIXY_START_WORD) {
                        blockType = BlockType.NORMAL_BLOCK;
                    } else if (w == PIXY_START_WORD_CC) {
                        blockType = BlockType.CC_BLOCK;
                    } else {
                        return blocks.toArray(Block);
                    }
                }
            }
            return blocks.toArray(Block);
        }else{
            return null;
        }
    }
}
