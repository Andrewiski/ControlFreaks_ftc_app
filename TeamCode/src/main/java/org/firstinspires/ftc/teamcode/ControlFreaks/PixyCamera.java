package org.firstinspires.ftc.teamcode.ControlFreaks;

//import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.CstArray;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.List;

/**
 * Created by adevries on 11/13/2017.
 * Requires Special Pixy Firmware
 */
public class PixyCamera {
    static final String logId = "Pixy:";       // Tag identifier in FtcRobotController.LogCat
    private String v_deviceName = "Not Set";
    //Put the Pixy in Lego Mode running my special firmware
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
    private static final int PIXY_MIN_X = 0;
    private static final int PIXY_MAX_X = 319;
    private static final int PIXY_MIN_Y = 0;
    private static final int PIXY_MAX_Y = 199;

    // RC-servo values
    private static final int PIXY_RCS_MIN_POS = 0;
    private static final int PIXY_RCS_MAX_POS = 1000;
    private static final int PIXY_RCS_CENTER_POS = ((PIXY_RCS_MAX_POS - PIXY_RCS_MIN_POS) / 2);

    public enum BlockType {
        NORMAL_BLOCK,
        CC_BLOCK
    }

    private BlockType g_blockType;
    private boolean v_pixy_enabled = false;

    //This is Used to enable querys for a particular Signature
    //Call Signature
    private Boolean[] v_signatureEnable = new Boolean[7];

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

            }
            for (int i = 0; i < v_signatureBlocks.length; i++) {
                v_signatureBlocks[i] = v_emptyBlock;
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
    Block v_emptyBlock = new Block();
    // Signature 0 is Max Area Block 1-7 is Signatures 1-7
    Block[] v_signatureBlocks = new Block[8];

    private void beginrequests() {
        if (v_signatureEnable[0]) {
            v_pixy.requestFrom(0x50, 6);
        }
        if (v_signatureEnable[1]) {
            v_pixy.requestFrom(0x51, 4);
        }
        if (v_signatureEnable[2]) {
            v_pixy.requestFrom(0x52, 4);
        }
        if (v_signatureEnable[3]) {
            v_pixy.requestFrom(0x53, 4);
        }
        if (v_signatureEnable[4]) {
            v_pixy.requestFrom(0x54, 4);
        }
        if (v_signatureEnable[5]) {
            v_pixy.requestFrom(0x55, 4);
        }
        if (v_signatureEnable[6]) {
            v_pixy.requestFrom(0x56, 4);
        }
        if (v_signatureEnable[7]) {
            v_pixy.requestFrom(0x57, 4);
        }
    }

    private Block v_signatureBlock;

    public void loop() {
        if (v_pixy_enabled) {
            if (v_pixy.responseCount() > 0) {
                v_pixy.getResponse();
                int regNumber = v_pixy.registerNumber();
                if (v_pixy.isRead()) {
                    int regCount = v_pixy.available();
                    switch (regNumber) {
                        case 0x50: //Largest Block
                            if (regCount == 6) {
                                v_signatureBlock = new Block();
                                v_signatureBlock.numBlocks = 1;
                                v_signatureBlock.signature = v_pixy.readLH();
                                v_signatureBlock.x = v_pixy.read();
                                v_signatureBlock.y = v_pixy.read();
                                v_signatureBlock.width = v_pixy.read();
                                v_signatureBlock.height = v_pixy.read();
                                debugPrint("largestBlock:" + v_signatureBlock.print());
                                //ask for largestBlock again
                                v_signatureBlocks[0] = v_signatureBlock;
                                if (v_signatureEnable[0]) {
                                    v_pixy.requestFrom(0x50, 6);
                                }

                            } else {
                                debugPrint("largestBlock: Error  only " + regCount + " bytes");
                                //ask for largestBlock again
                                if (v_signatureEnable[0]) {
                                    v_pixy.requestFrom(0x50, 6);
                                }
                            }
                            break;
                        case 0x51: //Signature 1 Block
                        case 0x52: //Signature 2 Block
                        case 0x53: //Signature 3 Block
                        case 0x54: //Signature 4 Block
                        case 0x55: //Signature 5 Block
                        case 0x56: //Signature 6 Block
                        case 0x57: //Signature 7 Block
                            int signum = (regNumber & 0x07);
                            if (regCount == 4) {
                                v_signatureBlock = new Block();
                                v_signatureBlock.signature = signum;
                                v_signatureBlock.numBlocks = v_pixy.read();
                                v_signatureBlock.x = v_pixy.read();
                                v_signatureBlock.y = v_pixy.read();
                                v_signatureBlock.width = v_pixy.read();
                                v_signatureBlock.height = v_pixy.read();
                                debugPrint("signature" + signum + ":" + v_signatureBlock.print());
                                v_signatureBlocks[signum] = v_signatureBlock;
                                //ask for SignatureBlock 1 again
                                if (v_signatureEnable[signum]) {
                                    v_pixy.requestFrom((0x50 | signum), 4);
                                }
                            } else {
                                debugPrint("signature" + signum + ": Error  only " + regCount + " bytes");
                                //ask for SignatureBlock again
                                if (v_signatureEnable[signum]) {
                                    v_pixy.requestFrom((0x50 | signum), 4);
                                }
                            }
                            break;

                    }
                }
                if (v_pixy.isWrite()) {
                    int regValue = v_pixy.read();
                    debugPrint(String.format("Pixy Write reg 0x%02X = 0x%02X", regNumber, regValue));
                }
            }
        }
    }


    //Signature 0 is max area Block request
    public void signature_enable(int signature, Boolean enable){
        if(signature >= 0 && signature <=7) {
            v_signatureEnable[signature] = enable;
            if(enable == false) {

            }
        }
    }
    public boolean signature_isEnabled(int signature, Boolean enable){
        if(signature >=0 && signature <=7) {
            return v_signatureEnable[signature];
        }else{
            return false;
        }
    }

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

    public Block largestBlock(){
            return v_signatureBlocks[0];
    }

    public Block largestSignatureBlock(int signature){
        if(signature >= 0 && signature <=7) {
            return v_signatureBlocks[signature];
        }else{
            return v_emptyBlock;
        }
    }

    public Block emptyBlock(){
        return v_emptyBlock;
    }

    public boolean isEnabled(){
        return v_pixy_enabled;
    }
    public boolean enabled(boolean enable){
        try{
            if (v_pixy_enabled != enable) {
                v_pixy_enabled = enable;
                if(enable == true) {
                    //Make the intial requests
                    beginrequests();
                }
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
        public Block(){

        }
        // print block structure!
        public String print()
        {
            try {
                int i, j;
                String sig = "";
                int d;
                Boolean flag;
                if (signature > PIXY_MAX_SIGNATURE) // color code! (CC)
                {
                    // convert signature number to an octal string
                    for (i = 12, j = 0, flag = false; i >= 0; i -= 3) {
                        d = (signature >> i) & 0x07;
                        if (d > 0 && !flag)
                            flag = true;
                        if (flag)
                            sig = sig + d + '0';
                    }

                    return "CC block! sig: " + signature + " x:" + x + " y:" + y + " width:" + width + " height:" + height + " angle:" + angle;
                } else // regular block.  Note, angle is always zero, so no need to print
                    return "sig: " + signature + " x:" + x + " y:" + y + " width:" + width + "height:" + height;
            }catch(Exception ex){
                debugLogException("Block.print()", ex);
                return "Error Block.print() " + ex.getMessage();
            }

        }
        public int signature = -1;
        public int numBlocks = 1;
        public int x = -1;
        public int y = -1;
        public int width = -1;
        public int height = -1;
        public int angle = -1;
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

    ArrayList<Block> blocks;
    private int getBlocks(int maxBlocks) {
        int i;
        int w, checksum, sum;

        blocks = new ArrayList<Block>();
        if (!skipStart) {
            if (getStart() == false)
                return 0;
        } else
            skipStart = false;

        for (blockCount = 0; blockCount < maxBlocks; ) {
            checksum = v_pixy.readLH();
            if (checksum == PIXY_START_WORD) // we've reached the beginning of the next frame
            {
                skipStart = true;
                blockType = BlockType.NORMAL_BLOCK;
                //Serial.println("skip");
                return blockCount;
            } else if (checksum == PIXY_START_WORD_CC) {
                skipStart = true;
                blockType = BlockType.CC_BLOCK;
                return blockCount;
            } else if (checksum == 0) {
                return blockCount;
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
                    return blockCount;
                }
            }
        }
        return blockCount;
    }
}
