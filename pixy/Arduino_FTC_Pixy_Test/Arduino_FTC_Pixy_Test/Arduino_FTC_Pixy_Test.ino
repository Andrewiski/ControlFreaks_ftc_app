#include <Wire.h>

#define PIXY_I2C_DEFAULT_ADDR           0x54  

uint8_t addr = PIXY_I2C_DEFAULT_ADDR;




  
  uint16_t getWord()
  {
    uint16_t w;
  uint8_t c;
  Wire.requestFrom((int)addr, 2);
    c = Wire.read();
    w = Wire.read();
    w <<= 8;
    w |= c; 
    return w;
  }
  uint8_t getByte()
  {
  Wire.requestFrom((int)addr, 1);
  return Wire.read();
  }

  int8_t send(uint8_t *data, uint8_t len)
  {
    Wire.beginTransmission(addr);
    Wire.write(data, len);
  Wire.endTransmission();
  return len;
  }
  
  


// Communication/misc parameters
#define PIXY_INITIAL_ARRAYSIZE      30
#define PIXY_MAXIMUM_ARRAYSIZE      130
#define PIXY_START_WORD             0xaa55
#define PIXY_START_WORD_CC          0xaa56
#define PIXY_START_WORDX            0x55aa
#define PIXY_MAX_SIGNATURE          7
#define PIXY_DEFAULT_ARGVAL         0xffff

// Pixy x-y position values
#define PIXY_MIN_X                  0L
#define PIXY_MAX_X                  319L
#define PIXY_MIN_Y                  0L
#define PIXY_MAX_Y                  199L

// RC-servo values
#define PIXY_RCS_MIN_POS            0L
#define PIXY_RCS_MAX_POS            1000L
#define PIXY_RCS_CENTER_POS         ((PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS)/2)


enum BlockType
{
  NORMAL_BLOCK,
  CC_BLOCK
};

struct Block 
{
  // print block structure!
  String print()
  {
    int i, j;
    char buf[128], sig[6], d;
  bool flag;  
    if (signature>PIXY_MAX_SIGNATURE) // color code! (CC)
  {
      // convert signature number to an octal string
      for (i=12, j=0, flag=false; i>=0; i-=3)
      {
        d = (signature>>i)&0x07;
        if (d>0 && !flag)
          flag = true;
        if (flag)
          sig[j++] = d + '0';
      }
      sig[j] = '\0';  
      sprintf(buf, "CC block! sig: %s (%d decimal) x: %d y: %d width: %d height: %d angle %d", sig, signature, x, y, width, height, angle);
    }     
  else{ // regular block.  Note, angle is always zero, so no need to print
      sprintf(buf, "sig: %d x: %d y: %d width: %d height: %d", signature, x, y, width, height);   
  }
  return String(buf);
     
  }
  uint16_t signature = 0;
  uint16_t x = 5555;
  uint16_t y = 5555;
  uint16_t width = 0;
  uint16_t height = 0;
  uint16_t angle = 0;
};



Block *blocks;
boolean  skipStart;
BlockType blockType;
uint16_t blockCount;
uint16_t blockArraySize;



boolean getStart()
{
  uint16_t w, lastw;

  lastw = 0xffff;
  
  while(true)
  {
    w = getWord();
    if (w==0 && lastw==0)
  {
      delayMicroseconds(10);
    return false;
  }   
    else if (w==PIXY_START_WORD && lastw==PIXY_START_WORD)
  {
      blockType = NORMAL_BLOCK;
      return true;
  }
    else if (w==PIXY_START_WORD_CC && lastw==PIXY_START_WORD)
  {
      blockType = CC_BLOCK;
      return true;
  }
  else if (w==PIXY_START_WORDX)
  {
    Serial.println("reorder");
    getByte(); // resync
  }
  lastw = w; 
  }
}

void resize()
{
  blockArraySize += PIXY_INITIAL_ARRAYSIZE;
  blocks = (Block *)realloc(blocks, sizeof(Block)*blockArraySize);
}  
    
uint16_t getBlocks(uint16_t maxBlocks)
{
  uint8_t i;
  uint16_t w, checksum, sum;
  Block *block;
  
  if (!skipStart)
  {
    if (getStart()==false)
      return 0;
  }
  else
  skipStart = false;
  
  for(blockCount=0; blockCount<maxBlocks && blockCount<PIXY_MAXIMUM_ARRAYSIZE;)
  {
    checksum = getWord();
    if (checksum==PIXY_START_WORD) // we've reached the beginning of the next frame
    {
      skipStart = true;
    blockType = NORMAL_BLOCK;
    //Serial.println("skip");
      return blockCount;
    }
  else if (checksum==PIXY_START_WORD_CC)
  {
    skipStart = true;
    blockType = CC_BLOCK;
    return blockCount;
  }
    else if (checksum==0)
      return blockCount;
    
  if (blockCount>blockArraySize)
    resize();
  
  block = blocks + blockCount;
  
    for (i=0, sum=0; i<sizeof(Block)/sizeof(uint16_t); i++)
    {
    if (blockType==NORMAL_BLOCK && i>=5) // skip 
    {
    block->angle = 0;
    break;
    }
      w = getWord();
      sum += w;
      *((uint16_t *)block + i) = w;
    }

    if (checksum==sum)
      blockCount++;
    else
      Serial.println("cs error");
  
  w = getWord();
  if (w==PIXY_START_WORD)
    blockType = NORMAL_BLOCK;
  else if (w==PIXY_START_WORD_CC)
    blockType = CC_BLOCK;
  else
      return blockCount;
  }
}


int8_t setServos(uint16_t s0, uint16_t s1)
{
  uint8_t outBuf[6];
   
  outBuf[0] = 0x00;
  outBuf[1] = 0xff; 
  *(uint16_t *)(outBuf + 2) = s0;
  *(uint16_t *)(outBuf + 4) = s1;
  
  return send(outBuf, 6);
}

int8_t setBrightness(uint8_t brightness)
{
  uint8_t outBuf[3];
   
  outBuf[0] = 0x00;
  outBuf[1] = 0xfe; 
  outBuf[2] = brightness;
  
  return send(outBuf, 3);
}

int8_t setLED(uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t outBuf[5];
  
  outBuf[0] = 0x00;
  outBuf[1] = 0xfd; 
  outBuf[2] = r;
  outBuf[3] = g;
  outBuf[4] = b;
  
  return send(outBuf, 5);
}
void clearI2C(){
    while(Wire.available()){
        uint8_t blockCount = Wire.read();
        Serial.println( "cleared:" + String(blockCount));
      }
  }
void getLegoLargestBlock()
  {
      uint16_t w;
      uint8_t c;
      Block myBlock = blocks[0];
      clearI2C();
      Wire.beginTransmission(addr);    // Get the slave's attention, tell it we're sending a command byte
      Wire.write(0x50); 
      Wire.endTransmission(); 
      //  The command byte, sets pointer to register with address of 0x50
      Wire.requestFrom(addr,(uint8_t)6);          // Tell slave we need to read 6 bytes from the current register
      
      int counter = 0;
      while(Wire.available() < 6 && counter < 10){
        delay(5);
        counter++;
      }
      if(Wire.available()){
        //Two Byte LSB 
        c = Wire.read();
        w = Wire.read();
        w <<= 8;
        w |= c;
        myBlock.signature = w;
        myBlock.x = Wire.read();
        myBlock.y = Wire.read();
        myBlock.width = Wire.read();
        myBlock.height = Wire.read();
        if(myBlock.signature > 7){
          Wire.beginTransmission(addr);    // Get the slave's attention, tell it we're sending a command byte
          Wire.write(0x60); 
          Wire.endTransmission();
          Wire.requestFrom(addr,(uint8_t)1); 
          while(Wire.available() < 1 && counter < 10){
            delay(5);
            counter++;
          }
          myBlock.angle = Wire.read();
          
        }
        Serial.println("bc:0 " +myBlock.print());       
      }else{
        Serial.println("getLegoLargestBlock: No I2C Data");
      }
      
      
  }

  void getLegoSignatureBlock(uint8_t signature)
  {
     
      Block myBlock = blocks[0];
      clearI2C();
      Wire.beginTransmission(addr);    // Get the slave's attention, tell it we're sending a command byte
      uint8_t sigRegister = (0x50 | signature);
      Wire.write(sigRegister); //  The command byte, sets pointer to register with address of 0x50
      Wire.endTransmission(); 
      Wire.requestFrom(addr,(uint8_t)5);          // Tell slave we need to read 5 bytes from the current register
      int counter = 0;
      while(Wire.available() < 5 && counter < 10){
        delay(5);
        counter++;
      }
      if(Wire.available()>=5){
        
        myBlock.signature = signature;
        uint8_t blockCount = Wire.read();
        myBlock.x = Wire.read();
        myBlock.y = Wire.read();
        myBlock.width = Wire.read();
        myBlock.height = Wire.read();
       Serial.println( "bc:" + String(blockCount) + " " +  myBlock.print());       
      }else{
        Serial.println("getLegoSignatureBlock: No I2C Data");
      }
      
      
  }

  void getLegoCCSignatureBlock(uint16_t signature)
  {
      uint8_t lb;
      uint8_t hb;
      Block myBlock = blocks[0];
      clearI2C();
      Wire.beginTransmission(addr);    // Get the slave's attention, tell it we're sending a command byte
      uint8_t sigRegister = (0x58);
      Wire.write(sigRegister); //  The command byte, sets pointer to register with address of 0x58
      lb = lowByte(signature);  //GetLest Significate Byte
      hb = highByte(signature);  //GetLest Significate Byte
      Wire.write(lb); //  Write LowByte
      Wire.write(hb); //  Write HighByte
      Wire.endTransmission(); 
      Wire.requestFrom(addr,(uint8_t)6);          // Tell slave we need to read 5 bytes from the current register
      int counter = 0;
      while(Wire.available() < 6 && counter < 10){
        delay(5);
        counter++;
      }
      if(Wire.available()>=6){
        
        myBlock.signature = signature;
        uint8_t blockCount = Wire.read();
        myBlock.x = Wire.read();
        myBlock.y = Wire.read();
        myBlock.width = Wire.read();
        myBlock.height = Wire.read();
        myBlock.angle = Wire.read();
       Serial.println( "bc:" + String(blockCount) + " " +  myBlock.print());       
      }else{
        Serial.println("getLegoSignatureBlock: No I2C Data");
      }
      
      
  }

  void getFTCLargestSignatureBlocks()
  {
      uint16_t w;
      uint8_t c;
      
      clearI2C();
      Wire.beginTransmission(addr);    // Get the slave's attention, tell it we're sending a command byte
      Wire.write(0x70); 
      Wire.endTransmission(); 
      //  The command byte, sets pointer to register with address of 0x50
      Wire.requestFrom(addr,(uint8_t)26);          // Tell slave we need to read 26 bytes from the current register  5 single Signatures Largest Area to Smallest FirstByte is Count of Total number of blocks
      
      int counter = 0;
      while(Wire.available() < 26 && counter < 10){
        delay(5);
        counter++;
      }
      
      if(Wire.available() >=26){
          uint8_t blockCount = Wire.read();
          Serial.println("getFTCLargestSignatureBlocks block count:" + String(blockCount));
          uint8_t offset = 0;
          for(uint8_t i = 0; i < 5; i++){
            Block myBlock = blocks[i];
            //Two Byte LSB 
            //c = Wire.read();
            //w = Wire.read();
            //w <<= 8;
            //w |= c;
            //myBlock.signature = w;
            myBlock.signature = Wire.read();  //Its only a single byte here as its only Signatures 1-7 When we get CC Signatures we need two bytes.
            myBlock.x = Wire.read();
            myBlock.y = Wire.read();
            myBlock.width = Wire.read();
            myBlock.height = Wire.read();
            myBlock.angle = 0;
            if(i<blockCount){
              Serial.println("b:" + String(i) + " " + myBlock.print());
            }
          }
                 
      }else{
        Serial.println("getFTCLargest4Blocks: No I2C Data");
      }
      
      
  }

  void getFTCLargestSignatureBlocks(uint8_t signature)
  {
      uint16_t w;
      uint8_t c;
      
      clearI2C();
      Wire.beginTransmission(addr);    // Get the slave's attention, tell it we're sending a command byte
      uint8_t sigRegister = (0x70 | signature);  // Valid for 0x71- 0x77  request top 6 blocks of with Signature ID 1-7 
      Wire.write(sigRegister); //  The command byte, sets pointer to register with address of 0x70 + signature
      Wire.endTransmission(); 
      //  The command byte, sets pointer to register with address of 0x50
      Wire.requestFrom(addr,(uint8_t)25); // Tell slave we need to read 25 bytes from the current register  6 Largest Signatures Areas  Largest to Smallest FirstByte is Count of Total number of blocks
      
      int counter = 0;
      while(Wire.available() < 25 && counter < 10){
        delay(5);
        counter++;
      }
      
      if(Wire.available() >=25){
          uint8_t blockCount = Wire.read();
          Serial.println("getFTCLargestSignatureBlocks(" + String(signature) + ") block count:" + String(blockCount));
          uint8_t offset = 0;
          for(uint8_t i = 0; i < 6; i++){
            Block myBlock = blocks[i];
            //Two Byte LSB 
            //c = Wire.read();
            //w = Wire.read();
            //w <<= 8;
            //w |= c;
            //myBlock.signature = w;
            myBlock.signature = signature;  //Its only a single byte here as its only Signatures 1-7 When we get CC Signatures we need two bytes.
            myBlock.x = Wire.read();
            myBlock.y = Wire.read();
            myBlock.width = Wire.read();
            myBlock.height = Wire.read();
            myBlock.angle = 0;
            if(i<blockCount){
              Serial.println("b:" + String(i) + " " + myBlock.print());
            }
          }          
      }else{
        Serial.println("getFTCLargestSignatureBlocks: No I2C Data");
      } 
  }

  void getFTCLargestCCSignatureBlocks()
  {
      uint16_t w;
      uint8_t c;
      
      clearI2C(); //handles any out of sync bytes should do nothing
      Wire.beginTransmission(addr);    // Get the slave's attention, tell it we're sending a command byte
      Wire.write(0x78); //  // Valid for 0x78 request top 4 CC block largest Area First 
      Wire.endTransmission(); 
      //  The command byte, sets pointer to register with address of 0x50
      Wire.requestFrom(addr,(uint8_t)25); // Tell slave we need to read 25 bytes from the current register  4 Largest CC Signatures Areas  Largest to Smallest FirstByte is Count of Total number of CC blocks
      
      int counter = 0;
      while(Wire.available() < 25 && counter < 10){
        delay(5);
        counter++;
      }
      
      if(Wire.available() >=25){
          uint8_t blockCount = Wire.read();
          Serial.println("getFTCLargestCCSignatureBlocks block count:" + String(blockCount));
          uint8_t offset = 0;
          for(uint8_t i = 0; i < 4; i++){
            Block myBlock = blocks[i];
            //Two Byte LSB 
            c = Wire.read();
            w = Wire.read();
            w <<= 8;
            w |= c;
            myBlock.signature = w; //Its a CC Signatures we need two bytes.  Convert Dec to Octal Octal 121 is sig 1 touching sig 2 touching sig 1
            myBlock.x = Wire.read();
            myBlock.y = Wire.read();
            myBlock.width = Wire.read();
            myBlock.height = Wire.read();
            myBlock.angle = 0;
            if(i<blockCount){
              Serial.println("b:" + String(i) + " " + myBlock.print());
            }
          }          
      }else{
        Serial.println("getFTCLargestSignatureBlocks: No I2C Data");
      } 
  }

  void getI2cBlocks(){
    static int i = 0;
    int j;
    uint16_t blockCount;
    char buf[32]; 
    
    blockCount = getBlocks(5);
    
    if (blockCount)
    {
      i++;
      
      // do this (print) every 50 frames because printing every
      // frame would bog down the Arduino
      if (i%50==0)
      {
        sprintf(buf, "Detected %d:\n", blockCount);
        Serial.print(buf);
        for (j=0; j<blockCount; j++)
        {
          sprintf(buf, "  block %d: ", j);
          Serial.print(buf);
          Block myBlock =  blocks[j];
          myBlock.print();
        }
      }
    }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("Starting...\n");
  skipStart = false;
  blockCount = 0;
  blockArraySize = PIXY_INITIAL_ARRAYSIZE;
  blocks = (Block *)malloc(sizeof(Block)*blockArraySize);
  Wire.begin();

}

uint8_t delaytime = 10;
void loop() {
  // put your main code here, to run repeatedly:
  //getLegoLargestBlock();
  //delay(200);
  //getLegoSignatureBlock(2);
  //delay(200);
  //getLegoSignatureBlock(1);
  //delay(200);
  //getLegoSignatureBlock(3);
  //delay(200);
  //getLegoCCSignatureBlock(5201);
  //delay(200);
  getFTCLargestSignatureBlocks(); //Get top 5 Area block with Signature 1-7
  delay(delaytime);
  getFTCLargestSignatureBlocks(1);  //Get top 6 Area block with Signature 1
  delay(delaytime);
  getFTCLargestSignatureBlocks(2);  //Get top 6 Area block with Signature 2
  delay(delaytime);
  getFTCLargestCCSignatureBlocks(); //Get top 4 Area block with Signature 2
  delay(delaytime);
  delay(3000);  
}

