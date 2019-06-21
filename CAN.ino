#include <TimerOne.h>

#define SYNC 0
#define SEG1 1
#define SEG2 2
#define WINDOW1 3
#define WINDOW2 4

#define SJW 2
#define ts1 3
#define ts2 3
#define TqTime 15000 //us

#define Pin_CAN_TX 2
#define Pin_CAN_RX 3

volatile int f_hard_sync=0;
volatile int f_soft_sync=0;
volatile int f_timer=0;
volatile int state_ERRO = 0;

volatile int state = 0;
volatile int phase_error=0;
volatile int cts1=0;
volatile int cts2=0;
volatile int ctsS=0;
volatile int CTq=0;
volatile int f_statew2 = 0, f_statew1 = 0, f_hs=0;
volatile int W_P=0;
volatile int S_P =0;
volatile int ERRO = 0, BREAK = 0, estado_stuffing, C_Frame_Stuffing, C_Frame_Pos = 0, CAN_RX = 1, CAN_TX = 1, W_BIT=1, C_bit_stuffing, CAN_IDLE=1, NOTACK=0, C_ERRO=0, CRC_ERROR=-0;
volatile int Stuffing, ACK_Stuffing, Arb_Stuffing, Pos_ACK, Pos_Arb, Frame_Size, Unstuffing, CRC_A;
volatile int Bit_Stream[200], rStream[200];
volatile int rsc=0;

volatile int teste_sp, c_sp=0, K = 0;
volatile int estado_unstuffing = 3, C_unstuffing=0, C_frame_unstuffed = 0, UCLK;
volatile int frame_unstuffed[200];
volatile int estado_F_D = 0, enable_unstuffing=0, C_F_D=0;
volatile char A_D[30], DLC_D[4], DATA_D[70], CRC_D[20], RTR_D, IDE_D, SRR_RTR_D, DLC_D_DEC, SRR_D, CRC_RC[20];
volatile int UCLK_D=1, VALID_CRC=1, f_F_D = 0, Frame_Available = 0, transmitindo = 0;
         
char A[] = {0,0,1,0,1,1,1,1,1,0,0};
char DLC[] = {0,1,0,1};
char DATA[] = {1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0}; 
char RTR = 0;
char IDE = 0;    
unsigned long tempo;
int clk = 0;
void setup(){ 
  Serial.begin(115200); 
  pinMode(Pin_CAN_TX, OUTPUT);
  pinMode(Pin_CAN_RX, INPUT);
  pinMode(11, INPUT);  
  
  Timer1.initialize(TqTime);
  Timer1.attachInterrupt(IR_T); 
  
  tempo = millis();
  
  attachInterrupt(digitalPinToInterrupt(Pin_CAN_RX), SYNC_TQ, FALLING); 
  
f_hard_sync=0;f_soft_sync=0;f_timer=0;state = 0;state_ERRO = 0; phase_error=0; cts1=0; cts2=0; ctsS=0; CTq=0; f_statew2 = 0;
f_statew1 = 0; f_hs=0; W_P=0; S_P =0; ERRO = 0; BREAK = 0; C_Frame_Pos = 0; CAN_RX = 1; CAN_TX = 1; W_BIT=1; CAN_IDLE=1; NOTACK=0; C_ERRO=0;
c_sp=0; K = 0; estado_unstuffing = 3; C_unstuffing=0; C_frame_unstuffed = 0; frame_unstuffed[200]; estado_F_D = 0; enable_unstuffing=0; C_F_D=0;
UCLK_D=1; VALID_CRC=1; f_F_D = 0; Frame_Available = 0; transmitindo = 0;
}

void IR_T(){
  TQ(0);
} 

void SYNC_TQ(){
  if(!transmitindo){
    if(CAN_IDLE == 0)//softSync
    {
      TQ(2);
    }
    else{ //HardSync
      CAN_IDLE = 0;
      TQ(1);    
    }
  }
} 

void bit_timing(int IR){
  if(IR == 0){    
    f_soft_sync = f_soft_sync==1? 0:f_soft_sync;  
    f_timer = 1;
    CTq++;
    if (f_hs==1){
      state = SEG1;
      f_hs=0;
    }else
      f_hard_sync = f_hard_sync==1? 0:f_hard_sync;
            
    if (f_statew2==1){
      state = WINDOW2;
      f_statew2=0;
    }
    
    if (f_statew1==1){
      state = WINDOW1;
      f_statew1=0;
    }
    
    if(state==SEG1 || state==WINDOW1){
      cts1++;
    }else if(state==SEG2 || state==WINDOW2){
      cts2++;
    }else if(state == SYNC){    
      ctsS++;
      CTq=0;
    }  
  
    switch(state){
      case SEG1:
        cts2=0;
        W_P=0;
        S_P=0;
        if(cts1==ts1){
          state=SEG2;
          S_P = 1;
        }
      break;
        
      case SEG2:
        cts1=0;      
        S_P=0;
        W_P=0;
        if(cts2==ts2){
          W_P=1;
          state=SYNC;
        }
      break;  
      
      case SYNC:
        S_P=0;
        if(ctsS == 1){
          state=SEG1;
          ctsS=0;
          W_P = 0;        
        }
      break;  
      
      case WINDOW1:
        S_P=0;
        if((phase_error>SJW && cts1==ts1+SJW-1) || (phase_error<=SJW && cts1==ts1+phase_error-1)){
          S_P = 1;
          state=SEG2;
        }
      break;  
      
      case WINDOW2:
        W_P=0;
        S_P=0;
        if((abs(phase_error)>SJW && cts2==ts2-SJW+1) || (abs(phase_error)<=SJW && cts2==ts2+phase_error+1)){
          state=SEG1;
          CTq=1;
        }
      break;
    }
  }

  if(IR == 1){
    f_hard_sync=1;
    clk = clk == 0? 1:0;    
    switch(state){
      case SEG1:
        cts2=0;
        cts1=-1;
        W_P=0;
        S_P=0;
        Timer1.restart();
        f_hs=1;       
      break;
      case SEG2:
        cts1=0;
        if(S_P==1){
          S_P=0;
        }
        S_P=1;
        W_P=0;     
        Timer1.restart();
        f_hs=1;     
      break;
    }
  }
  
  if(IR == 2){
    f_soft_sync=1;
    switch(state){
      case SEG1:
        cts2=0;        
        W_P=0;
        S_P=0;     
        Timer1.initialize(TqTime);        
        phase_error=cts1+1;
        f_statew1=1;      
      break;
      case SEG2:
        cts1=0;
        if(S_P==1){
          S_P=0;
        }
        S_P=1;
        W_P=0;      
        Timer1.initialize(TqTime);
        phase_error=cts2-ts2;
        f_statew2=1;        
      break;
    }  
  }
}

void  bit_stuffing(){  
  if(BREAK || ERRO){
    C_Frame_Stuffing = Frame_Size;
    Arb_Stuffing = 0;    
  }

  if(!ERRO && !BREAK && W_P && !CAN_IDLE)
  {
    if(C_Frame_Stuffing < (Frame_Size - 25))
    {
      CRC_A = 1;
      switch(estado_stuffing)
      {
        case 0:
          if(Bit_Stream[C_Frame_Stuffing] == 0 && C_bit_stuffing < 5)
          {
            CAN_TX=0;
            W_BIT=0;
            Stuffing=0;
            C_bit_stuffing++;
            C_Frame_Stuffing++;
          }
          else if(Bit_Stream[C_Frame_Stuffing] == 1 && C_bit_stuffing < 5)
          {
            CAN_TX=1;
            W_BIT=1;
            Stuffing=0;
            C_bit_stuffing=1;
            C_Frame_Stuffing++;
            estado_stuffing=1;
          }
          else if(C_bit_stuffing == 5)
          {
            CAN_TX=1;
            W_BIT=1;
            Stuffing=1;
            C_bit_stuffing=1;
            estado_stuffing=Bit_Stream[C_Frame_Stuffing];
          }
          break;

        case 1:
        if(Bit_Stream[C_Frame_Stuffing] == 0 && C_bit_stuffing < 5)
          {
            CAN_TX=0;
            W_BIT=0;
            Stuffing=0;
            C_bit_stuffing=1;
            C_Frame_Stuffing++;
            estado_stuffing=0;
          }
          else if(Bit_Stream[C_Frame_Stuffing] == 1 && C_bit_stuffing < 5)
          {
            CAN_TX=1;
            W_BIT=1;
            Stuffing=0;
            C_bit_stuffing++;
            C_Frame_Stuffing++;
          }
          else if(C_bit_stuffing == 5)
          {
            CAN_TX=0;
            W_BIT=0;
            Stuffing=1;
            C_bit_stuffing=1;
            estado_stuffing=Bit_Stream[C_Frame_Stuffing];
          }
          break;
      }
    }
    else{//A partir do CRC
      CRC_A = 0;
      CAN_TX = Bit_Stream[C_Frame_Stuffing];
      W_BIT = Bit_Stream[C_Frame_Stuffing++];
    }
    if(C_Frame_Stuffing == Pos_ACK+1)
      ACK_Stuffing = 1;
    else
      ACK_Stuffing = 0;

    if(C_Frame_Stuffing < Pos_Arb && C_Frame_Stuffing)
      Arb_Stuffing = 1;
    else
      Arb_Stuffing = 0;

    C_Frame_Pos++;
  }
  else if(!ERRO && !BREAK && W_P){
    CAN_TX = 1;
  }
  if(C_Frame_Stuffing == Frame_Size && !ERRO){
    CAN_IDLE = 1;
    C_Frame_Pos = 0;
    transmitindo = 0;
  }
}

void unstuffing(){
    if(estado_unstuffing == 3 && f_hard_sync){
        estado_unstuffing = 0;
        C_frame_unstuffed = 0;//Contador do tamanho do frame
        C_unstuffing = 0;//Contador interno para unstuffing < 5     
        enable_unstuffing=1;
        f_F_D = 0;
    }        
    if(S_P && enable_unstuffing){
        if(f_F_D) {
          estado_unstuffing = 3;
          UCLK = 1-UCLK; //bypass
        }
        else
        switch(estado_unstuffing)
        {
        case 0:
          if(CAN_RX == 0 && C_unstuffing < 5)
          {       
            Unstuffing = 0;
            UCLK = 1-UCLK;          
            C_unstuffing++;
          }
          else if(CAN_RX == 1 && C_unstuffing < 5)
          {
            Unstuffing = 0;
            UCLK = 1-UCLK;            
            C_unstuffing=1;
            estado_unstuffing=1;
          }
          else if(C_unstuffing == 5)
          {            
            Unstuffing = 1;
            C_unstuffing=1;
            estado_unstuffing=CAN_RX;
          }
          break;

        case 1:
        if(CAN_RX == 0 && C_unstuffing < 5)
          {
            Unstuffing = 0;
            UCLK = 1-UCLK;            
            C_unstuffing=1;
            estado_unstuffing=0;
          }
          else if(CAN_RX == 1 && C_unstuffing < 5)
          {
            Unstuffing = 0;
            UCLK = 1-UCLK;
            CAN_TX=1;
            W_BIT=1;            
            C_unstuffing++;
          }
          else if(C_unstuffing == 5)
          { 
            Unstuffing = 1;                      
            C_unstuffing=1;
            estado_unstuffing=CAN_RX;
          }
          break;
      }      
    }
    
}

void frame_decoder(){
    if (UCLK_D == 1-UCLK)
    {
        UCLK_D = UCLK;
        switch (estado_F_D)
        {
            case 0 : //idle
                if (CAN_RX == 0){//HardSync e Start of Frame
                    rsc=0;
                    rStream[rsc++]=CAN_RX;
                    estado_F_D = 1;
                    enable_unstuffing = 1;
                    C_F_D = 0;
                }            
            break;        
            case 1://ID A
                A_D[C_F_D++] = CAN_RX;
                rStream[rsc++]=CAN_RX;
                if (C_F_D == 11){
                    estado_F_D = 2;
                    C_F_D = 0;
                }                
            break;
            case 2://SRR/RTR
                rStream[rsc++]=CAN_RX;
                SRR_RTR_D = CAN_RX;
                estado_F_D = 3;
            break;
            case 3://IDE
                rStream[rsc++]=CAN_RX;
                if (CAN_RX == 1 && SRR_RTR_D == 1){
                    C_F_D = 0;
                    IDE_D = 1;
                    SRR_D = SRR_RTR_D;
                    estado_F_D = 4;
                }else if (CAN_RX == 0){
                    IDE_D = 0;
                    RTR_D = SRR_RTR_D;
                    estado_F_D = 7;
                } else if(CAN_RX == 1 && SRR_RTR_D == 0){
                    IDE_D = 1;
                    estado_F_D = 15;
                }      
            break;
            case 4://ID B
              rStream[rsc++]=CAN_RX;
              if(C_F_D!=18)
                {
                  A_D[11+C_F_D++] = CAN_RX; 
                }
                if (C_F_D==18){
                    estado_F_D = 5;
                    C_F_D = 0;
                }                
                        
            break;
            case 5://RTR B
                rStream[rsc++]=CAN_RX;
                estado_F_D = 6;
            break;
            case 6://R1
                rStream[rsc++]=CAN_RX;
                estado_F_D = 7;
            break;
            case 7://R0
                rStream[rsc++]=CAN_RX;
                estado_F_D = 8;
                C_F_D = 0;
            break;
            case 8://DLC            
                    rStream[rsc++]=CAN_RX;
                    DLC_D[C_F_D++] = CAN_RX;
                    if (C_F_D == 4){
                       if(RTR_D == 1){                          
                          estado_F_D = 10;
                          C_F_D = 0;
                       }else{                      
                          DLC_D_DEC = DLC_to_DEC(DLC_D);
                          //DLC_D_DEC = DLC_D_DEC = 8;
                          estado_F_D = 9;
                          C_F_D = 0;
                        }
                    }
                             
            break;
            case 9://DATA
                rStream[rsc++]=CAN_RX;
                DATA_D[C_F_D++] = CAN_RX;
                if (C_F_D == DLC_D_DEC){
                    C_F_D = 0;
                    estado_F_D = 10;
                }            
            break;
            case 10://CRC
                CRC_D[C_F_D++] = CAN_RX;
                if (C_F_D == 15){
                    estado_F_D = 11;
                    f_F_D = 1;
                }            
            break;
            case 11://CRC D
                if(!checa_CRC(rsc))
                {
                  ERRO = 1;
                }
                if (VALID_CRC == 0 || CAN_RX == 0){
                    estado_F_D = 15;
                }else{
                    estado_F_D = 12;
                }           
            break;
            case 12: //ACK, condicional usando numero do estado
                estado_F_D = 13;
            break;
            case 13: //ACK D
                if (CAN_RX == 0){
                    estado_F_D = 15;
                }else{
                    C_F_D = 0;
                    estado_F_D = 14;
                }           
            break;
            case 14: //EOF
                C_F_D++;
                if (CAN_RX == 0)
                    estado_F_D = 15;               
                   
                if(C_F_D == 7){
                    f_F_D = 0;
                    estado_F_D = 0;
                    enable_unstuffing = 0;
                    CAN_IDLE = 1;
                    Frame_Available = 1;
                }            
            break;
            case 15://ERRO
                f_F_D = 0;
            break;            
        }
    }
    if (W_P && estado_F_D == 12){
        CAN_TX = 0;
    }
    else if(W_P && estado_F_D!=12)
    {
      CAN_TX = 1;
    }
}

void listener(){
  if(!ERRO && S_P && !CAN_IDLE){
    if(ACK_Stuffing){
      NOTACK=CAN_RX;
    }
    else{
      NOTACK=3;  
    }
    if(CAN_RX != W_BIT && !ACK_Stuffing && Arb_Stuffing && !Stuffing){
      BREAK=1;
    }
    if((CAN_RX != W_BIT && Stuffing) || (CAN_RX == 0 && W_BIT == 1&&!ACK_Stuffing&&!Arb_Stuffing)){
      ERRO=1;
    }
  }
}

void error_frame(){
  switch(state_ERRO){
    case 0:   
      if (ERRO == 1){
        state_ERRO = 1;
        C_ERRO = 0;
        }
      break;
    case 1:
      CAN_TX = 0;
      if(W_P && C_ERRO <= 6){
        state_ERRO = 1;
        C_ERRO++;
      }
      if(W_P && C_ERRO == 6){        
        state_ERRO = 2;
      }      
      break;
    case 2:
      CAN_TX = 1;
      if(S_P && CAN_RX ==0){
        state_ERRO = 2;
      }
      else if(S_P && CAN_RX==1){
        state_ERRO = 3;
        C_ERRO = 0;
      }
      break;
    case 3:
      CAN_TX=1;
      if(S_P && C_ERRO <8){
        state_ERRO = 3;
        C_ERRO++;
      }
      if(S_P && C_ERRO == 8){
        C_ERRO=0;        
        state_ERRO = 4;
      }
      break;
    case 4:
      CAN_TX=1;
      if(W_P && CAN_RX == 1 && C_ERRO < 3){
        state_ERRO = 4;
        C_ERRO++;
      }
      if(W_P && CAN_RX == 0){
        ERRO = 1;
        state_ERRO = 1;
        C_ERRO = 0;
      }
      if(W_P && C_ERRO==3){
        ERRO=0;
        state_ERRO = 0;
        C_ERRO=0;
      }
      break;
  }
  }

char DLC_to_DEC(char * DLC){
    char DLC_DEC = 0;
    DLC_DEC += DLC[0]*8;
    DLC_DEC += DLC[1]*4;
    DLC_DEC += DLC[2]*2;
    DLC_DEC += DLC[3];
    if(DLC_DEC>8)
        DLC_DEC = 8;
    DLC_DEC *= 8;
    return DLC_DEC;
}

bool checa_CRC(int size){
    static char Res[16];                                 // CRC Result
    char CRC[15]="";
    int  i;
    char DoInvert;

    for (i=0; i<size; ++i)
    {
        DoInvert = (1==rStream[i]) ^ CRC[14];         // XOR required?
        CRC[14] = CRC[13] ^ DoInvert;
        CRC[13] = CRC[12];
        CRC[12] = CRC[11];
        CRC[11] = CRC[10];
        CRC[10] = CRC[9] ^ DoInvert;
        CRC[9] = CRC[8];
        CRC[8] = CRC[7] ^ DoInvert;
        CRC[7] = CRC[6] ^ DoInvert;
        CRC[6] = CRC[5];
        CRC[5] = CRC[4];
        CRC[4] = CRC[3] ^ DoInvert;
        CRC[3] = CRC[2] ^ DoInvert;
        CRC[2] = CRC[1];
        CRC[1] = CRC[0];
        CRC[0] = DoInvert;
    }
        
    for (i=0; i<15; ++i)  Res[14-i] = CRC[i] ? 1 : 0;
    //Res[0] = 2;
    Res[15] = 0;                                         // Set string terminator
    for(int i=0; i<15; i++)
    {
      if(Res[i] != CRC_D[i])
      {
        return 0;
        CRC_ERROR=1;
      }
    }
    return 1;
}

char* calcula_CRC(int size){
    for(int i=0; i<size; i++)
        printf("%d", Bit_Stream[i]);
    printf("\n");
    static char Res[16];                                 // CRC Result
    char CRC[15]="";
    int  i;
    char DoInvert;

    for (i=0; i<size; ++i)
    {
        DoInvert = (1==Bit_Stream[i]) ^ CRC[14];         // XOR required?
        CRC[14] = CRC[13] ^ DoInvert;
        CRC[13] = CRC[12];
        CRC[12] = CRC[11];
        CRC[11] = CRC[10];
        CRC[10] = CRC[9] ^ DoInvert;
        CRC[9] = CRC[8];
        CRC[8] = CRC[7] ^ DoInvert;
        CRC[7] = CRC[6] ^ DoInvert;
        CRC[6] = CRC[5];
        CRC[5] = CRC[4];
        CRC[4] = CRC[3] ^ DoInvert;
        CRC[3] = CRC[2] ^ DoInvert;
        CRC[2] = CRC[1];
        CRC[1] = CRC[0];
        CRC[0] = DoInvert;
    }
        
    for (i=0; i<15; ++i)  Res[14-i] = CRC[i] ? 1 : 0;
    //Res[0] = 2;
    Res[15] = 0;                                         // Set string terminator
    return(Res);
}

void encoder(char * identifier_A, char IDE, char RTR, char * DLC, char * Data){
    char DLC_DEC;
    char *CRC;
    int i;
    Bit_Stream[0]=0;                                             //SOF
    for(i = 0; i<11;i++) Bit_Stream[i+1]=identifier_A[i];        // ID A
    Bit_Stream[13] = IDE;                                        //IDE
    
    if(IDE){    
        Pos_Arb = 33;
        Bit_Stream[12] = 1;                                      //SRR
        for(i = 13; i<31; i++) Bit_Stream[i+1]=identifier_A[i-2];//ID B
        Bit_Stream[32] = RTR;                                    //RTR
        Bit_Stream[33]=0;                                        //r0
        Bit_Stream[34]=0;                                        //r1
        for(i = 0; i<4;i++) Bit_Stream[i+35] = DLC[i];           //DLC
        DLC_DEC = DLC_to_DEC(DLC);       
        if(RTR==1)
        {
          DLC_DEC=0;
        }
        for(i=0;i<DLC_DEC;i++) Bit_Stream[i+39] = Data[i];       //DATA
        CRC = calcula_CRC(39+(RTR==0)*DLC_DEC);
        for(i=0;i<15;i++) Bit_Stream[i+39+DLC_DEC] = CRC[i];     //CRC
        Bit_Stream[54+DLC_DEC] = 1;                              //CRC delimiter
        Pos_ACK = 55+DLC_DEC;
        Bit_Stream[55+DLC_DEC] = 1;                              //ACK slot
        Bit_Stream[56+DLC_DEC] = 1;                              //ACK delimiter
        for(i=0;i<7;i++) Bit_Stream[i+57+DLC_DEC] = 1;           //EOF        
       Bit_Stream[i+57+DLC_DEC]=2;
        Frame_Size = i+57+DLC_DEC;
    }
    else
    {
        Pos_Arb = 13;
        Bit_Stream[12] = RTR;                                    //RTR
        Bit_Stream[14]=0;                                        //r0
        for(i = 0; i<4;i++) Bit_Stream[i+15] = DLC[i];           //DLC
        DLC_DEC = DLC_to_DEC(DLC);
        if(RTR==1)
        {
          DLC_DEC=0;
        }
        for(i=0;i<DLC_DEC;i++) Bit_Stream[i+19] = Data[i];       //DATA        
        CRC = calcula_CRC(19+(RTR==0)*DLC_DEC);
        for(i=0;i<15;i++) Bit_Stream[i+19+DLC_DEC] = CRC[i];     //CRC
        Bit_Stream[34+DLC_DEC] = 1;                              //CRC delimiter
        Pos_ACK = 35+DLC_DEC;
        Bit_Stream[35+DLC_DEC] = 1;                              //ACK slot
        Bit_Stream[36+DLC_DEC] = 1;                              //ACK delimiter
        for(i=0;i<7;i++) Bit_Stream[i+37+DLC_DEC] = 1;           //EOF

        Bit_Stream[i+37+DLC_DEC]=2;
        Frame_Size = i+37+DLC_DEC;
    }
    for(int i = 0; i< Frame_Size; i++){
      Serial.print(Bit_Stream[i]);  
    }
    Serial.println();
    
    if(CAN_IDLE == 1){
      transmitindo = 1;
      CAN_IDLE = 0;
    }
   
    C_Frame_Stuffing = 0;
    BREAK = 0;
}


void TQ(int IR){
  clk = clk == 0? 1:0;
  bit_timing(IR);
  if(transmitindo && !CAN_IDLE)
  {
    bit_stuffing();
    listener();
  }
  if(!transmitindo)
  {
    frame_decoder();
    unstuffing();
  }
  error_frame();
}

int k = 1;
void loop(){
  Leitura();

  if(CAN_IDLE){
    CAN_TX = 1;
    digitalWrite(Pin_CAN_TX, !1);
  }
  else
    digitalWrite(Pin_CAN_TX, !CAN_TX);
  CAN_RX = digitalRead(Pin_CAN_RX);
  //plotter();
  delay(10);
}
void plotterRXTX(){
 Serial.println(String(CAN_RX)+" "+String(CAN_TX+2));  
}
void plotter(){
   //  Serial.println(String(CAN_IDLE)+" "+String(transmitindo+2));
//   Serial.println(String(CAN_RX)+" "+String(estado_F_D+2)+" "+String(W_BIT+4)+" "+String(Unstuffing+6)+" "+String(Arb_Stuffing+8)+" "+String(C_F_D)+" "+String(BREAK+12)+" "+String(ERRO+14)+" "+String(CAN_IDLE+16));
   //Serial.println(String(a)+" "+String(Unstuffing+2)+" "+String(CRC_A+4)+" "+String(CAN_IDLE+6)+" "+String(CAN_RX+8));
    Serial.println(String(CAN_RX-2)+" "+String(CAN_TX)+" "+String(Arb_Stuffing+2)+" "+String(Stuffing+4)+" "+String(ERRO+6)+" "+String(BREAK+8)+" "+String(NOTACK+10)+" "+String(state_ERRO+12));    
   // Serial.println(String(state)+" "+String(W_P + 4)+" "+String(S_P + 5)+" "+String(f_hard_sync + 6)+" "+String(f_soft_sync + 7)+" "+String(clk+8)+" "+String(CAN_TX+9)+" "+String(Arb_Stuffing+10)+" "+String(ACK_Stuffing+11));
}

void Leitura(){
  /*
  if(ACK_Stuffing && S_P)
     Serial.println("CAN NO ACK - "+ String(CAN_RX));
  else if(S_P)
    Serial.println("TX: "+String(CAN_TX)+" - RX: "+String(CAN_RX));
 */
 // Serial.println("Tran"+String(transmitindo)+"IDEL"+String(CAN_IDLE));
 //Serial.println("NOTACK : "+ String(NOTACK));
  if(Serial.available()){
    if(Serial.read()=='S')
      RTR = 1;      
    else
      RTR = 0;    
    encoder(A, IDE, RTR, DLC, DATA);
    for(int i = 0; i<Frame_Size ; i++)
      Serial.print(String(Bit_Stream[i])+" ");
    Serial.println();
    Serial.println(Frame_Size);
  } 
  if(Frame_Available == 1)
  {
    Frame_Available = 0;
    Serial.print("A-");
    if(IDE_D == 0)
      for(int i = 0; i<11; i++)
        Serial.print((int)A_D[i]);
    if(IDE_D == 1)
      for(int i = 11; i<29; i++)
        Serial.print((int)A_D[i]);
    Serial.print(" RTR-");
    Serial.print((int)RTR_D);
    Serial.print(" IDE-");
    Serial.print((int)IDE_D);    
    Serial.print(" DLC-");    
    for(int i = 0; i<4; i++)
      Serial.print((int)DLC_D[i]);
    if(RTR_D == 0){
      Serial.print(" DATA-");
      for(int i = 0; i<DLC_D_DEC ; i++){
        Serial.print((int)DATA_D[i]);
      }
    }
    Serial.print(" CRC-");
    for(int i = 0; i<15 ; i++){
      Serial.print((int)CRC_D[i]);
    }
    Serial.println();   
  }
}
