//`include "clk_divider.sv"

//`default_nettype none

module Processor_Core( // declaring the inputs and outputs
    input logic clk,
    input logic resetn,
    input logic [31:0] MEM_dout,

    output logic [31:0] MEM_addr,
    output logic rMEM_en,
    output logic MEM_Wdata,
    output logic MEM_Wmask,
    output logic [31:0] x10
);

//wire clk;
//wire resetn;

//reg [4:0] leds;
//assign LEDS = leds;

// declaring a 5 bit register
//reg [31:0] MEM [0:255]; // BRAM
reg [31:0] PC = 0; // Program Counter
reg [31:0] C_INST;

 /*
        // add x0, x0, x0
              //                   rs2   rs1  add  rd   ALUREG
reg [31:0] C_INST =  32'b0000000_00000_00000_000_00000_0110011; //cureent instruction reg

 
  // initiliasing the SoC Memmory with some instructions
   initial begin // this is only for TB simulation purposes

      // add x1, x0, x0
      //                    rs2   rs1  add  rd   ALUREG
      MEM[0] = 32'b0000000_00000_00000_000_00001_0110011;
      // addi x1, x1, 1
      //             imm         rs1  add  rd   ALUIMM
      MEM[1] = 32'b000000000001_00001_000_00001_0010011;
      // addi x1, x1, 1
      //             imm         rs1  add  rd   ALUIMM
      MEM[2] = 32'b000000000001_00001_000_00001_0010011;
      // addi x1, x1, 1
      //             imm         rs1  add  rd   ALUIMM
      MEM[3] = 32'b000000000001_00001_000_00001_0010011;
      // addi x1, x1, 1
      //             imm         rs1  add  rd   ALUIMM
      MEM[4] = 32'b000000000001_00001_000_00001_0010011;
      // add x2, x1, x0
      //                    rs2   rs1  add  rd   ALUREG
      MEM[5] = 32'b0000000_00000_00001_000_00010_0110011;
      // add x3, x1, x2
      //                    rs2   rs1  add  rd   ALUREG
      MEM[6] = 32'b0000000_00010_00001_000_00011_0110011;
      // srli x3, x3, 3
      //                   shamt   rs1  sr  rd   ALUIMM
      MEM[7] = 32'b0000000_00011_00011_101_00011_0010011;
      // slli x3, x3, 31
      //                   shamt   rs1  sl  rd   ALUIMM
      MEM[8] = 32'b0000000_11111_00011_001_00011_0010011;
      // srai x3, x3, 5
      //                   shamt   rs1  sr  rd   ALUIMM
      MEM[9] = 32'b0100000_00101_00011_101_00011_0010011;
      // srli x1, x3, 26
      //                   shamt   rs1  sr  rd   ALUIMM
      MEM[10] = 32'b0000000_11010_00011_101_00001_0010011;

      // ebreak
      //                                          SYSTEM
      MEM[11] = 32'b000000000001_00000_000_00000_1110011;
        
   end

    */

  /* 
   initial begin
      PC = 0;
      ADD(x0,x0,x0);
      ADD(x1,x0,x0);
      ADDI(x1,x1,1);
      ADDI(x1,x1,1);
      ADDI(x1,x1,1);
      ADDI(x1,x1,1);
      ADD(x2,x1,x0);
      ADD(x3,x1,x2);
      SRLI(x3,x3,3);
      SLLI(x3,x3,31);
      SRAI(x3,x3,5);
      SRLI(x1,x3,26);
      EBREAK();
   end
  */

  /*
  `include "riscv_assembly.sv"
  integer L0_ = 8;
  initial begin

    ADD(x1,x0,x0);
    ADDI(x2,x0,32);
  Label(L0_);
    ADDI(x1,x1,1);
    BNE(x1,x2, LabelRef(L0_));
    EBREAK();


    endASM();

  end
  */
   // RISCV instructions Decoder
  //*********************************************************************************//
  //*********************************************************************************//

  // The 10 RISC-V instructions
   wire isALUreg  =  (C_INST[6:0] == 7'b0110011); // rd <- rs1 OP rs2   
   wire isALUimm  =  (C_INST[6:0] == 7'b0010011); // rd <- rs1 OP Iimm
   wire isBranch  =  (C_INST[6:0] == 7'b1100011); // if(rs1 OP rs2) PC<-PC+Bimm
   wire isJALR    =  (C_INST[6:0] == 7'b1100111); // rd <- PC+4; PC<-rs1+Iimm
   wire isJAL     =  (C_INST[6:0] == 7'b1101111); // rd <- PC+4; PC<-PC+Jimm
   wire isAUIPC   =  (C_INST[6:0] == 7'b0010111); // rd <- PC + Uimm
   wire isLUI     =  (C_INST[6:0] == 7'b0110111); // rd <- Uimm   
   wire isLoad    =  (C_INST[6:0] == 7'b0000011); // rd <- mem[rs1+Iimm]
   wire isStore   =  (C_INST[6:0] == 7'b0100011); // mem[rs1+Simm] <- rs2
   wire isSYSTEM  =  (C_INST[6:0] == 7'b1110011); // special

  // The 5 immediate formats
   wire [31:0] Uimm={    C_INST[31],   C_INST[30:12], {12{1'b0}}};
   wire [31:0] Iimm={{21{C_INST[31]}}, C_INST[30:20]};
   wire [31:0] Simm={{21{C_INST[31]}}, C_INST[30:25],C_INST[11:7]};
   wire [31:0] Bimm={{20{C_INST[31]}}, C_INST[7],C_INST[30:25],C_INST[11:8],1'b0};
   wire [31:0] Jimm={{12{C_INST[31]}}, C_INST[19:12],C_INST[20],C_INST[30:21],1'b0};

  // Source and destination registers
   wire [4:0] rs1Id = C_INST[19:15]; // source reg 1 
   wire [4:0] rs2Id = C_INST[24:20]; // source reg 2
   wire [4:0] rdId  = C_INST[11:7]; // destenation reg

  // function codes
   wire [2:0] funct3 = C_INST[14:12]; // function to be performed on src 1 and 2 regs
   wire [6:0] funct7 = C_INST[31:25];

  //*********************************************************************************//
  //*********************************************************************************//


  //*********************************************************************************//
  //*********************************************************************************//
  // Registers File
  reg [31:0] rs1;
  reg [31:0] rs2;
  wire [31:0] writeBackData;
  wire        writeBackEn; 

    reg [31:0] RegisterFile [0:31] = // all is 0
    
    '{

    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[0] 
    32'b0000_0000_0000_0000_0000_0000_0000_0100,   // REG[1]
    32'b0000_0000_0000_0000_0000_0000_0000_0101,   // REG[2]
    32'b0000_0000_0000_0000_0000_0000_0000_0110,   // REG[3]
    32'b0000_0000_0000_0000_0000_0000_0000_0111,   // REG[4]
    32'b0000_0000_0000_0000_0000_0000_0000_1000,   // REG[5]
    32'b0000_0000_0000_0000_0000_0000_0000_1001,   // REG[6]
    32'b0000_0000_0000_0000_0000_0000_0000_1010,   // REG[7]
    32'b0000_0000_0000_0000_0000_0000_0000_1011,   // REG[8]
    32'b0000_0000_0000_0000_0000_0000_0000_1100,   // REG[9]
    32'b0000_0000_0000_0000_0000_0000_0000_1101,   // REG[10]
    32'b0000_0000_0000_0000_0000_0000_0000_1110,   // REG[11]
    32'b0000_0000_0000_0000_0000_0000_0000_1111,   // REG[12]
    32'b0000_0000_0000_0000_0000_0000_0001_0000,   // REG[13]
    32'b0000_0000_0000_0000_0000_0000_0001_0001,   // REG[14]
    32'b0000_0000_0000_0000_0000_0000_0001_0010,   // REG[15]
    32'b0000_0000_0000_0000_0000_0000_0001_0011,   // REG[16]
    32'b0000_0000_0000_0000_0000_0000_0001_0100,   // REG[17]
    32'b0000_0000_0000_0000_0000_0000_0001_0101,   // REG[18]
    32'b0000_0000_0000_0000_0000_0000_0001_0110,   // REG[19]
    32'b0000_0000_0000_0000_0000_0000_0001_0111,   // REG[20]
    32'b0000_0000_0000_0000_0000_0000_0001_1000,   // REG[21]
    32'b0000_0000_0000_0000_0000_0000_0001_1001,   // REG[22]
    32'b0000_0000_0000_0000_0000_0000_0001_1010,   // REG[23]
    32'b0000_0000_0000_0000_0000_0000_0001_1011,   // REG[24]
    32'b0000_0000_0000_0000_0000_0000_0001_1100,   // REG[25]
    32'b0000_0000_0000_0000_0000_0000_0001_1101,   // REG[26]
    32'b0000_0000_0000_0000_0000_0000_0001_1110,   // REG[27]
    32'b0000_0000_0000_0000_0000_0000_0001_1111,   // REG[28]
    32'b0000_0000_0000_0000_0000_0000_0010_0000,   // REG[29]
    32'b0000_0000_0000_0000_0000_0000_0010_0001,   // REG[30]
    32'b0000_0000_0000_0000_0000_0000_0010_0010    // REG[31]

    };
    

  //*********************************************************************************//
  //*********************************************************************************//




  //*********************************************************************************//
  //*********************************************************************************//
  // The ALU
   wire [31:0] aluIn1 = rs1;
   wire [31:0] aluIn2 = isALUreg | isBranch ? rs2 : Iimm;
   reg [31:0] aluOut;
   wire [4:0] shamt = isALUreg ? rs2[4:0] : C_INST[24:20]; // shift amount
   //the shift amount is either the content of rs2 for ALUreg instructions or instr[24:20] (the same bits as rs2Id) for ALUimm instructions.


  //*********************************************************************************//
   // from this point the most cruicial thing to think of is the fact that all operations are done as (AluIn1 - AluIn2) if the most sig bit is 1 means 
   // AluIn1 is less than AluIn2 if , MSB was 0 it means that AluIn1 is bigger

   // the line below is like saying [ wire [32:0] aluMinus = aluIn1 - aluIn2 ]
   wire [32:0] aluMinus = {1'b1, ~aluIn2} + {1'b0, aluIn1} + 33'b1;

          wire aluin1EQ2aluin2 = (aluMinus[31:0] == 0); // this is gonna be 1 if the subtraction results in 0
          wire aluin1LessThanUaluin2 = aluMinus[32]; // this gonna be 1 if the bit 33 of aluMinus is 1 ie (aluin1 < aluin2) signed

          wire aluin1LessThanaluin2 = (aluIn1[31] ^ aluIn2[31]) ? aluIn1[31] : aluMinus[32]; // ^ = XOR, so if 1 ^ 0 = ~0 = 1 , 1^1 = ~1 = 0.
          // so here it takes the MSB of the aluin1 and aluin2 (sign bits), so if they were having the same sign means:
          // 1 and 1 or 0 and 0 -> 1 ^ 1 = ~1 = 0. therfore the output would depend on the sign of aluMinus[32] (aluin1 - aluin2)
          // so if aluMinus[32] was 1 (negative) -> {subtraction result(aluin1 - aluin2) was negative}, thefore condition is true aluin1 is less than aluin2
          // 1 and 0 or 0 and 1 -> 1 ^ 0 = ~0 = 1. therfore the output would depend on the sign of aluin1 which from the equation we got in this line was 1 (aluin1 negative) hence the condition is true
          // and aluin1 is less than aluin2.


   wire [31:0] aluPlus = aluIn1 + aluIn2;

   // Flip a 32 bit word. Used by the shifter (a single shifter for
   // left and right shifts, saves silicium !)
   // functionality: takes a 32 bit input and puts the LSB in the MSB position and so on.....
   function [31:0] flip32;
      input [31:0] x;
      flip32 = {x[ 0], x[ 1], x[ 2], x[ 3], x[ 4], x[ 5], x[ 6], x[ 7], 
		x[ 8], x[ 9], x[10], x[11], x[12], x[13], x[14], x[15], 
		x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23],
		x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31]};
   endfunction

   // shifters
   wire [31:0] shifter_in = (funct3 == 3'b001) ? flip32(aluIn1) : aluIn1; 
   wire [31:0] shifter = $signed({C_INST[30] & aluIn1[31], shifter_in}) >>> aluIn2[4:0]; 
   wire [31:0] leftshift = flip32(shifter);
  // Example:
  //aluIn1 = 32'b00000000000000000000000000001111 (15 in decimal)
  //aluIn2 = 32'b00000000000000000000000000000010 (2 in decimal)
  //funct3 = 3'b001
  //C_INST = 32'b00000000000000000000000000000000

  //LSB becomes MSB, second LSB becomes second MSB, etc.:
  //flip32(aluIn1) = 32'b11110000000000000000000000000000
  // >>> means arithmatic right shift , hence -> wire [31:0] shifter = $signed({C_INST[30] & aluIn1[31], shifter_in}) >>> aluIn2[4:0];
  // {C_INST[30] & aluIn1[31], shifter_in} -> C_INST[30] & aluIn1[31] -> aluIn[31] = 0 , therfore C_INST[30] & aluIn1[31] = 0
  //  33-bit number -> (sign bit + 31 bits of shifter_in) and shift right by aluIn2[4:0] (which is 2)
  // {0, shifter_in} = 33'b0_11110000000000000000000000000000
  // Right shift by 2: shifter = 33'b0_0011110000000000000000000000000 = 32'b00111100000000000000000000000000

  //shifter = 32'b00111100000000000000000000000000
  //leftshift = flip32(shifter) = 32'b000000000000000000000000111100




  //*********************************************************************************//



   // ADD/SUB/ADDI: 
   // funct7[5] is 1 for SUB and 0 for ADD. We need also to test instr[5]
   // to make the difference with ADDI
   //
   // SRLI/SRAI/SRL/SRA: 
   // funct7[5] is 1 for arithmetic shift (SRA/SRAI) and 
   // 0 for logical shift (SRL/SRLI)
   always_comb begin
    case(funct3)

        3'b000: aluOut = (funct7[5] & C_INST[5]) ?  // C_INST[5] determines wether the instruction is immediate or ALUreg, funct7[5] determines wether if its ADD or SUB
            aluMinus[31:0] : aluPlus;

        3'b001: aluOut = leftshift; // shifts the ALU in1 (logically) to the left by the shifting ammount

        3'b010: aluOut = {31'b0, aluin1LessThanaluin2}; // signed comparison

        3'b011: aluOut = {31'b0, aluin1LessThanUaluin2}; // unsigned comparison

        3'b100: aluOut = (aluIn1 ^ aluIn2); // XOR

        3'b101: aluOut = shifter; // for logical or arithmetic right shift, by testing bit 5 of funct7 we determine which function to use, 1 for arithmetic shift (with sign expansion) and 0 for logical shift.

        3'b110: aluOut = (aluIn1 | aluIn2); // OR

        3'b111: aluOut = (aluIn1 & aluIn2);	 // AND

    endcase
   end

reg Branch;
   always_comb begin
    case (funct3)

      3'b000: Branch = aluin1EQ2aluin2; // if source reg 1  == source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

      3'b001: Branch = !aluin1EQ2aluin2; // if source reg 1  != source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

      3'b100: Branch = aluin1LessThanaluin2; // if signed source reg 1  < signed source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

      3'b101: Branch = !aluin1LessThanaluin2; // if signed source reg 1  >= signed source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

      3'b110: Branch = aluin1LessThanUaluin2; // if source reg 1  < source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.
        
      3'b111: Branch = !aluin1LessThanUaluin2; // if  source reg 1  >  source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

      default: Branch = 1'b0; // Branch is set to 0 as default
    endcase
   end

/*
   always_comb begin
    case (funct3)

      3'b000: aluOut = (funct7[5] & C_INST[5]) ?  // C_INST[5] determines wether the instruction is immediate or ALUreg, funct7[5] determines wether if its ADD or SUB
          (aluIn1 - aluIn2) : (aluIn1 + aluIn2);
      3'b000: Branch = (rs1 == rs2); // if source reg 1  == source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

      3'b001: aluOut = aluIn1 << shamt; // shifts the ALU in1 (logically) to the left by the shifting ammount
      3'b001: Branch = (rs1 != rs2); // if source reg 1  != source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

      3'b010: aluOut = ($signed(aluIn1) < $signed(aluIn2)); // signed comparison
      3'b100: Branch = ($signed(rs1) < $signed(rs2)); // if signed source reg 1  < signed source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

      3'b011: aluOut = (aluIn1 < aluIn2); // unsigned comparison
      3'b101: Branch = ($signed(rs1) >= $signed(rs2)); // if signed source reg 1  >= signed source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

      3'b100: aluOut = (aluIn1 ^ aluIn2); // XOR
      3'b110: Branch = (rs1 < rs2); // if source reg 1  < source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.
        
      3'b101: aluOut = funct7[5]? ($signed(aluIn1) >>> shamt) : 
          ($signed(aluIn1) >> shamt); // for logical or arithmetic right shift, by testing bit 5 of funct7 we determine which function to use, 1 for arithmetic shift (with sign expansion) and 0 for logical shift.
      3'b111: Branch = (rs1 > rs2=); // if  source reg 1  >  source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

      3'b111: aluOut = (aluIn1 & aluIn2);	 // AND

      default: Branch = 1'b0;
    endcase
   end


*/

  //*********************************************************************************//
  //*********************************************************************************//

  wire [31:0] PCplusImm = PC + (C_INST[3] ? Jimm[31:0] :
                                C_INST[4] ? Uimm[31:0] :
                                           Bimm[31:0] );
                                          
  wire [31:0] PCplus4 = PC + 4;


  localparam FETCH_INSTR = 0; // first state
  localparam WAIT_INSTR = 1;
  localparam FETCH_REGS = 2; // second state
  localparam EXECUTE = 3; // third state
  localparam LOAD = 4;
  localparam WAIT_DATA = 5;
  localparam STORE = 6;


  reg [2:0] state = FETCH_INSTR; // startsd at fetching  instructions

  // register write back
  assign writeBackData = (isJAL || isJALR ) ? (PCplus4) :
                         (isLUI) ? Uimm :
                         (isAUIPC) ? (PCplusImm) :
                          aluOut; // output of the ALU is assigned to writeback data , which hence will be written into the Reg File
  /*
  assign writeBackEn = (state == EXECUTE && 
                        
                        (isALUreg ||
                         isALUimm ||
                         isJAL    ||
                         isJALR   ||
                         isLUI    ||
                         isAUIPC  
                         )   

                       );  // the writting back enable signal, depends on : 1 : we at EXECUTE state
                           //                                               2 : the instruction wich state is 1

  */
  assign writeBackEn = (state == EXECUTE && !isBranch && !isStore && !isLoad) || (state == WAIT_DATA);


  //now the next PC "program counter" will be dependant on the on the instruction wether if it is JAL (set nextPC = PC + immed ) and (set rd = PC + 4 ) 
  // JALR (set nextPC = rs1 + immed) and (rd = PC + 4)
  // Branch (set nextPC = PC + Bimmed)
  // if the instruction was none of the jumping instruction then just increase the PC as normal (nextPC = PC + 4)
  wire [31:0] nextPC = ((isBranch && Branch) || isJAL)? PCplusImm: // (PC + immed)
                       (isJALR) ?  {aluPlus[31:1], 1'b0}: // (rs1 + immed) --> aluIn2 = isALUreg | isBranch ? rs2 : Iimm; , therfore aluPlus = rs1 + Iimm since it is JALR it is Iimm
                        PCplus4;

  // loading registers with memmory contents, the memmory accessing is done
  // by accessing a byte , halfword  or a word. we diffrentiate using funct3[1:0] :
  // - funct3[1:0]:  00->byte 01->halfword 10->word

  // address of memmory to be accessed
  wire [31:0] LoadNstore_addr = rs1 + (isStore? Simm : Iimm); // the adress of the instruction in memmory to be loaded into a reg

  // Decoding type of data to be obtained from memmory 
  wire MEM_byteA = funct3[1:0] == 2'b00; // Decoding the current C_INST, and checking how to get data from memmory (Byte, Half Word) 
  wire MEM_halfwordA = funct3[1:0] == 2'b01; // so if funct3[1:0] = 2'b00, it means we want to get data from memmory as bytes, if 2'b01, it means we want data as half a word.

  // routing memmory blocks to diffrent wires and signals
  wire [15:0] Load_halfword = LoadNstore_addr[1] ? MEM_dout[31:16] : MEM_dout[15:0]; // if loadNstore_addr[1] = 0 , Load_halfword = MEM_dout[15:0];, which is the first half word of the instruction
  wire [7:0] Load_byte = LoadNstore_addr[0] ? Load_halfword[15:8] : Load_halfword[7:0]; // therfore lets say loadNstore_addr[0] = 1 , Load_byte = Load_halfword[15:8], which is the 2nd byte of the instruction

  // since load has sign expansion features, one need to create a logic for that:
  // if funct3[2] =  0: do sign expansion ,  1: no sign expansion
  // Exapmle : take the number -1 -> 8'b11111111, loading it in a 32-bit register with
  //LBU will result in 32'b0000000000000000000000011111111, whereas loading it with LB will result in 32'b11111111111111111111111111111111,
  wire Load_sign = !funct3[2] & (MEM_byteA ? Load_byte[7] : Load_halfword[15]); // & !funct3[2] with either the MSB of the byte if the load was byte accessible otherwise & with the MSB of the Half word. 

  // the reg in wich the data to be loaded into
  wire [31:0] Load_data = MEM_byteA ? {{24{Load_sign}}, Load_byte} : // if the decoding funct3[1:0] indicates to load byte get it from Load_byte signal / if funct3[2] is 0 load_sign will concatinate the 24 bits with 1s else 0s 
                          MEM_halfwordA ? {{16{Load_sign}}, Load_halfword} : // if the decoding funct3[1:0] indicates to load half a word get it from Load_halfword signal / if funct3[2] is 0 load_sign will concatinate the 16 bits with 1s else 0s
                          MEM_dout; // otherwise load the whole word


  // Store----
  assign MEM_Wdata[7:0] = rs2[7:0];
  assign MEM_Wdata[15:8] = loadNstore_addr[0]? rs2[7:0] : rs2[15:8];
  assign MEM_Wdata[23:16] = loadNstore_addr[1]? rs2[7:0] : rs2[23:16];
  assign MEM_Wdata[31:24] = loadNstore_addr[0]? rs2[7:0] : loadNstore_addr[1]? rs2[15:8] : rs2[31:24];


  wire [31:0] Store_mask =      MEM_byteA ?
                                        (loadNstore_addr[1]?

                                              (loadNstore_addr[0]? 4'b1000 : 4'b0100) : (loadNstore_addr[0]? 4'b0010 : 4'b0001)

                                        );
                                MEM_halfwordA?
                                        (loadNstore_addr[1]? 4'b1100 : 4'b0011) : (4'b1111);
                          

  // sequential logic based on clock edges
  always_ff @(posedge clk, negedge resetn) begin
      if (!resetn) begin 
        PC <= 0;
        state <= FETCH_INSTR;
        //C_INST <= 32'b0000000_00000_00000_000_00000_0110011; // NOP
      end

      //************************//
      else begin // the writting logic
        if (writeBackEn && rdId != 0) begin // these signals are not set as they need they need an ALU into the module
          RegisterFile[rdId] <= writeBackData;

		if(rdId != 0) begin // disblays contents of reg[1] on leds
	              x10 <= writeBackData;
	              end
        end
     //************************//
              case(state)

              FETCH_INSTR: begin
               // C_INST <= MEM[PC[31:2]]; // assign the instruction in which the program counter is currently pointing towards
                state <=  WAIT_INSTR; // go to the next state
              end

              WAIT_INSTR: begin
                C_INST <= MEM_dout;
                state <= FETCH_REGS;
              end

              FETCH_REGS: begin
                rs1 <= RegisterFile[rs1Id]; // get the first source register id 
                rs2 <= RegisterFile[rs2Id]; // get the second source register // theses will probably need some sort of a program counter to keep changing
                state <= EXECUTE; // jump to the next state
              end

              EXECUTE: begin // this part will enable the writing back enable signal
                if(!isSYSTEM) begin
                PC <= nextPC; // increase the program counter (based on instruction type) ie go to the next instruction in RAM
                end

                else if(isSYSTEM) begin
                PC <= nextPC;
                //C_INST <= EBREAK(); // NOP // may not be needed !!!!!!
                end
                state <= FETCH_INSTR; 
              end

              LOAD: begin
                state <= WAIT_DATA;
              end

              WAIT_DATA: begin
                state <= FETCH_INSTR;
              end

              STORE: begin
                state <= FETCH_INSTR;
              end
   
              endcase
      end
  end

  assign MEM_addr = (state == WAIT_INSTR || state == FETCH_INSTR)?  PC : LoadNstore_addr;


  assign rMEM_en = (state == FETCH_INSTR || state == LOAD);

  assign MEM_Wmask = {4{(state == STORE)}} & Store_mask;

/*
  clk_divider #(.SLOW(2))
  clk_divider (

     .CLK(CLK),
     .RESET(RESET),
     .clk(clk),
     .resetn(resetn)

    );
*/


  // assigning the count to the leds
  //assign LEDS = (isSYSTEM) ? 16 : {PC[0],isALUimm,isStore,isLoad,isALUreg};
 //assign TXD = 1'b0;
    
endmodule
