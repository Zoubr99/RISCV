`include "clk_divider.sv"
//`default_nettype none

module SOC( // declaring the inputs and outputs
    input logic CLK,
    input logic RESET,
    output logic [4:0] LEDS,
    input logic RXD, // UART receiver
    output logic TXD // UART Transmitter
);

wire clk;
wire resetn;

reg [4:0] leds;
assign LEDS = leds;

// declaring a 5 bit register
reg [31:0] MEM [0:255]; // BRAM
reg [31:0] PC = 0; // Program Counter
reg [31:0] C_INST;
 
        // add x0, x0, x0
              //                   rs2   rs1  add  rd   ALUREG
//reg [31:0] C_INST =  32'b0000000_00000_00000_000_00000_0110011; //cureent instruction reg

 /*
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
/*
 * A simple assembler for RiscV written in SYSTEMVERILOG.
 * See table page 104 of RiscV instruction manual.
 * Bruno Levy, March 2022
 */

// Machine code will be generated in MEM,
// starting from address 0 (can be changed below,
// initial value of memPC).
//
// Example:
//
// module MyModule( my inputs, my outputs ...);
//    ...
//    reg [31:0] MEM [0:255]; 
//    `include "riscv_assembly.sv" // yes, needs to be included from here.
//    integer L0_;
//    initial begin
//                  ADD(x1,x0,x0);
//                  ADDI(x2,x0,32);
//      Label(L0_); ADDI(x1,x1,1); 
//                  BNE(x1, x2, LabelRef(L0_));
//                  EBREAK();
//    end
//   1) simulate with icarus, it will complain about uninitialized labels,
//      and will display for each Label() statement the address to be used
//      (in the present case, it is 8)
//   2) replace the declaration of the label:
//      integer L0_ = 8;
//      re-simulate with icarus
//      If you made an error, it will be detected
//   3) synthesize with yosys
// (if you do not use labels, you can synthesize directly, of course...)
//
//
// You can change the address where code is generated
//   by assigning to memPC (needs to be a word boundary).
//
// NOTE: to be checked, LUI, AUIPC take as argument
//     pre-shifted constant, unlike in GNU assembly

reg [31:0] memPC = 0;

/***************************************************************************/

/*
 * Register names.
 * Looks stupid, but makes assembly code more legible (without it,
 * one does not make the difference between immediate values and 
 * register ids).
 */ 

localparam x0 = 0, x1 = 1, x2 = 2, x3 = 3, x4 = 4, x5 = 5, x6 = 6, x7 = 7, 
           x8 = 8, x9 = 9, x10=10, x11=11, x12=12, x13=13, x14=14, x15=15,
           x16=16, x17=17, x18=18, x19=19, x20=20, x21=21, x22=22, x23=23,
           x24=24, x25=25, x26=26, x27=27, x28=28, x29=29, x30=30, x31=31;

/***************************************************************************/

/*
 * R-Type instructions.
 * rd <- rs1 OP rs2
 */
    task RType(
      input logic [6:0] opcode,
      input logic [4:0] rd,  
      input logic [4:0] rs1,
      input logic [4:0] rs2,
      input logic [2:0] funct3,
      input logic [6:0] funct7
      );
      begin
          MEM[memPC[31:2]] = {funct7, rs2, rs1, funct3, rd, opcode};
          memPC = memPC + 4;
      end
    endtask

    task ADD(
      input logic [4:0] rd,
      input logic [4:0] rs1,
      input logic [4:0] rs2
      );
      RType(7'b0110011, rd, rs1, rs2, 3'b000, 7'b0000000);
    endtask



    task IType(
      input logic [6:0]  opcode,
      input logic [4:0]  rd,  
      input logic [4:0]  rs1,
      input logic [31:0] imm,
      input logic [2:0]  funct3
      );
      begin
          MEM[memPC[31:2]] = {imm[11:0], rs1, funct3, rd, opcode};
          memPC = memPC + 4;
      end
    endtask

    task ADDI(
      input logic [4:0]  rd,   
      input logic [4:0]  rs1,
      input logic [31:0] imm
      );
      begin
          IType(7'b0010011, rd, rs1, imm, 3'b000);
      end
    endtask
/***************************************************************************/

always_ff @(posedge CLK) begin
      ADD(x0,x0,x0);
      ADD(x1,x0,x0);
      ADDI(x1,x1,1);
      ADDI(x1,x1,1);
      ADDI(x1,x1,1);
      ADDI(x1,x1,1);
      ADD(x2,x1,x0);
      ADD(x3,x1,x2);
  end

/***************************************************************************/
/****************************************************************************/ 

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

    reg [31:0] RegisterFile [0:31]=// all is 0

    '{

    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[0] 
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[1]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[2]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[3]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[4]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[5]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[6]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[7]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[8]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[9]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[10]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[11]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[12]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[13]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[14]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[15]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[16]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[17]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[18]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[19]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[20]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[21]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[22]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[23]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[24]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[25]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[26]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[27]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[28]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[29]
    32'b0000_0000_0000_0000_0000_0000_0000_0000,   // REG[30]
    32'b0000_0000_0000_0000_0000_0000_0000_0000    // REG[31]

    };




  //*********************************************************************************//
  //*********************************************************************************//


  //*********************************************************************************//
  //*********************************************************************************//
  // The ALU
   wire [31:0] aluIn1 = rs1;
   wire [31:0] aluIn2 = isALUreg ? rs2 : Iimm;
   reg [31:0] aluOut;
   wire [4:0] shamt = isALUreg ? rs2[4:0] : C_INST[24:20]; // shift amount
   //the shift amount is either the content of rs2 for ALUreg instructions or instr[24:20] (the same bits as rs2Id) for ALUimm instructions.

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
            (aluIn1 - aluIn2) : (aluIn1 + aluIn2);

        3'b001: aluOut = aluIn1 << shamt; // shifts the ALU in1 (logically) to the left by the shifting ammount

        3'b010: aluOut = ($signed(aluIn1) < $signed(aluIn2)); // signed comparison

        3'b011: aluOut = (aluIn1 < aluIn2); // unsigned comparison

        3'b100: aluOut = (aluIn1 ^ aluIn2); // XOR

        3'b101: aluOut = funct7[5]? ($signed(aluIn1) >>> shamt) : 
            ($signed(aluIn1) >> shamt); // for logical or arithmetic right shift, by testing bit 5 of funct7 we determine which function to use, 1 for arithmetic shift (with sign expansion) and 0 for logical shift.

        3'b110: aluOut = (aluIn1 | aluIn2); // OR

        3'b111: aluOut = (aluIn1 & aluIn2);	 // AND

    endcase
   end

reg Branch;
   always_comb begin
    case (funct3)

      3'b000: Branch = (rs1 == rs2); // if source reg 1  == source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

      3'b001: Branch = (rs1 != rs2); // if source reg 1  != source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

      3'b100: Branch = ($signed(rs1) < $signed(rs2)); // if signed source reg 1  < signed source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

      3'b101: Branch = ($signed(rs1) >= $signed(rs2)); // if signed source reg 1  >= signed source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

      3'b110: Branch = (rs1 < rs2); // if source reg 1  < source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.
        
      3'b111: Branch = (rs1 >= rs2); // if  source reg 1  >  source reg 2 set Branch to 1, brnach to the (PC + immediate offset value), if (isBranch && Branch) were true.

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


  localparam FETCH_INSTR = 0; // first state
  localparam FETCH_REGS = 1; // second state
  localparam EXECUTE = 2; // third state

  reg [1:0] state = FETCH_INSTR; // startsd at fetching  instructions

  // register write back
  assign writeBackData = (isJAL || isJALR ) ? (PC + 4) :
                         (isLUI) ? Uimm :
                         (isAUIPC) ? (PC + Uimm) :
                          aluOut; // output of the ALU is assigned to writeback data , which hence will be written into the Reg File

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


  //now the next PC "program counter" will be dependant on the on the instruction wether if it is JAL (set nextPC = PC + immed ) and (set rd = PC + 4 ) 
  // JALR (set nextPC = rs1 + immed) and (rd = PC + 4)
  // Branch (set nextPC = PC + Bimmed)
  // if the instruction was none of the jumping instruction then just increase the PC as normal (nextPC = PC + 4)
  wire [31:0] nextPC = (isBranch && Branch)? PC + Bimm:
                       (isJAL) ? PC + Jimm:
                       (isJALR) ? rs1 + Iimm:
                        PC + 4;



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

          	    if(rdId == 1) begin // disblays contents of reg[1] on leds
	              leds <= writeBackData;
	              end
        end
     //************************//
              case(state)

              FETCH_INSTR: begin
                C_INST <= MEM[PC[31:2]]; // assign the instruction in which the program counter is currently pointing towards
                state <=  FETCH_REGS; // go to the next state
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
            
              endcase
      end
  end





  clk_divider #(.SLOW(26))
  clk_divider (

     .CLK(CLK),
     .RESET(RESET),
     .clk(clk),
     .resetn(resetn)

    );
   
  // assigning the count to the leds
  //assign LEDS = (isSYSTEM) ? 16 : {PC[0],isALUimm,isStore,isLoad,isALUreg};
  assign TXD = 1'b0;
    
endmodule