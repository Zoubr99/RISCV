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

    reg [31:0] RegisterFile [0:31] = '{

    32'b0000_0000_0000_0000_0000_0000_0000_0011,   // REG[0] 
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


  //*********************************************************************************//
  //*********************************************************************************//


  localparam FETCH_INSTR = 0; // first state
  localparam FETCH_REGS = 1; // second state
  localparam EXECUTE = 2; // third state

  reg [1:0] state = FETCH_INSTR; // startsd at fetching  instructions

  // register write back
  assign writeBackData = aluOut; // output of the ALU is assigned to writeback data , which hence will be written into the Reg File
  assign writeBackEn = (state == EXECUTE && (isALUreg || isALUimm));  // the writting back enable signal, depends on : 1 : we at EXECUTE state
                                                                       //                                               2 : the instruction si either ALUreg or Imm


  // sequential logic based on clock edges
  always_ff @(posedge clk, negedge resetn) begin
      if (!resetn) begin 
        PC <= 0;
        state <= FETCH_INSTR;
        C_INST <= 32'b0000000_00000_00000_000_00000_0110011; // NOP
      end
      else begin
        if (writeBackEn && rdId != 0) begin // these signals are not set as they need they need an ALU into the module
          RegisterFile[rdId] <= writeBackData;

          	    if(rdId == 1) begin
	              leds <= writeBackData;
	              end
        end

              case(state)

              FETCH_INSTR: begin
                C_INST <= MEM[PC]; // assign the instruction in which the program counter is currently pointing towards
                state <=  FETCH_REGS; // go to the next state
              end

              FETCH_REGS: begin
                rs1 <= RegisterFile[rs1Id]; // get the first source register id 
                rs2 <= RegisterFile[rs2Id]; // get the second source register // theses will probably need some sort of a program counter to keep changing
                state <= EXECUTE; // jump to the next state
              end

              EXECUTE: begin
                if(!isSYSTEM) begin
                PC <= PC + 1; // increase the program counter ie go to the next instruction in RAM
                end
                else if(isSYSTEM) begin
                PC <= 0;
                C_INST <= 32'b0000000_00000_00000_000_00000_0110011; // NOP
                end
                state <= FETCH_INSTR;
              end
            
              endcase
      end
  end

  clk_divider #(.SLOW(2))
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