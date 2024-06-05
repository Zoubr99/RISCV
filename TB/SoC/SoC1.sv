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


// declaring a 5 bit register
reg [31:0] MEM [0:255]; // BRAM
reg [31:0] PC = 0; // Program Counter



        // add x0, x0, x0
              //                   rs2   rs1  add  rd   ALUREG
reg [31:0] C_INST =  32'b0000000_00000_00000_000_00000_0110011; //cureent instruction reg

  // initiliasing the SoC Memmory with some instructions
   initial begin // this is only for TB simulation purposes
        // add x1, x0, x0
        //                    rs2   rs1  add  rd  ALUREG
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
        // lw x2,0(x1)
        //             imm         rs1   w   rd   LOAD
        MEM[5] = 32'b000000000000_00001_010_00010_0000011;
        // sw x2,0(x1)
        //             imm   rs2   rs1   w   imm  STORE
        MEM[6] = 32'b000000_00010_00001_010_00000_0100011;
        
        // ebreak
        //                                        SYSTEM
        MEM[7] = 32'b000000000001_00000_000_00000_1110011;
        
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


  // sequential logic based on clock edges
  always_ff @(posedge clk or negedge resetn) begin
      if (!resetn) begin 
        PC <= 0;
        C_INST <= 32'b0000000_00000_00000_000_00000_0110011; // NOP
      end
      else if (!isSYSTEM) begin
        C_INST <= MEM[PC]; // assign the instruction in which the program counter is currently pointing towards in the 
                          // memmory to the the current instruction register
        PC <= PC + 1; // increase the program counter ie go to the next instruction in RAM
      end  else if(isSYSTEM) begin
        PC <= 0;
        C_INST <= 32'b0000000_00000_00000_000_00000_0110011; // NOP
      end
  end

  clk_divider #(.SLOW(27))
  clk_divider (

     .CLK(CLK),
     .RESET(RESET),
     .clk(clk),
     .resetn(resetn)

    );



  // assigning the count to the leds
  assign LEDS = (isSYSTEM) ? 16 : {PC[0],isALUimm,isStore,isLoad,isALUreg};
  assign TXD = 1'b0;
  

    
endmodule