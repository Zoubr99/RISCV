//`include "clk_divider.sv"

module Ins_Memmory(
    input logic CLK,
    input logic clk, 
    input logic [31:0] MEM_addr,
    output logic [31:0] MEM_dout,
    input logic rMEM_en,
    input logic [31:0] MEM_Wdata,
    input logic [3:0] MEM_Wmask
);
    
    reg [31:0] MEM [0:255];

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
  //*************************************************************************\\
 //***************************************************************************\\ 
//*****************************************************************************\\


  /*
 * A simple assembler for RiscV written in VERILOG.
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
//    `include "riscv_assembly.v" // yes, needs to be included from here.
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

integer memPC = 0;
//initial memPC = 0;

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


// add x0,x0,x0   
localparam [31:0] NOP_CODEOP = 32'b0000000_00000_00000_000_00000_0110011; 

/***************************************************************************/

/*
 * R-Type instructions.
 * rd <- rs1 OP rs2
 */

task RType;
   input [6:0] opcode;
   input [4:0] rd;   
   input [4:0] rs1;
   input [4:0] rs2;
   input [2:0] funct3;
   input [6:0] funct7;
   begin
      MEM[memPC[31:2]] = {funct7, rs2, rs1, funct3, rd, opcode};
      memPC = memPC + 4;
   end
endtask

task ADD;
   input [4:0] rd;
   input [4:0] rs1;
   input [4:0] rs2;
   RType(7'b0110011, rd, rs1, rs2, 3'b000, 7'b0000000);
endtask
   
task SUB;
   input [4:0] rd;
   input [4:0] rs1;
   input [4:0] rs2;
   RType(7'b0110011, rd, rs1, rs2, 3'b000, 7'b0100000);
endtask

task SLL;
   input [4:0] rd;
   input [4:0] rs1;
   input [4:0] rs2;
   RType(7'b0110011, rd, rs1, rs2, 3'b001, 7'b0000000);
endtask

task SLT;
   input [4:0] rd;
   input [4:0] rs1;
   input [4:0] rs2;
   RType(7'b0110011, rd, rs1, rs2, 3'b010, 7'b0000000);
endtask
   
task SLTU;
   input [4:0] rd;
   input [4:0] rs1;
   input [4:0] rs2;
   RType(7'b0110011, rd, rs1, rs2, 3'b011, 7'b0000000);
endtask

task XOR;
   input [4:0] rd;
   input [4:0] rs1;
   input [4:0] rs2;
   RType(7'b0110011, rd, rs1, rs2, 3'b100, 7'b0000000);
endtask

task SRL;
   input [4:0] rd;
   input [4:0] rs1;
   input [4:0] rs2;
   RType(7'b0110011, rd, rs1, rs2, 3'b101, 7'b0000000);
endtask

task SRA;
   input [4:0] rd;
   input [4:0] rs1;
   input [4:0] rs2;
   RType(7'b0110011, rd, rs1, rs2, 3'b101, 7'b0100000);
endtask

task OR;
   input [4:0] rd;
   input [4:0] rs1;
   input [4:0] rs2;
   RType(7'b0110011, rd, rs1, rs2, 3'b110, 7'b0000000);
endtask

task AND;
   input [4:0] rd;
   input [4:0] rs1;
   input [4:0] rs2;
   RType(7'b0110011, rd, rs1, rs2, 3'b111, 7'b0000000);
endtask

/***************************************************************************/

/*
 * I-Type instructions.
 * rd <- rs1 OP imm
 */
   
task IType;
   input [6:0]  opcode;
   input [4:0]  rd;   
   input [4:0]  rs1;
   input [31:0] imm;
   input [2:0]  funct3;
   begin
      MEM[memPC[31:2]] = {imm[11:0], rs1, funct3, rd, opcode};
      memPC = memPC + 4;
   end
endtask

task ADDI;
   input [4:0]  rd;   
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      IType(7'b0010011, rd, rs1, imm, 3'b000);
   end
endtask

task SLTI;
   input [4:0]  rd;   
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      IType(7'b0010011, rd, rs1, imm, 3'b010);
   end
endtask

task SLTIU;
   input [4:0]  rd;   
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      IType(7'b0010011, rd, rs1, imm, 3'b011);
   end
endtask

task XORI;
   input [4:0]  rd;   
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      IType(7'b0010011, rd, rs1, imm, 3'b100);
   end
endtask

task ORI;
   input [4:0]  rd;   
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      IType(7'b0010011, rd, rs1, imm, 3'b110);
   end
endtask

task ANDI;
   input [4:0]  rd;   
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      IType(7'b0010011, rd, rs1, imm, 3'b111);
   end
endtask

// The three shifts, SLLI, SRLI, SRAI, encoded in RType format
// (rs2 is replaced with shift amount=imm[4:0])   
   
task SLLI;
   input [4:0]  rd;   
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      RType(7'b0010011, rd, rs1, imm[4:0], 3'b001, 7'b0000000);
   end
endtask

task SRLI;
   input [4:0]  rd;   
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      RType(7'b0010011, rd, rs1, imm[4:0], 3'b101, 7'b0000000);
   end
endtask

task SRAI;
   input [4:0]  rd;   
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      RType(7'b0010011, rd, rs1, imm[4:0], 3'b101, 7'b0100000);
   end
endtask

/***************************************************************************/

/*
 * Jumps (JAL and JALR)
 */

task JType;
   input [6:0]  opcode;
   input [4:0]  rd;
   input [31:0] imm;
   begin
      MEM[memPC[31:2]] = {imm[20], imm[10:1], imm[11], imm[19:12], rd, opcode};
      memPC = memPC + 4;
   end
endtask
   
task JAL;
   input [4:0] rd;
   input [31:0] imm;
   begin
      JType(7'b1101111, rd, imm);
   end
endtask 

// JALR is encoded in the IType format.
   
task JALR;
   input [4:0] rd;
   input [4:0] rs1;
   input [31:0] imm;
   begin
      IType(7'b1100111, rd, rs1, imm, 3'b000);
   end
endtask   

/***************************************************************************/   

/*
 * Branch instructions.
 */    
   
task BType;
   input [6:0]  opcode;
   input [4:0]  rs1;
   input [4:0]  rs2;   
   input [31:0] imm;
   input [2:0]  funct3;
   begin
      MEM[memPC[31:2]] = {imm[12],imm[10:5], rs2, rs1, funct3, imm[4:1], imm[11], opcode};
      memPC = memPC + 4;
   end
endtask

task BEQ;
   input [4:0]  rs1;
   input [4:0]  rs2;   
   input [31:0] imm;
   begin
      BType(7'b1100011, rs1, rs2, imm, 3'b000);
   end
endtask

task BNE;
   input [4:0]  rs1;
   input [4:0]  rs2;   
   input [31:0] imm;
   begin
      BType(7'b1100011, rs1, rs2, imm, 3'b001);
   end
endtask

task BLT;
   input [4:0]  rs1;
   input [4:0]  rs2;   
   input [31:0] imm;
   begin
      BType(7'b1100011, rs1, rs2, imm, 3'b100);
   end
endtask

task BGE;
   input [4:0]  rs1;
   input [4:0]  rs2;   
   input [31:0] imm;
   begin
      BType(7'b1100011, rs1, rs2, imm, 3'b101);
   end
endtask

task BLTU;
   input [4:0]  rs1;
   input [4:0]  rs2;   
   input [31:0] imm;
   begin
      BType(7'b1100011, rs1, rs2, imm, 3'b110);
   end
endtask

task BGEU;
   input [4:0]  rs1;
   input [4:0]  rs2;   
   input [31:0] imm;
   begin
      BType(7'b1100011, rs1, rs2, imm, 3'b111);
   end
endtask
   
/***************************************************************************/   

/*
 * LUI and AUIPC
 */    

task UType;
   input [6:0]  opcode;
   input [4:0]  rd;
   input [31:0] imm;
   begin
      MEM[memPC[31:2]] = {imm[31:12], rd, opcode};
      memPC = memPC + 4;
   end
endtask

task LUI;
   input [4:0]  rd;
   input [31:0] imm;
   begin
      UType(7'b0110111, rd, imm);
   end
endtask

task AUIPC;
   input [4:0]  rd;
   input [31:0] imm;
   begin
      UType(7'b0010111, rd, imm);
   end
endtask
   
/***************************************************************************/   

/*
 * Load instructions
 */    

task LB;
   input [4:0]  rd;
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      IType(7'b0000011, rd, rs1, imm, 3'b000);
   end
endtask      

task LH;
   input [4:0]  rd;
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      IType(7'b0000011, rd, rs1, imm, 3'b001);
   end
endtask      
   
task LW;
   input [4:0]  rd;
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      IType(7'b0000011, rd, rs1, imm, 3'b010);
   end
endtask      

task LBU;
   input [4:0]  rd;
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      IType(7'b0000011, rd, rs1, imm, 3'b100);
   end
endtask      

task LHU;
   input [4:0]  rd;
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      IType(7'b0000011, rd, rs1, imm, 3'b101);
   end
endtask      
   
/***************************************************************************/   

/*
 * Store instructions
 */ 

task SType;
   input [6:0]  opcode;
   input [4:0]  rs1;
   input [4:0]  rs2;   
   input [31:0] imm;
   input [2:0]  funct3;
   begin
      MEM[memPC[31:2]] = {imm[11:5], rs2, rs1, funct3, imm[4:0], opcode};
      memPC = memPC + 4;
   end
endtask

// Note: in SB, SH, SW, rs1 and rs2 are swapped, to match assembly code:
// for instance:
//   
//     rs2   rs1   
//  sw ra, 0(sp)   
   
task SB;
   input [4:0]  rs1;
   input [4:0]  rs2;   
   input [31:0] imm;
   begin
      SType(7'b0100011, rs2, rs1, imm, 3'b000);
   end
endtask   

task SH;
   input [4:0]  rs1;
   input [4:0]  rs2;   
   input [31:0] imm;
   begin
      SType(7'b0100011, rs2, rs1, imm, 3'b001);
   end
endtask   

task SW;
   input [4:0]  rs1;
   input [4:0]  rs2;   
   input [31:0] imm;
   begin
      SType(7'b0100011, rs2, rs1, imm, 3'b010);
   end
endtask   
   
/***************************************************************************/   
   
/*
 * SYSTEM instructions
 */

task FENCE;
   input [3:0] pred;
   input [3:0] succ;
   begin
      MEM[memPC[31:2]] = {4'b0000, pred, succ, 5'b00000, 3'b000, 5'b00000, 7'b1110011};
      memPC = memPC + 4;
   end
endtask   

task FENCE_I;
   begin
      MEM[memPC[31:2]] = {4'b0000, 4'b0000, 4'b0000, 5'b00000, 3'b001, 5'b00000, 7'b1110011};
      memPC = memPC + 4;
   end
endtask   
   
task ECALL;
   begin
      MEM[memPC[31:2]] = {12'b000000000000, 5'b00000, 3'b000, 5'b00000, 7'b1110011};
      memPC = memPC + 4;
   end
endtask   
   
task EBREAK;
   begin
      MEM[memPC[31:2]] = {12'b000000000001, 5'b00000, 3'b000, 5'b00000, 7'b1110011};
      memPC = memPC + 4;
   end
endtask   

task CSRRW;
   input [4:0] rd;
   input [11:0] csr;
   input [4:0] rs1;
   begin
      MEM[memPC[31:2]] = {csr, rs1, 3'b001, rd, 7'b1110011};
      memPC = memPC + 4;      
   end
endtask

task CSRRS;
   input [4:0] rd;
   input [11:0] csr;
   input [4:0] rs1;
   begin
      MEM[memPC[31:2]] = {csr, rs1, 3'b010, rd, 7'b1110011};
      memPC = memPC + 4;      
   end
endtask

task CSRRC;
   input [4:0] rd;
   input [11:0] csr;   
   input [4:0] rs1;
   begin
      MEM[memPC[31:2]] = {csr, rs1, 3'b011, rd, 7'b1110011};
      memPC = memPC + 4;      
   end
endtask

task CSRRWI;
   input [4:0] rd;
   input [11:0] csr;
   input [31:0] imm;
   begin
      MEM[memPC[31:2]] = {csr, imm[4:0], 3'b101, rd, 7'b1110011};
      memPC = memPC + 4;      
   end
endtask

task CSRRSI;
   input [4:0] rd;
   input [11:0] csr;
   input [31:0] imm;
   begin
      MEM[memPC[31:2]] = {csr, imm[4:0], 3'b110, rd, 7'b1110011};
      memPC = memPC + 4;      
   end
endtask

task CSRRCI;
   input [4:0] rd;
   input [11:0] csr;
   input [31:0] imm;
   begin
      MEM[memPC[31:2]] = {csr, imm[4:0], 3'b111, rd, 7'b1110011};
      memPC = memPC + 4;      
   end
endtask
   
/***************************************************************************/   

/*
 * Labels.
 * Example of usage:
 *  
 *                   ADD(x1,x0,x0);
 *       Label(L0_); ADDI(x1,x1,1); 
 *                   JAL(x0, LabelRef(L0_));
 */

   integer ASMerror=0;
   
   task Label;
      inout integer L;
      begin
`ifdef BENCH
	if(L[0] === 1'bx) begin
	   $display("Missing label initialization");
	   ASMerror = 1;
	end else if(L != memPC) begin
	   $display("Incorrect label initialization");
	   $display("Expected: %0d    Got: %0d",memPC,L);
	   ASMerror = 1;
	end
	$display("Label:",memPC);
`endif	 
     end
   endtask

   function [31:0] LabelRef;
      input integer L;
      begin
`ifdef BENCH
	if(L[0] === 1'bx) begin
	   $display("Reference to uninitialized label");
	   ASMerror = 1;
	end
`endif	 
	 LabelRef = L - memPC;
      end
   endfunction

   task endASM;
      begin
`ifdef GET_ASM_LABELS
	 $finish();
`endif	 
`ifdef BENCH  
	 if(ASMerror) $finish();
`endif
      end
   endtask
   
   
/****************************************************************************/

/*
 * RISC-V ABI register names.
 */    

   localparam zero = x0;
   localparam ra   = x1;
   localparam sp   = x2;
   localparam gp   = x3;
   localparam tp   = x4;
   localparam t0   = x5;
   localparam t1   = x6;
   localparam t2   = x7;
   localparam fp   = x8;
   localparam s0   = x8;
   localparam s1   = x9;
   localparam a0   = x10;
   localparam a1   = x11;
   localparam a2   = x12;
   localparam a3   = x13;
   localparam a4   = x14;
   localparam a5   = x15;
   localparam a6   = x16;
   localparam a7   = x17;
   localparam s2   = x18;
   localparam s3   = x19;
   localparam s4   = x20;
   localparam s5   = x21;
   localparam s6   = x22;
   localparam s7   = x23;
   localparam s8   = x24;
   localparam s9   = x25;
   localparam s10  = x26;
   localparam s11  = x27;
   localparam t3   = x28;
   localparam t4   = x29;
   localparam t5   = x30;      
   localparam t6   = x31;   

/*
 * RISC-V pseudo-instructions
 */

task NOP;
   begin
      ADD(x0,x0,x0);
   end
endtask

// See https://stackoverflow.com/questions/50742420/
// risc-v-build-32-bit-constants-with-lui-and-addi
// Add imm[11] << 12 to the constant passed to LUI,
// so that it cancels sign expansion done by ADDI
// if imm[11] is 1.
task LI;
   input [4:0]  rd;
   input [31:0] imm;
   begin
      if(imm == 0) begin
	 ADD(rd,zero,zero);
      end else if($signed(imm) >= -2048 && $signed(imm) < 2048) begin
	 ADDI(rd,zero,imm);
      end else begin
	 LUI(rd,imm + (imm[11] << 12)); // cancel sign expansion
	 if(imm[11:0] != 0) begin
	    ADDI(rd,rd,imm[11:0]);
	 end
      end
   end
endtask

task CALL;
  input [31:0] offset;
  begin
     AUIPC(x6, offset);
     JALR(x1, x6, offset[11:0]);
  end
endtask 

task RET;
  begin
     JALR(x0,x1,0);
  end
endtask   

task MV;
   input [4:0]  rd;
   input [4:0] 	rs1;
   begin
      ADD(rd,rs1,zero);
   end
endtask

task J;
   input [31:0] imm;
   begin
      // TODO: far targets
      JAL(zero,imm);
   end
endtask

task JR;
   input [4:0]  rs1;
   input [31:0] imm;   
   begin
      JALR(zero,rs1,imm);
   end
endtask
   
task BEQZ;
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      BEQ(rs1,x0,imm);
   end
endtask

task BNEZ;
   input [4:0]  rs1;
   input [31:0] imm;
   begin
      BNE(rs1,x0,imm);
   end
endtask

task BGT;
   input [4:0]  rs1;
   input [4:0]  rs2;   
   input [31:0] imm;
   begin
      BLT(rs2,rs1,imm);
   end
endtask

task DATAW;
   input [31:0] w;
   begin
      MEM[memPC[31:2]] = w;
      memPC = memPC+4;
   end
endtask

task DATAB;
   input [7:0] b1;
   input [7:0] b2;
   input [7:0] b3;
   input [7:0] b4;   
   begin
      MEM[memPC[31:2]][ 7: 0] = b1;
      MEM[memPC[31:2]][15: 8] = b2;
      MEM[memPC[31:2]][23:16] = b3;
      MEM[memPC[31:2]][31:24] = b4;            
      memPC = memPC+4;
   end
endtask

      
   
  //*************************************************************************\\
 //***************************************************************************\\ 
//*****************************************************************************\\
  logic prog_i = 0;
  
    wire [29:0] Word_addr = MEM_addr[31:2];

    localparam io_leds_bit = 0;
    localparam io_uartd_bit = 1;
    localparam io_uartc_bit = 2;


         function [31:0] io_bit_to_offset;
         input [31:0] bitid;
               begin
                  io_bit_to_offset = 1 << (bitid + 2);
               end   
         endfunction





  localparam slow_bit = 1;

   // these are labels just like the ones in C, goto: (LabelRef(L0_)) and label: label(L0_)
   // every 4 represents 32 bit word (instruction)
  integer L0_ = 12;
  integer L1_ = 20;
  integer L2_ = 52;
  integer wait_ = 104;
  integer wait_L0_ = 112;
  integer putc_ = 124;
  integer putc_L0_ = 132;
  

  always_ff @(posedge CLK) begin
  
    if(prog_i == 0)begin



    LI(sp,32'h1800);   // loading the stack pointer reg an address so it starts at end of RAM 6kB
    LI(gp,32'h400000); // IO page, global pointer


   // the code below are set of loops that keeps calling functions which are declared after the looping code
   // the looping code breaks at BREAK(); and will go back to beginning

  Label(L0_); // start of the program loop
    

   // LOOP 1

    // initilising loop boundries ie like saying (int i = 0, i < n , ++i)
    // count from 0 to 15 on the LEDs
    LI(s0,16); // upper bound of loop
    LI(a0,0);
  Label(L1_);
    SW(a0,gp,io_bit_to_offset(io_leds_bit)); // store word into  
   //JAL(x1, LabelRef(wait_)); // call(wait_) //12 , calling a function
    CALL(LabelRef(wait_));
    ADDI(a0,a0,1);
    BNE(a0,s0,LabelRef(L1_));



   // LOOP 2
    // initilising loop boundries ie like saying (int i = 0, i < n , ++i)
    // count from 0 to 26 on the LEDs
    // add 1 to the ascii chars each go, to send chars from A up to Z
    LI(s0,26); // upper bound of loop
    LI(a0,"a");
    LI(s1,0);
  Label(L2_);
    CALL(LabelRef(putc_)); // function calling
    ADDI(a0,a0,1);
    ADDI(s1,s1,1);
    BNE(s1,s0,LabelRef(L2_));

      // CR;LF, not clear what this code is for could be for terminal settings
      LI(a0,13);
      CALL(LabelRef(putc_));
      LI(a0,10);
      CALL(LabelRef(putc_));

    //JAL(zero, LabelRef(L0_)); // jump(L0_) , jumps back to the beginning of the program
    J(LabelRef(L0_));
    EBREAK();

    // functions that get called while in the loop

  Label(wait_);
    //ADDI(x11,x0,1); 
    LI(t0,1);
    SLLI(t0,t0,slow_bit); 
  Label(wait_L0_);
    ADDI(t0,t0,-1); 
    //BNE(x11,x0, LabelRef(wait_L0_));
    BNEZ(t0,LabelRef(wait_L0_));
    //JALR(x0,x1,0);
    RET();
  
   Label(putc_);
      // Send character to UART
      SW(a0,gp,io_bit_to_offset(io_uartd_bit));
      // Read UART status, and loop until bit 9 (busy sending)
      // is zero.
      LI(t0,1<<9);
   Label(putc_L0_);
      LW(t1,gp,io_bit_to_offset(io_uartc_bit));     
      AND(t1,t1,t0);
      BNEZ(t1,LabelRef(putc_L0_));
      RET();


    endASM();
    
       prog_i = 1;
     end
    
    if(MEM_Wmask[0]) MEM[Word_addr][7:0] <= MEM_Wdata[7:0];
    
    if(MEM_Wmask[1]) MEM[Word_addr][15:8] <= MEM_Wdata[15:8];
    
    if(MEM_Wmask[2]) MEM[Word_addr][23:16] <= MEM_Wdata[23:16];

    if(MEM_Wmask[3]) MEM[Word_addr][31:24] <= MEM_Wdata[31:24]; 

    
    
  end
  /*
initial begin
    MEM[100] = {8'h4, 8'h3, 8'h2, 8'h1}; 
    MEM[101] = {8'h8, 8'h7, 8'h6, 8'h5};
    MEM[102] = {8'hc, 8'hb, 8'ha, 8'h9};
    MEM[103] = {8'hff, 8'hf, 8'he, 8'hd};
end
*/

//reg [31:0] tmp_MEM [0:255];

always_ff @(posedge clk) begin
    
    if (rMEM_en) begin
        MEM_dout <= MEM[Word_addr];
    end 
/*     
    if(MEM_Wmask[0])begin
     tmp_MEM[Word_addr][7:0] <= MEM_Wdata[7:0];
    end

    if(MEM_Wmask[1])begin
     tmp_MEM[Word_addr][15:8] <= MEM_Wdata[15:8];
    end
    
    if(MEM_Wmask[2])begin
     tmp_MEM[Word_addr][23:16] <= MEM_Wdata[23:16];
    end
    
    if(MEM_Wmask[3])begin
     tmp_MEM[Word_addr][31:24] <= MEM_Wdata[31:24]; 
    end
  */
end
/*
always_comb begin
 MEM[Word_addr] = tmp_MEM[Word_addr];
end
*/

endmodule
