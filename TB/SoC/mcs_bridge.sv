module mcs_bridge

        // bridge core -> this core is an interface between the uBlaze processing core and the basic bus

   #(parameter BRG_BASE=32'hc000_0000)   // default base address
   (
    // RISCV Signals

    
    input  logic [31:0] io_address,
    output logic [31:0] io_read_data,
    input  logic io_read_strobe,
    input  logic [31:0] io_write_data,
    input  logic io_write_strobe,


/*
    // uBlaze MCS I/O bus
    input  logic io_addr_strobe,   // not used
    input  logic io_read_strobe, 
    input  logic io_write_strobe, 
    input  logic [3:0] io_byte_enable, 
    input  logic [31:0] io_address, 
    input  logic [31:0] io_write_data, 
    output logic [31:0] io_read_data, 
    output logic io_ready, 
*/


    // Basic bus - b indicates Basic
    output logic b_video_cs,
    output logic b_mmio_cs, 
    output logic b_wr,
    output logic b_rd,
    output logic [20:0]b_addr,
    output logic [31:0] b_wr_data ,
    input logic [31:0] b_rd_data
    );
     
   // declaration
   logic mcs_bridge_en;
   logic [29:0] word_addr;

   // body
   // address translation and decoding
   //  2 LSBs are "00" due to word alignment
   assign word_addr = io_address[31:2];

   //this bit is not clear !!!!!
   assign mcs_bridge_en = (io_address[31:24] == BRG_BASE[31:24]);


   assign b_video_cs = (mcs_bridge_en && io_address[23] == 1);
   assign b_mmio_cs = (mcs_bridge_en && io_address[23] == 0);
   assign b_addr = word_addr[20:0];
   //  control line conversion 
   assign b_wr = io_write_strobe;
   assign b_rd = io_read_strobe;
  // assign io_ready = 1;   // not used ; transaction done in 1 clock
   // data line conversion
   assign b_wr_data = io_write_data;
   assign io_read_data = b_rd_data;  
 endmodule
   


