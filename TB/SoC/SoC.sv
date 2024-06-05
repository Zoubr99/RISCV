//`include "clk_divider.sv"

module SoC(

        input logic CLK,
        input logic RESET,
        output  [4:0] LEDS,
        input logic  RXD, 
        output logic TXD

);

        wire clk;
        wire resetn;
    
    
        wire [31:0] inter_MEMaddr;
        wire [31:0] inter_MEMdout;
        wire inter_rMEMenable;
        wire [31:0] leds_x10;
        
       
        
    Ins_Memmory RAM (
        .CLK(CLK),
        .clk(clk),
        .MEM_addr(inter_MEMaddr),
        .MEM_dout(inter_MEMdout),
        .rMEM_en(inter_rMEMenable)
    );





    Processor_Core CPU(
        .clk(clk),
        .resetn(resetn),
        .MEM_addr(inter_MEMaddr),
        .MEM_dout(inter_MEMdout),
        .rMEM_en(inter_rMEMenable),
        .x10(leds_x10)
    );

        assign LEDS = leds_x10[4:0];


    clk_divider #(.SLOW(23))
    clk_divider (

        .CLK(CLK),
        .RESET(RESET),
        .clk(clk),
        .resetn(resetn)

    );

    assign TXD = 1'b0;

endmodule