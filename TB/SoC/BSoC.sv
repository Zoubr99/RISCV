//`include "clk_divider.sv"
//`include "emitter_uart.v"

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
        wire [31:0] inter_MEM_Wdata;
        wire [3:0] inter_MEM_Wmask;
        wire [31:0] leds_x10;
        reg [31:0] MEM_io_leds;


        wire [31:0] inter_RAM_dout;
        wire [29:0] inter_MEM_Waddr = inter_MEMaddr[31:2];
        wire isIO = inter_MEMaddr[22];
        //wire isIO = (inter_MEMaddr[31] && inter_MEMaddr[30]); // SoC
        wire isRAM = !isIO;
        wire inter_wMEM_en = |inter_MEM_Wmask; 
        
       
        
    Ins_Memmory RAM (
        .CLK(CLK),
        .clk(clk),
        .MEM_addr(inter_MEMaddr),
        .MEM_dout(inter_RAM_dout),
        .rMEM_en(isRAM & inter_rMEMenable), 
        .MEM_Wdata(inter_MEM_Wdata),
        .MEM_Wmask({4{isRAM}} & inter_MEM_Wmask)
    );





    Processor_Core CPU(
        .clk(clk),
        .resetn(resetn),
        .MEM_addr(inter_MEMaddr),
        .MEM_dout(inter_MEMdout),
        .rMEM_en(inter_rMEMenable),
        .MEM_Wdata(inter_MEM_Wdata),
        .MEM_Wmask(inter_MEM_Wmask),
        .x10(leds_x10)
    );
/*
       // instantiate bridge
   mcs_bridge #(.BRG_BASE(BRG_BASE))
    bridge_unit (
                    .io_address(inter_MEMaddr),
                    .io_read_data(inter_MEMdout),
                    .io_read_strobe(isIO & inter_rMEMenable),
                    .io_write_data(inter_MEM_Wdata),
                    .io_write_strobe(isIO & inter_wMEM_en),
                 

                 // mmio bit signals
                    .b_video_cs(),
                    .b_mmio_cs(), 
                    .b_wr(),
                    .b_rd(),
                    .b_addr(),
                    .b_wr_data(),
                    .b_rd_data()                 
                );



*/
    localparam io_leds_bit = 0;
    localparam io_uartd_bit = 1;
    localparam io_uartc_bit = 2;

    always_ff @(posedge clk) begin
        if (isIO & inter_wMEM_en &inter_MEM_Waddr[io_leds_bit]) begin
            MEM_io_leds <= inter_MEM_Wdata;
        end
    end

assign LEDS = MEM_io_leds;

    wire uart_valid = (isIO & inter_wMEM_en & inter_MEM_Waddr[io_uartd_bit]);
    wire uart_ready;

    corescore_emitter_uart #(
        .clk_freq_hz(100*1000000),
        .baud_rate(115200)

    ) UART (

            .i_clk(CLK),
            .i_rst(!resetn),
            .i_data(inter_MEM_Wdata[7:0]),
            .i_valid(uart_valid),
            .o_ready_port(uart_ready),
            .o_uart_tx(TXD)
    );

   wire [31:0] inter_IO_dout = inter_MEM_Waddr[io_uartc_bit] ? {22'b0, !uart_ready, 9'b0} : 32'b0;

    assign inter_MEMdout = isRAM ? inter_RAM_dout : inter_IO_dout;    



    //    assign LEDS = leds_x10[4:0];


    clk_divider #(.SLOW(1))
    clk_divider (

        .CLK(CLK),
        .RESET(RESET),
        .clk(clk),
        .resetn(resetn)

    );

    

    //assign TXD = 1'b0;

endmodule