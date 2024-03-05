module clk_divider #(parameter SLOW)( // clock devider
    input logic  CLK,
    input logic  RESET,
    output logic clk,
    output logic resetn
);


// a parameterized regester (counter) -> 2^SLOW ticks of fpga clock (CLK) =1 tick  of (clk)
reg RST = 1;
reg [SLOW:0] slow_CLK = 0;
logic sclk;

always_ff @(posedge CLK) begin
    // increas the count
    slow_CLK <= slow_CLK + 1;

end
 
always_comb begin
sclk = (slow_CLK[SLOW-1])? 1 : 0;
end

// this sets the clk value based on the position chosen from SLOW
// so lets say SLOW is 8 -> taht means clk will remain zero untill bit 8 in SLOW is 1, keep in mind if lets say we wanted the clock to be 8 bits 2^7. we should set SLOW to 6 so as soon ass it reaches 2^7 or 01111111 it rolls over.
assign clk = sclk;

assign resetn = RST;
    
endmodule