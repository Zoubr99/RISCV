`include "clockworks.sv"

module SOC( // declaring the inputs and outputs
    input CLK,
    input RESET, // to be used with a reset. for now we will just declare a resetn as 1.
    output [4:0] LEDS,
    input RXD,
    output TXD
);

// adding a ROM
reg [4:0]Memmory[0:19]  = '{
  5'b00000, 5'b00001, 5'b00010, 5'b00011, 5'b00100, 
  5'b00101, 5'b00110, 5'b00111, 5'b01000, 5'b01001, 
  5'b01010, 5'b01011, 5'b01100, 5'b01101, 5'b01110, 
  5'b01111, 5'b10000, 5'b10001, 5'b10010, 5'b10011
};  



logic [4:0]PC = 0; // program counter

logic clk; // slower clock
logic resetn = 1;


logic [4:0] leds; // a register to store led values

// sequential logic based on clock edges
always_ff @(posedge clk) begin

        leds <= Memmory[PC]; // this assign the value of the memmory to the leds register
        PC <= (!resetn || PC == 20) ? 0 : (PC + 1); // program counter increases each cycle

end

clockworks #(.SLOW(27))
CW (

    .CLK(CLK),
    .RESET(RESET),
    .clk(clk),
    .resetn(resetn)


  );

    assign LEDS = leds; // assigning leds register to the output LEDS

    
endmodule