// SPDX-License-Identifier: MIT
// Reset Synchronizer
//
// Asynchronous assert, synchronous deassert. Three-stage shift register
// to mitigate metastability. Output is high while async_rst is high or
// during the first 3 clock cycles after async_rst deasserts.

module reset_sync #(
    parameter STAGES = 3
) (
    input  wire clk,
    input  wire async_rst_n,   // active-low async reset (e.g. PLL lock)
    output wire sync_rst       // active-high synchronized reset
);

    (* ASYNC_REG = "TRUE" *) reg [STAGES-1:0] sync_chain;

    always @(posedge clk or negedge async_rst_n) begin
        if (!async_rst_n)
            sync_chain <= {STAGES{1'b1}};
        else
            sync_chain <= {sync_chain[STAGES-2:0], 1'b0};
    end

    assign sync_rst = sync_chain[STAGES-1];

endmodule
