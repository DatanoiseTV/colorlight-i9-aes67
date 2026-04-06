// SPDX-License-Identifier: MIT
// Synchronous FIFO using block RAM inference
// Parameterized width and depth (depth must be power of 2)

module fifo_sync #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 4  // depth = 2^ADDR_WIDTH
) (
    input  wire                    clk,
    input  wire                    rst,
    input  wire                    wr_en,
    input  wire [DATA_WIDTH-1:0]   wr_data,
    input  wire                    rd_en,
    output reg  [DATA_WIDTH-1:0]   rd_data,
    output wire                    full,
    output wire                    empty,
    output wire [ADDR_WIDTH:0]     count
);

    localparam DEPTH = 1 << ADDR_WIDTH;

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH:0]   wr_ptr;
    reg [ADDR_WIDTH:0]   rd_ptr;

    assign count = wr_ptr - rd_ptr;
    assign full  = (count == DEPTH);
    assign empty = (count == 0);

    always @(posedge clk) begin
        if (rst) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
        end else begin
            if (wr_en && !full) begin
                mem[wr_ptr[ADDR_WIDTH-1:0]] <= wr_data;
                wr_ptr <= wr_ptr + 1;
            end
            if (rd_en && !empty) begin
                rd_data <= mem[rd_ptr[ADDR_WIDTH-1:0]];
                rd_ptr <= rd_ptr + 1;
            end
        end
    end

endmodule
