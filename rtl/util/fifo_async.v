// SPDX-License-Identifier: MIT
// Asynchronous FIFO for clock domain crossing
// Uses Gray-code pointers for safe CDC
// Depth must be power of 2

module fifo_async #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 4
) (
    // Write domain
    input  wire                    wr_clk,
    input  wire                    wr_rst,
    input  wire                    wr_en,
    input  wire [DATA_WIDTH-1:0]   wr_data,
    output wire                    full,

    // Read domain
    input  wire                    rd_clk,
    input  wire                    rd_rst,
    input  wire                    rd_en,
    output reg  [DATA_WIDTH-1:0]   rd_data,
    output wire                    empty
);

    localparam DEPTH = 1 << ADDR_WIDTH;

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // Binary and Gray-code pointers
    reg  [ADDR_WIDTH:0] wr_bin, rd_bin;
    reg  [ADDR_WIDTH:0] wr_gray, rd_gray;

    // Synchronized pointers (2-FF synchronizers)
    reg  [ADDR_WIDTH:0] wr_gray_sync1, wr_gray_sync2;
    reg  [ADDR_WIDTH:0] rd_gray_sync1, rd_gray_sync2;

    // Binary to Gray conversion
    function [ADDR_WIDTH:0] bin2gray;
        input [ADDR_WIDTH:0] bin;
        bin2gray = bin ^ (bin >> 1);
    endfunction

    // Write domain
    wire wr_allow = wr_en && !full;

    always @(posedge wr_clk) begin
        if (wr_rst) begin
            wr_bin  <= 0;
            wr_gray <= 0;
        end else if (wr_allow) begin
            mem[wr_bin[ADDR_WIDTH-1:0]] <= wr_data;
            wr_bin  <= wr_bin + 1;
            wr_gray <= bin2gray(wr_bin + 1);
        end
    end

    // Read domain
    wire rd_allow = rd_en && !empty;

    always @(posedge rd_clk) begin
        if (rd_rst) begin
            rd_bin  <= 0;
            rd_gray <= 0;
        end else if (rd_allow) begin
            rd_data <= mem[rd_bin[ADDR_WIDTH-1:0]];
            rd_bin  <= rd_bin + 1;
            rd_gray <= bin2gray(rd_bin + 1);
        end
    end

    // Synchronize write pointer to read domain
    always @(posedge rd_clk) begin
        if (rd_rst) begin
            wr_gray_sync1 <= 0;
            wr_gray_sync2 <= 0;
        end else begin
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
        end
    end

    // Synchronize read pointer to write domain
    always @(posedge wr_clk) begin
        if (wr_rst) begin
            rd_gray_sync1 <= 0;
            rd_gray_sync2 <= 0;
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
        end
    end

    // Full: write gray == inverted MSBs of synced read gray
    assign full  = (wr_gray == {~rd_gray_sync2[ADDR_WIDTH:ADDR_WIDTH-1],
                                 rd_gray_sync2[ADDR_WIDTH-2:0]});
    assign empty = (rd_gray == wr_gray_sync2);

endmodule
