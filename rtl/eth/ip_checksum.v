// SPDX-License-Identifier: MIT
// Streaming IPv4 header checksum (RFC 1071)
//
// Accepts header bytes one at a time, accumulates the 16-bit one's complement
// sum, and outputs the final checksum after `done` is asserted.

module ip_checksum (
    input  wire        clk,
    input  wire        rst,
    input  wire        clear,         // start a new sum
    input  wire        word_valid,    // assert when a 16-bit word is presented
    input  wire [15:0] word,
    input  wire        done,          // assert when all header words have been fed
    output reg  [15:0] checksum
);

    reg [16:0] sum;     // one extra bit for carry

    always @(posedge clk) begin
        if (rst || clear) begin
            sum      <= 0;
            checksum <= 0;
        end else begin
            if (word_valid) begin
                // Add with end-around carry
                sum <= sum[15:0] + sum[16] + word;
            end
            if (done) begin
                checksum <= ~(sum[15:0] + sum[16]);
            end
        end
    end

endmodule
