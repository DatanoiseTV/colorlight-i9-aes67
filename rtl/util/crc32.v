// SPDX-License-Identifier: MIT
// Ethernet CRC-32 (IEEE 802.3)
// Polynomial: 0x04C11DB7 (bit-reversed: 0xEDB88320)
// Processes 8 bits per clock cycle

module crc32 (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire [7:0]  data_in,
    output wire [31:0] crc_out,
    output wire        crc_valid  // high when crc_out == 32'hC704DD7B (residue)
);

    reg [31:0] crc_reg;

    // XOR-reduce 8 bits into 32-bit CRC state (unrolled Sarwate)
    function [31:0] crc_next;
        input [31:0] crc_prev;
        input [7:0]  din;
        integer i;
        reg [31:0] c;
        begin
            c = crc_prev;
            for (i = 0; i < 8; i = i + 1) begin
                if (c[0] ^ din[i])
                    c = {1'b0, c[31:1]} ^ 32'hEDB88320;
                else
                    c = {1'b0, c[31:1]};
            end
            crc_next = c;
        end
    endfunction

    always @(posedge clk) begin
        if (rst)
            crc_reg <= 32'hFFFFFFFF;
        else if (enable)
            crc_reg <= crc_next(crc_reg, data_in);
    end

    // Output is bit-reversed and inverted per IEEE 802.3
    assign crc_out = ~{crc_reg[0],  crc_reg[1],  crc_reg[2],  crc_reg[3],
                       crc_reg[4],  crc_reg[5],  crc_reg[6],  crc_reg[7],
                       crc_reg[8],  crc_reg[9],  crc_reg[10], crc_reg[11],
                       crc_reg[12], crc_reg[13], crc_reg[14], crc_reg[15],
                       crc_reg[16], crc_reg[17], crc_reg[18], crc_reg[19],
                       crc_reg[20], crc_reg[21], crc_reg[22], crc_reg[23],
                       crc_reg[24], crc_reg[25], crc_reg[26], crc_reg[27],
                       crc_reg[28], crc_reg[29], crc_reg[30], crc_reg[31]};

    assign crc_valid = (crc_reg == 32'hC704DD7B);

endmodule
