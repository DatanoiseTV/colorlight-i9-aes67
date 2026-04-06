// SPDX-License-Identifier: MIT
// MDIO Master (IEEE 802.3 Clause 22)
//
// Bit-banged MDIO master state machine with CSR interface so the CPU
// can read and write PHY management registers for link status,
// autonegotiation control, statistics, etc.
//
// Frame format (Clause 22):
//
//   PRE  ST OP   PHYAD REGAD TA  DATA
//   1×32 01 R/W  AAAAA RRRRR ZZ  DDDDDDDDDDDDDDDD
//
//   ST     = 01 (start of frame)
//   OP     = 10 (read), 01 (write)
//   PHYAD  = 5-bit PHY address
//   REGAD  = 5-bit register address
//   TA     = turnaround (Z0 for read, 10 for write)
//   DATA   = 16-bit register value, MSB first
//
// MDC is a divided version of clk; we run at clk/(2*MDC_DIV) which gives
// a 2.5 MHz MDC at 125 MHz / 50.

module mdio_master #(
    parameter MDC_DIV = 50    // 125 MHz / (2 * 50) = 1.25 MHz MDC
) (
    input  wire        clk,
    input  wire        rst,

    // CSR interface
    input  wire        cmd_start,        // pulse to start a transaction
    input  wire        cmd_op_read,      // 1 = read, 0 = write
    input  wire [4:0]  cmd_phy_addr,
    input  wire [4:0]  cmd_reg_addr,
    input  wire [15:0] cmd_write_data,
    output reg  [15:0] read_data,
    output reg         busy,

    // MDIO pins
    output reg         mdc,
    output reg         mdio_oe,          // 1 = drive, 0 = high-Z
    output reg         mdio_o,
    input  wire        mdio_i
);

    // ---- MDC clock divider ----
    reg [$clog2(MDC_DIV)-1:0] mdc_cnt;
    reg                       mdc_tick;   // pulse on MDC rising/falling edges

    always @(posedge clk) begin
        if (rst) begin
            mdc_cnt  <= 0;
            mdc_tick <= 0;
            mdc      <= 0;
        end else begin
            mdc_tick <= 0;
            if (mdc_cnt == MDC_DIV - 1) begin
                mdc_cnt  <= 0;
                mdc      <= ~mdc;
                mdc_tick <= 1;
            end else begin
                mdc_cnt <= mdc_cnt + 1;
            end
        end
    end

    // ---- Frame state machine ----
    // We assemble the entire frame as a 64-bit shift register and shift one
    // bit out per MDC rising edge. For reads, we tri-state on the TA cycle
    // and sample mdio_i on the falling edge of MDC for the next 16 cycles.

    localparam [2:0]
        S_IDLE      = 3'd0,
        S_PREAMBLE  = 3'd1,
        S_FRAME_TX  = 3'd2,
        S_TURN_RD   = 3'd3,
        S_DATA_RD   = 3'd4,
        S_DONE      = 3'd5;

    reg [2:0]  state;
    reg [5:0]  bit_cnt;
    reg [31:0] frame_tx;          // ST(2)+OP(2)+PHYAD(5)+REGAD(5)+TA(2)+DATA(16) = 32
    reg [4:0]  preamble_cnt;
    reg [15:0] read_shift;
    reg        op_was_read;
    reg        mdc_d;             // for edge detection within the state machine

    wire mdc_rise = ~mdc_d &  mdc;
    wire mdc_fall =  mdc_d & ~mdc;

    always @(posedge clk) begin
        if (rst) begin
            state         <= S_IDLE;
            bit_cnt       <= 0;
            frame_tx      <= 0;
            preamble_cnt  <= 0;
            read_shift    <= 0;
            op_was_read   <= 0;
            mdio_oe       <= 0;
            mdio_o        <= 1;
            busy          <= 0;
            read_data     <= 0;
            mdc_d         <= 0;
        end else begin
            mdc_d <= mdc;

            case (state)
                S_IDLE: begin
                    busy    <= 0;
                    mdio_oe <= 0;
                    mdio_o  <= 1;
                    if (cmd_start) begin
                        busy        <= 1;
                        op_was_read <= cmd_op_read;
                        // Build the 32-bit frame after preamble:
                        //   bit 31..30 : ST = 01
                        //   bit 29..28 : OP (10 read, 01 write)
                        //   bit 27..23 : PHYAD
                        //   bit 22..18 : REGAD
                        //   bit 17..16 : TA
                        //   bit 15..0  : DATA
                        frame_tx     <= {2'b01,
                                         (cmd_op_read ? 2'b10 : 2'b01),
                                         cmd_phy_addr,
                                         cmd_reg_addr,
                                         (cmd_op_read ? 2'bzz : 2'b10),
                                         cmd_write_data};
                        preamble_cnt <= 5'd31;     // 32 ones
                        state        <= S_PREAMBLE;
                    end
                end

                S_PREAMBLE: begin
                    mdio_oe <= 1;
                    mdio_o  <= 1;
                    if (mdc_fall) begin
                        if (preamble_cnt == 0) begin
                            state   <= S_FRAME_TX;
                            bit_cnt <= 6'd31;          // shift out 32 bits
                        end else begin
                            preamble_cnt <= preamble_cnt - 1;
                        end
                    end
                end

                S_FRAME_TX: begin
                    mdio_oe <= 1;
                    mdio_o  <= frame_tx[bit_cnt];
                    if (mdc_fall) begin
                        // For reads we go high-Z at TA (bits 17..16)
                        if (op_was_read && bit_cnt == 6'd17) begin
                            mdio_oe <= 0;
                            state   <= S_TURN_RD;
                            bit_cnt <= 6'd1;
                        end else if (bit_cnt == 0) begin
                            state <= S_DONE;
                        end else begin
                            bit_cnt <= bit_cnt - 1;
                        end
                    end
                end

                S_TURN_RD: begin
                    mdio_oe <= 0;
                    if (mdc_fall) begin
                        if (bit_cnt == 0) begin
                            state   <= S_DATA_RD;
                            bit_cnt <= 6'd15;
                        end else begin
                            bit_cnt <= bit_cnt - 1;
                        end
                    end
                end

                S_DATA_RD: begin
                    mdio_oe <= 0;
                    // Sample on MDC rising edge (PHY drives mid-MDC-low)
                    if (mdc_rise)
                        read_shift <= {read_shift[14:0], mdio_i};
                    if (mdc_fall) begin
                        if (bit_cnt == 0) begin
                            read_data <= {read_shift[14:0], mdio_i};
                            state     <= S_DONE;
                        end else begin
                            bit_cnt <= bit_cnt - 1;
                        end
                    end
                end

                S_DONE: begin
                    mdio_oe <= 0;
                    if (mdc_fall) begin
                        busy  <= 0;
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
