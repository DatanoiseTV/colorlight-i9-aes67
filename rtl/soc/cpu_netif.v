// SPDX-License-Identifier: MIT
// CPU Network Interface
//
// BRAM-buffered Ethernet netif accessible from a Wishbone CPU.
// Modeled after the LiteEth MAC slot interface so it integrates cleanly with
// LiteX's CSR/Wishbone wrapper.
//
// Layout:
//   - 2 KB RX BRAM (single slot, complete frame)
//   - 2 KB TX BRAM (single slot, complete frame)
//   - Status/control registers (Wishbone-mapped via LiteX CSR wrapper)
//
// CSRs (read by AutoCSR wrapper in LiteX):
//   rx_ready    R    : 1 = a frame is available in RX BRAM
//   rx_length   R    : length of received frame in bytes
//   rx_ack      W    : write 1 to release RX slot for the next frame
//   tx_pending  R    : 1 = TX slot is in use
//   tx_length   W    : length of frame to send (writing also triggers send)
//   irq         R    : RX-pending interrupt status
//
// The CPU stages a packet by writing to the TX BRAM via Wishbone, then
// writes `tx_length` to send. RX is mirrored: a complete frame lands in RX
// BRAM, the CPU reads from BRAM, then writes `rx_ack`.

module cpu_netif #(
    parameter RX_BUF_BITS = 11,    // 2 KB
    parameter TX_BUF_BITS = 11
) (
    input  wire        clk,
    input  wire        rst,

    // Streaming side: from packet router
    input  wire [7:0]  rx_axis_tdata,
    input  wire        rx_axis_tvalid,
    input  wire        rx_axis_tlast,
    output reg         rx_axis_tready,

    // Streaming side: to TX arbiter
    output reg  [7:0]  tx_axis_tdata,
    output reg         tx_axis_tvalid,
    output reg         tx_axis_tlast,
    input  wire        tx_axis_tready,

    // CPU access (registered) - simple synchronous SRAM port
    // RX BRAM read
    input  wire [RX_BUF_BITS-1:0] rx_mem_addr,
    output reg  [7:0]  rx_mem_data,
    // TX BRAM write
    input  wire [TX_BUF_BITS-1:0] tx_mem_addr,
    input  wire [7:0]  tx_mem_wdata,
    input  wire        tx_mem_we,

    // Control/status (registered, read by LiteX CSR wrapper)
    output reg         rx_ready,
    output reg  [15:0] rx_length,
    input  wire        rx_ack,        // pulse from CPU
    output reg         tx_pending,
    input  wire        tx_send,       // pulse from CPU
    input  wire [15:0] tx_length,
    output wire        irq            // RX-pending interrupt
);

    // ---- BRAMs ----
    reg [7:0] rx_mem [0:(1<<RX_BUF_BITS)-1];
    reg [7:0] tx_mem [0:(1<<TX_BUF_BITS)-1];

    // ---- RX state machine ----
    reg [RX_BUF_BITS-1:0] rx_wr_addr;

    always @(posedge clk) begin
        if (rst) begin
            rx_wr_addr     <= 0;
            rx_ready       <= 0;
            rx_length      <= 0;
            rx_axis_tready <= 0;
        end else begin
            rx_axis_tready <= ~rx_ready;  // accept if slot is empty

            if (rx_axis_tvalid && rx_axis_tready) begin
                rx_mem[rx_wr_addr] <= rx_axis_tdata;
                rx_wr_addr <= rx_wr_addr + 1;

                if (rx_axis_tlast) begin
                    rx_ready   <= 1;
                    rx_length  <= {{(16-RX_BUF_BITS){1'b0}}, rx_wr_addr} + 1;
                    rx_wr_addr <= 0;
                end
            end

            if (rx_ack) begin
                rx_ready   <= 0;
                rx_wr_addr <= 0;
            end

            // CPU read port (registered, 1-cycle latency)
            rx_mem_data <= rx_mem[rx_mem_addr];
        end
    end

    // ---- TX state machine ----
    reg [TX_BUF_BITS-1:0] tx_rd_addr;
    reg [15:0]            tx_len_latch;
    reg                   tx_running;

    always @(posedge clk) begin
        if (rst) begin
            tx_rd_addr     <= 0;
            tx_len_latch   <= 0;
            tx_running     <= 0;
            tx_pending     <= 0;
            tx_axis_tdata  <= 0;
            tx_axis_tvalid <= 0;
            tx_axis_tlast  <= 0;
        end else begin
            // CPU write port
            if (tx_mem_we)
                tx_mem[tx_mem_addr] <= tx_mem_wdata;

            // Trigger transmission
            if (tx_send && !tx_running) begin
                tx_running   <= 1;
                tx_pending   <= 1;
                tx_rd_addr   <= 0;
                tx_len_latch <= tx_length;
            end

            // Stream out
            if (tx_running) begin
                tx_axis_tdata  <= tx_mem[tx_rd_addr];
                tx_axis_tvalid <= 1;
                tx_axis_tlast  <= (tx_rd_addr == tx_len_latch - 1);

                if (tx_axis_tready) begin
                    if (tx_rd_addr == tx_len_latch - 1) begin
                        tx_running <= 0;
                        tx_pending <= 0;
                        tx_axis_tvalid <= 0;
                        tx_axis_tlast  <= 0;
                    end else begin
                        tx_rd_addr <= tx_rd_addr + 1;
                    end
                end
            end
        end
    end

    assign irq = rx_ready;

endmodule
