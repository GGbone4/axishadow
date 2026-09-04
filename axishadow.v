//========================================================================
// axishadow.v
//========================================================================
// AXI Shadow module.
// 
// Description (from readme.md):
//   - Manages local line buffers for MPRED / IPP / LPF.
//   - Located between u_aml_axi_2to1 and u_hevc_axi4kb_split.
//   - Provides an AXI slave input port (data written in from upstream) and an
//     AXI master output port (data sourced to downstream).
// 
// This implementation shadows (buffers locally) the LPF traffic:
//   - An LPF WRITE whose AXI address falls inside a shadowed CTU region is
//     stored into the local line buffer (LLB) spsram and the write response is
//     generated locally.  It is NOT forwarded downstream.
//   - An LPF READ whose AXI address falls inside the shadowed CTU region is
//     served from the LLB.
//   - All other AXI traffic (non-LPF, or outside the shadowed region) is
//     forwarded straight through slave -> master (bypassed).
// 
// The base address of each CTU's data is:
//     base(ctu_count) = cfg_addr_lpf + (ctu_count << (cfg_ctu128 ? 12 : 11))
// where ctu_count is derived from the transaction address:
//     ctu_count = (addr - cfg_addr_lpf) >> (cfg_ctu128 ? 12 : 11)
// 
// A transaction is shadowed when:
//     base(ctu_count) <= addr < base(ctu_count) + cfg_ctucap_lpf*16
// (cfg_ctucap_lpf is a count of 128-bit AXI beats, i.e. 16 bytes each).
// 
// The LLB is a single-port 5268x128 spsram (read-first).  Shadowed writes and
// reads are serialized by the shadow FSMs so the single port is never
// contended.  AXI writes/reads are assumed to be non-interleaved (one write
// and one read transaction at a time), which holds for line-buffer traffic.
// 
// Coding style: Verilog-2005 (IEEE 1364-2005).
//========================================================================

`timescale 1ns / 1ps

module axishadow #(
    parameter ADDR_W = 34,        // AXI address width
    parameter ID_W = 8,           // AXI ID width
    parameter DATA_W = 128,       // AXI data width
    parameter STRB_W = DATA_W/8   // AXI strobe width  16byte
) (
    //========================================================================
    // AXI SLAVE input port (upstream)
    //========================================================================
    // Write address channel
    input wire [ID_W-1:0]   s_awid,
    input wire [ADDR_W-1:0] s_awaddr,
    input wire [3:0]        s_awlen,
    input wire [2:0]        s_awsize,
    input wire [1:0]        s_awburst,
    input wire [1:0]        s_awlock,
    input wire [3:0]        s_awcache,
    input wire [2:0]        s_awprot,
    input wire              s_awvalid,
    output wire             s_awready,
    // Write data channel
    input wire [ID_W-1:0]   s_wid,
    input wire [DATA_W-1:0] s_wdata,
    input wire [STRB_W-1:0] s_wstrb,
    input wire              s_wlast,
    input wire              s_wvalid,
    output wire             s_wready,
    // Write response channel
    output wire [ID_W-1:0]   s_bid,
    output wire [1:0]        s_bresp,
    output wire              s_bvalid,
    input wire               s_bready,
    // Read address channel
    input wire [ID_W-1:0]   s_arid,
    input wire [ADDR_W-1:0] s_araddr,
    input wire [3:0]        s_arlen,
    input wire [2:0]        s_arsize,
    input wire [1:0]        s_arburst,
    input wire [1:0]        s_arlock,
    input wire [3:0]        s_arcache,
    input wire [2:0]        s_arprot,
    input wire              s_arvalid,
    output wire             s_arready,
    // Read data channel
    output wire [ID_W-1:0]   s_rid,
    output wire [DATA_W-1:0] s_rdata,
    output wire [1:0]        s_rresp,
    output wire              s_rlast,
    output wire              s_rvalid,
    input wire               s_rready,

    //========================================================================
    // AXI MASTER output port (downstream)
    //========================================================================
    // Write address channel
    output wire [ID_W-1:0]   m_awid,
    output wire [ADDR_W-1:0] m_awaddr,
    output wire [3:0]        m_awlen,
    output wire [2:0]        m_awsize,
    output wire [1:0]        m_awburst,
    output wire [1:0]        m_awlock,
    output wire [3:0]        m_awcache,
    output wire [2:0]        m_awprot,
    output wire              m_awvalid,
    input wire               m_awready,
    // Write data channel
    output wire [ID_W-1:0]   m_wid,
    output wire [DATA_W-1:0] m_wdata,
    output wire [STRB_W-1:0] m_wstrb,
    output wire              m_wlast,
    output wire              m_wvalid,
    input wire               m_wready,
    // Write response channel
    input wire [ID_W-1:0]   m_bid,
    input wire [1:0]        m_bresp,
    input wire              m_bvalid,
    output wire             m_bready,
    // Read address channel
    output wire [ID_W-1:0]   m_arid,
    output wire [ADDR_W-1:0] m_araddr,
    output wire [3:0]        m_arlen,
    output wire [2:0]        m_arsize,
    output wire [1:0]        m_arburst,
    output wire [1:0]        m_arlock,
    output wire [3:0]        m_arcache,
    output wire [2:0]        m_arprot,
    output wire              m_arvalid,
    input wire               m_arready,
    // Read data channel
    input wire [ID_W-1:0]   m_rid,
    input wire [DATA_W-1:0] m_rdata,
    input wire [1:0]        m_rresp,
    input wire              m_rlast,
    input wire              m_rvalid,
    output wire             m_rready,

    //========================================================================
    // Local control / status
    //========================================================================
    input wire                  cfg_bypass        , // 1 = pass-through (default)
    input wire                  cfg_ctu128        , // 1 = CTU size is 128
    input wire  [ADDR_W-1:0]    cfg_addr_mpred    , // start axi address of MPRED
    input wire  [ADDR_W-1:0]    cfg_addr_ipp      , // start axi address of IPP
    input wire  [ADDR_W-1:0]    cfg_addr_lpf      , // start axi address of LPF
    input wire                  cfg_ctumode_mpred , // CTU mode for MPRED
    input wire                  cfg_ctumode_ipp   , // CTU mode for IPP
    input wire                  cfg_ctumode_lpf   , // CTU mode for LPF
    input wire  [5:0]           cfg_ctucap_mpred  , // CTU capacity of MPRED
    input wire  [5:0]           cfg_ctucap_ipp    , // CTU capacity of IPP
    input wire  [5:0]           cfg_ctucap_lpf    , // CTU capacity of LPF

    //========================================================================
    // Local line buffer (single-port sram) 5268x128
    //========================================================================
    output wire                 llb_we            , // local line buffer (spsram) write enable
    output wire [12:0]          llb_wa            , // local line buffer (spsram) write address
    output wire [127:0]         llb_wd            , // local line buffer (spsram) write data
    output wire                 llb_re            , // local line buffer (spsram) read enable
    output wire [12:0]          llb_ra            , // local line buffer (spsram) read address
    input  wire [127:0]         llb_rd            , // local line buffer (spsram) read data

    //========================================================================
    // Global
    //========================================================================
    input wire                  clk               , // Clock
    input wire                  rst_n               // Asynchronous reset, active low
);

    //========================================================================
    // Local parameters
    //========================================================================
    localparam LLB_AW = 13; // llb address width (5268 deep)
    localparam BURST_W = 4; // awlen/aelen width

    // Shadow read FSM states
    localparam R_IDLE   = 2'd0;
    localparam R_ISSUE  = 2'd1;
    localparam R_STREAM = 2'd2;

    //========================================================================
    // ---- Shadow-mode flag and hit decode (combinational) ----
    //========================================================================
    wire sh_en = ~cfg_bypass;

    // Shift for CTU stride: 128-CTU -> 12 (4096B), else 11 (2048B).
    wire [4:0]          ctu_shift = cfg_ctu128 ? 5'd12 : 5'd11;
    wire [ADDR_W-1:0]   cap_bytes = {ADDR_W{1'b0}} | (cfg_ctucap_lpf << 4);

    // ---- write-side decode ----
    wire [ADDR_W-1:0]   wr_offset = s_awaddr - cfg_addr_lpf;
    wire [ADDR_W-1:0]   wr_ctunum = wr_offset >> ctu_shift;
    wire [ADDR_W-1:0]   wr_base   = cfg_addr_lpf + (wr_ctunum << ctu_shift);
    wire [ADDR_W-1:0]   wr_end    = wr_base + cap_bytes;
    wire                lpf_wr_hit = (s_awaddr >= wr_base) & (s_awaddr < wr_end);

    // ---- read-side decode ----
    wire [ADDR_W-1:0]   rd_offset = s_araddr - cfg_addr_lpf;
    wire [ADDR_W-1:0]   rd_ctunum = rd_offset >> ctu_shift;
    wire [ADDR_W-1:0]   rd_base   = cfg_addr_lpf + (rd_ctunum << ctu_shift);
    wire [ADDR_W-1:0]   rd_end    = rd_base + cap_bytes;
    wire                lpf_rd_hit = (s_araddr >= rd_base) & (s_araddr < rd_end);

    //========================================================================
    // ---- Shadow READ state (declared early: used by write arbitration) ----
    //========================================================================
    reg  [1:0]          shr_state;
    reg  [BURST_W-1:0]  shr_cnt;     // beats remaining after current presented
    reg  [ID_W-1:0]     shr_arid;
    reg  [LLB_AW-1:0]   shr_ra;      // address driven on llb_ra
    reg  [DATA_W-1:0]   shr_rdata;   // presented read data (registered llb_rd)
    reg                 shr_rvalid;
    reg                 shr_rlast;
    wire                shr_idle = (shr_state == R_IDLE);

    //========================================================================
    // ---- Shadow WRITE datapath (store LPF write beats into the LLB) ----
    // To be robust to AW and the first W beat arriving in the same cycle,
    // we use an "armed" flag that is asserted as soon as the shadowed AW is
    // accepted and stays asserted until the WLAST beat is accepted.  It is
    // combinational-visible during the AW-accept cycle, so the concurrent
    // first W beat is correctly written and never forwarded.
    // The LLB write address for each W beat is (first address)+(beat index),
    // computed combinationally from the captured base and a beat counter.
    //========================================================================
    reg                 shw_armed;      // shadowed write AW accepted, W in progress
    reg  [BURST_W-1:0]  shw_awlen;      // captured awlen (beats - 1)
    reg  [ADDR_W-1:0]   shw_awaddr;     // captured awaddr (base of burst)
    reg  [ID_W-1:0]     shw_awid;       // captured awid
    reg  [BURST_W-1:0]  shw_nbeats;     // beats written so far in this burst

    wire                shw_aw_arm  = s_awvalid & s_awready & lpf_wr_hit; // AW accepted now
    wire                in_shadow_wr = shw_armed;                       // shadowed write in flight
    wire                shw_busy    = shw_armed;

    // Write beats accepted this cycle while a shadowed burst is active.
    wire                shw_w_ok    = s_wvalid & s_wready & (shw_armed | shw_aw_arm);

    // Accept AW: shadowed write when write FSM idle and read path idle
    // (single-port serialization); forwarded write otherwise via m_awready.
    assign s_awready = cfg_bypass ? m_awready
                     : (lpf_wr_hit ? ((~shw_busy) & (shr_state == R_IDLE)) : m_awready);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shw_armed   <= 1'b0;
            shw_awlen   <= {BURST_W{1'b0}};
            shw_awaddr  <= {ADDR_W{1'b0}};
            shw_awid    <= {ID_W{1'b0}};
            shw_nbeats  <= {BURST_W{1'b0}};
        end
        else if (cfg_bypass) begin
            shw_armed   <= 1'b0;
            shw_nbeats  <= {BURST_W{1'b0}};
        end
        else begin
            // AW accepted (may be same cycle as the first W beat).
            if (shw_aw_arm) begin
                shw_armed   <= 1'b1;
                shw_awlen   <= s_awlen;
                shw_awaddr  <= s_awaddr;
                shw_awid    <= s_awid;
                // Reset the beat counter at the start of a new shadowed burst
                // so the LLB write address is relative to this burst's base,
                // not the cumulative count of all prior shadowed beats.
                shw_nbeats  <= {BURST_W{1'b0}};
            end

            // Count accepted W beats of the shadowed burst.  The burst ends
            // when s_wlast is accepted.
            if (shw_w_ok) begin
                shw_nbeats  <= shw_nbeats + 1'b1;
                if (s_wlast) begin
                    shw_armed <= 1'b0;
                end
            end
        end
    end

    // Combinational LLB write address for the current W beat: the beat index
    // is shw_nbeats, added to the first beat's LLB address.  On the cycle the
    // AW is accepted together with the first W beat, the first address is
    // still on s_awaddr, so select that when shw_aw_arm is asserted.
    wire [LLB_AW-1:0]   shw_first_wa = (shw_awaddr - wr_base) >> 4;
    wire [LLB_AW-1:0]   shw_cur_wa   = shw_first_wa + shw_nbeats;

    // Write the LLB on each W handshake of the shadowed burst.
    assign llb_we = shw_w_ok;
    assign llb_wa = shw_aw_arm ? ((s_awaddr - wr_base) >> 4) : shw_cur_wa;
    assign llb_wd = s_wdata;

    // Local write response: one per shadowed write burst, after WLAST.
    reg                 shw_bvalid;
    reg  [ID_W-1:0]     shw_bid;
    reg  [1:0]          shw_bresp;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shw_bvalid <= 1'b0;
            shw_bid    <= {ID_W{1'b0}};
            shw_bresp  <= 2'b00;
        end
        else if (cfg_bypass) begin
            shw_bvalid <= 1'b0;
        end
        else if (shw_w_ok & s_wlast) begin
            shw_bvalid <= 1'b1;
            shw_bid    <= shw_awid;
            shw_bresp  <= 2'b00;        // OKAY
        end
        else if (shw_bvalid & s_bready) begin
            shw_bvalid <= 1'b0;         // B consumed
        end
    end

    //========================================================================
    // ---- Shadow READ datapath (serve LPF read beats from the LLB) ----
    // read-first spsram: llb_rd = data addressed by llb_ra of the previous
    // cycle.  R_ISSUE drives the first address; R_STREAM presents one beat
    // per cycle from the registered llb_rd (3-cycle latency after AR).
    //========================================================================
    // Accept AR: shadowed read when read FSM idle and write path idle.
    assign s_arready = cfg_bypass ? m_arready
                     : (lpf_rd_hit ? (shr_idle & (~shw_busy)) : m_arready);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shr_state   <= R_IDLE;
            shr_cnt     <= {BURST_W{1'b0}};
            shr_arid    <= {ID_W{1'b0}};
            shr_ra      <= {LLB_AW{1'b0}};
            shr_rdata   <= {DATA_W{1'b0}};
            shr_rvalid  <= 1'b0;
            shr_rlast   <= 1'b0;
        end
        else if (cfg_bypass) begin
            shr_state   <= R_IDLE;
            shr_rvalid  <= 1'b0;
            shr_rlast   <= 1'b0;
        end
        else begin
            case (shr_state)
                R_IDLE: begin
                    if (s_arvalid & s_arready & lpf_rd_hit) begin
                        shr_state   <= R_ISSUE;
                        shr_cnt     <= s_arlen;         // beats after beat0
                        shr_arid    <= s_arid;
                        shr_ra      <= (s_araddr - rd_base) >> 4;
                        shr_rvalid  <= 1'b0;
                        shr_rlast   <= 1'b0;
                    end
                    else begin
                        shr_rvalid  <= 1'b0;
                        shr_rlast   <= 1'b0;
                    end
                end
                R_ISSUE: begin
                    // llb_ra = first address is being read (data valid next
                    // cycle in R_STREAM).  Advance the address for beat1.
                    shr_state   <= R_STREAM;
                    shr_ra      <= shr_ra + 1'b1;
                    shr_rvalid  <= 1'b0;
                    shr_rlast   <= 1'b0;
                end
                R_STREAM: begin
                    // Always capture llb_rd (the data for the address read the
                    // previous cycle).  On a hold the address does not advance,
                    // so this re-captures the same data (idempotent).
                    shr_rdata   <= llb_rd;
                    if (s_rready) begin
                        // present the current beat and fetch the next one
                        shr_rvalid  <= 1'b1;
                        shr_rlast   <= (shr_cnt == 0);
                        shr_ra      <= shr_ra + 1'b1;
                        // Note: if shr_cnt hits 0, state transition logic would go back to R_IDLE (or handle completion).
                    end
                end
            endcase`
        end
    end

    // Write data (forward W only when not part of a shadowed write).
    // shw_w_ok covers both the armed phase and the AW+W same-cycle first beat.
    assign m_wid    = s_wid   ;
    assign m_wdata  = s_wdata ;
    assign m_wstrb  = s_wstrb ;
    assign m_wlast  = s_wlast ;
    assign m_wvalid = s_wvalid & ~(sh_en & (in_shadow_wr | shw_aw_arm));

    // Write response: accept the master B (consumed only for forwarded writes).
    assign m_bready = s_bready;

    // Read address (forward AR only when not a shadowed LPF read).
    assign m_arid   = s_arid   ;
    assign m_araddr = s_araddr ;
    assign m_arlen  = s_arlen  ;
    assign m_arsize = s_arsize ;
    assign m_arburst= s_arburst;
    assign m_arlock = s_arlock ;
    assign m_arcache= s_arcache;
    assign m_arprot = s_arprot ;
    assign m_arvalid= s_arvalid & ~(sh_en & lpf_rd_hit);

    // Read data: don't consume the master R channel while serving a local read.
    assign m_rready = sh_en ? (shr_idle ? s_rready : 1'b0) : s_rready;

    //========================================================================
    // ---- Slave-side output mux (local shadow vs. forwarded) ----
    //========================================================================
    // Write response.
    assign s_bid    = sh_en ? (shw_bvalid ? shw_bid    : m_bid)    : m_bid;
    assign s_bresp  = sh_en ? (shw_bvalid ? shw_bresp  : m_bresp)  : m_bresp;
    assign s_bvalid = sh_en ? (shw_bvalid | m_bvalid)               : m_bvalid;

    // Write data ready: accept W beats of a shadowed burst (including the
    // AW+W same-cycle first beat, covered by shw_aw_arm).
    assign s_wready = cfg_bypass ? m_wready
                    : ((in_shadow_wr | shw_aw_arm) ? 1'b1 : m_wready);

    // Read data.
    assign s_rid    = sh_en ? (shr_rvalid ? shr_arid   : m_rid)    : m_rid;
    assign s_rdata  = sh_en ? (shr_rvalid ? shr_rdata  : m_rdata)  : m_rdata;
    assign s_rresp  = sh_en ? (shr_rvalid ? 2'b00      : m_rresp)  : m_rresp;
    assign s_rlast  = sh_en ? (shr_rvalid ? shr_rlast  : m_rlast)  : m_rlast;
    assign s_rvalid = sh_en ? (shr_rvalid | m_rvalid)              : m_rvalid;

endmodule
