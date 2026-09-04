AXI Shadow
# 1. 模块功能概述

监控 AXI 总线流量。当发现发往特定 LPF（环路滤波）区域的数据时，将其拦截并存入本地 SRAM（LLB）中，并直接在本地返回响应；对于其他不相关的数据，则透传（Bypass）给下游，节省外部 DDR 的带宽。

# 2. 代码段详细介绍

## 2.1 端口定义

```verilog
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
```

这部分代码定义了它与外界环境（上游、下游、配置寄存器、SRAM）的连接方式。
模块被夹在上游（Slave 接口）和下游（Master 接口）之间。它还需要知道当前系统的配置（如 LPF 基地址 `cfg_addr_lpf`），并与一块 128-bit 宽度的本地 SRAM（LLB）相连，用于临时缓存数据。

## 2.2 地址解码/Shadow缓存

说明：对于一个CTU内部，如果访问的地址落在了LPF模块内，那么就和sram进行数据访问，如果不在lpf就和ddr进行交互

```verilog
内存地址增加方向 ===>
[第 0 个 CTU ]                                                    [第 1 个 CTU ]
|-------------------------------------| |-------------------------------------|
|███████████████████                  | |███████████████████                  |
|█ LPF 有效数据区域█   其余数据区域       | |█ LPF 有效数据区域█   其余数据区域   |
|███████████████████                  | |███████████████████                  |
|-------------------------------------| |-------------------------------------|
^                   ^                 ^                                ^
|                   |                 | |
wr_base             wr_end                                    | (下一个) wr_base
|                   |                 |
|<--- cap_bytes --->|                
|                                     |
|<---------- ctu_shift (4096B) ------>|
```

```verilog
// ---- 影子模式使能 ----
wire sh_en = ~cfg_bypass;

// Shift 决定 CTU 的步长: 128x128的CTU位移12位(4096B)，否则位移11位(2048B)。
wire [4:0]          ctu_shift = cfg_ctu128 ? 5'd12 : 5'd11;
wire [ADDR_W-1:0]   cap_bytes = {ADDR_W{1'b0}} | (cfg_ctucap_lpf << 4);

// ---- 写通道地址解码 ----
wire [ADDR_W-1:0]   wr_offset = s_awaddr - cfg_addr_lpf;       // 计算相对地址
wire [ADDR_W-1:0]   wr_ctunum = wr_offset >> ctu_shift;        // 计算属于第几个 CTU
wire [ADDR_W-1:0]   wr_base   = cfg_addr_lpf + (wr_ctunum << ctu_shift); // 算出该 CTU 的绝对基地址
wire [ADDR_W-1:0]   wr_end    = wr_base + cap_bytes;           // 算出有效边界
// 判断当前写地址是否在有效范围内
wire                lpf_wr_hit = (s_awaddr >= wr_base) & (s_awaddr < wr_end);

// ---- 读通道地址解码 (同理) ----
wire                lpf_rd_hit = (s_araddr >= rd_base) & (s_araddr < rd_end);
```

1. 模块通过 `s_awaddr`（当前 AXI 请求地址）减去 LPF 首地址，得到相对地址。
2. 根据 CTU（编码树单元）大小进行移位运算，算出当前数据属于哪个 CTU。
3. 计算出当前CTU的基地址和边界容量
4. 如果当前请求地址落在了CTU边界内，则表示命中，开启拦截
5. 读写同理

但是这里有几个问题：

- **ctu_shift**(CTU的移位)是什么意思

```verilog
// Shift 决定 CTU 的步长: 128x128的CTU位移12位(4096B)，否则位移11位(2048B)。
wire [4:0]          ctu_shift = cfg_ctu128 ? 5'd12 : 5'd11;
```

CTU（Coding Tree Unit，编码树单元）是视频压缩（如 HEVC/H.265）中的基本处理块。处理高分辨率视频时，内存中会把图像划分为一个一个的 CTU 数据块。
当配置为 128x128 像素的 CTU 时，它在内存中占据的空间（步长）是 **4096 字节(2^12 = 4096)**。
当配置为其他尺寸（比如 64x64）时，占据的空间是 **2048 字节(2^11 = 2048)**。
**在计算机和硬件当中字节是基本的处理和计算单位**

- `cap_bytes`是什么意思

```verilog
wire [ADDR_W-1:0]   cap_bytes = {ADDR_W{1'b0}} | (cfg_ctucap_lpf << 4);
```

计算**一个 CTU 的 LPF 数据实际上有多大（以字节为单位）**

**第 1 部分：`(cfg_ctucap_lpf << 4)` —— 单位转换**
`cfg_ctucap_lpf` 代表的是 LPF 数据的容量，但它的单位是 **AXI beat（节拍）**。
AXI 数据位宽是 128-bit，也就是 **16 字节 (16 Bytes)**。
所以，实际的字节数 = `cfg_ctucap_lpf * 16`。
同样为了不用乘法器，乘以 16 在二进制中就是**向左移动 4 位 (`<< 4`)**。（因为 2^4 = 16）。
**第 2 部分：`{ADDR_W{1'b0}}` —— 补零扩宽**
• 这是一个 Verilog 的语法，叫**复制拼接操作符（Replication operator）**。
• `1'b0` 表示 1 位的数字 0。
• `{ADDR_W{...}}` 表示把大括号里的内容复制 `ADDR_W` 次。
• 假设系统的地址位宽 `ADDR_W` 是 34，那 `{ADDR_W{1'b0}}` 就直接生成了一个 **34 个 0 组成的二进制数**（`34'b0000...0000`）。
**第 3 部分：`|` （按位或操作） —— 安全的位宽对齐**
• 为什么要把一堆 0 和算出来的容量进行“或（OR）”操作呢？
• 在代码定义里，`cfg_ctucap_lpf` 只有 **6 位**宽。左移 4 位后，它最多只有 10 位宽。
• 但是，最终算出来的边界 `wr_end = wr_base + cap_bytes;`，这里的运算都是基于完整的 AXI 地址位宽（比如 **34 位**）来进行的。
• 把一个 10 位宽的变量强行加到一个 34 位宽的变量上，有些严谨的综合工具（EDA 软件）会报 Warning（位宽不匹配）。
• 为了写出最安全、无警告的代码，设计师用 34 位的全 `0` 和容量值做按位或。**任何数与 0 做或运算，值都不变**。这样做的唯一目的，就是把一个短位数（10位）**高位补零，安全地强制扩展成了 34 位的长位数 `cap_bytes`**。

cap_bytes表示的是一个CTU内占的实际内存大小

## 2.3 Shadow写数据通路

当写操作命中（`lpf_wr_hit == 1`）时，数据不会发往内存，而是存进本地 SRAM。

```verilog
    reg                 shw_armed;      // 标志位：写地址(AW)已被接受，正在等待/接收写数据(W)
    reg  [BURST_W-1:0]  shw_awlen;      // 锁存的突发长度
    reg  [ADDR_W-1:0]   shw_awaddr;     // 锁存的写首地址
    reg  [ID_W-1:0]     shw_awid;       // 锁存的写事务ID
    reg  [BURST_W-1:0]  shw_nbeats;     // 计数器：记录当前突发已经写入了多少拍(beat)数据

    // 当输入写地址有效(awvalid)、本模块准备好(awready)、且地址命中缓存区(lpf_wr_hit)时，触发一次写入接受(arm)
    wire                shw_aw_arm  = s_awvalid & s_awready & lpf_wr_hit; 
    wire                in_shadow_wr = shw_armed;                      // 当前是否处于影子写过程中
    wire                shw_busy    = shw_armed;                       // 写模块忙碌标志

    // shw_w_ok: 当前周期是否成功握手接收了一笔影子写数据。要求W有效、W准备好、且处于影子写激活状态。
    wire                shw_w_ok    = s_wvalid & s_wready & (shw_armed | shw_aw_arm);

    // s_awready (反向握手信号控制)
    // 如果旁路，直连下游 m_awready。
    // 如果命中拦截区，则需等待当前写和读FSM都不忙(由于单端口SRAM不支持同时读写)，才拉高。否则还是交给下游。
    assign s_awready = cfg_bypass ? m_awready
                     : (lpf_wr_hit ? ((~shw_busy) & (shr_state == R_IDLE)) : m_awready);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 复位清零所有寄存器
            shw_armed   <= 1'b0;
            shw_awlen   <= {BURST_W{1'b0}};
            shw_awaddr  <= {ADDR_W{1'b0}};
            shw_awid    <= {ID_W{1'b0}};
            shw_nbeats  <= {BURST_W{1'b0}};
        end
        else if (cfg_bypass) begin
            shw_armed   <= 1'b0;
            shw_nbeats  <= {BURST_W{1'b0}}; // 旁路模式下强制清零核心状态
        end
        else begin
            if (shw_aw_arm) begin // 当接收到合法的影子AW握手时
                shw_armed   <= 1'b1;        // 置起 armed 标志
                shw_awlen   <= s_awlen;     // 记录相关信息
                shw_awaddr  <= s_awaddr;
                shw_awid    <= s_awid;
                shw_nbeats  <= {BURST_W{1'b0}}; // 数据拍数清零，准备开始接收数据
            end

            if (shw_w_ok) begin // 当接收到有效的写数据(W通道握手)
                shw_nbeats  <= shw_nbeats + 1'b1; // 拍数+1
                if (s_wlast) begin // 如果是突发的最后一拍数据
                    shw_armed <= 1'b0; // 写事务结束，清空 armed 标志
                end
            end
        end
    end

    // 本地SRAM地址计算逻辑
    // SRAM每个地址存128bit(16Byte)。(shw_awaddr - wr_base) 算出基于当前CTU的字节偏移，右移4(/16)算出首地址SRAM字偏移。
    
    // **这里定义的AXI 地址必须是16字节对齐 即低四位为0 而且AXI每一拍都是128byte正好对应每个SRAM的行**
    wire [LLB_AW-1:0]   shw_first_wa = (shw_awaddr - wr_base) >> 4; 
    // 当前拍对应的SRAM地址 = 首地址 + 当前已写入的拍数
    wire [LLB_AW-1:0]   shw_cur_wa   = shw_first_wa + shw_nbeats;

    // 连接给内部SRAM的写信号
    assign llb_we = shw_w_ok; // 每收到一拍有效拦截数据，就写一次SRAM
    // 如果首地址和首数据同一周期到达(shw_aw_arm有效)，用输入地址现算；否则用已锁存并累加的地址 shw_cur_wa。
    assign llb_wa = shw_aw_arm ? ((s_awaddr - wr_base) >> 4) : shw_cur_wa; 
    assign llb_wd = s_wdata;  // AXI写入的数据直接接到SRAM数据口
```

```verilog
// ---- 伪造本地 B 通道写响应 ----
    always @(posedge clk or negedge rst_n) begin
        // ...
        else if (shw_w_ok & s_wlast) begin
            shw_bvalid <= 1'b1;       // 伪造写成功响应信号
            shw_bid    <= shw_awid;
            shw_bresp  <= 2'b00;      // OKAY (表示写入成功)
        end
        else if (shw_bvalid & s_bready) begin
            shw_bvalid <= 1'b0;       // 响应被上游接收后拉低
        end
    end
```

上游以为它在往内存里写数据，但实际上这个模块把 AXI 地址转换为了 SRAM 的地址 (`llb_wa`)，把数据 (`s_wdata`) 直接写进了 SRAM (`llb_wd`)。
因为拦截了数据，外部内存自然不会返回写成功信号（B 通道）。所以模块必须在本地自己“捏造”一个写成功响应 (`bresp = OKAY`)，发给上游，以完成 AXI 协议的闭环。

## 2.4 Shadow读数据通路拦截

当读操作命中（`lpf_rd_hit == 1`）时，直接从本地 SRAM 里取数据发给上游。

```verilog
	// AR通道反向握手控制：同样要求在空闲且非影子写状态下才能接收(确保单端口SRAM不冲突)
    assign s_arready = cfg_bypass ? m_arready
                     : (lpf_rd_hit ? (shr_idle & (~shw_busy)) : m_arready);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 复位信号...
            shr_state   <= R_IDLE;
            shr_cnt     <= {BURST_W{1'b0}};
            shr_arid    <= {ID_W{1'b0}};
            shr_ra      <= {LLB_AW{1'b0}};
            shr_rdata   <= {DATA_W{1'b0}};
            shr_rvalid  <= 1'b0;
            shr_rlast   <= 1'b0;
        end
        else if (cfg_bypass) begin
            // 旁路清零...
            shr_state   <= R_IDLE;
            shr_rvalid  <= 1'b0;
            shr_rlast   <= 1'b0;
        end
        else begin
            case (shr_state)
                R_IDLE: begin // 空闲态
                    if (s_arvalid & s_arready & lpf_rd_hit) begin // 接受了有效的影子读请求
                        shr_state   <= R_ISSUE; // 状态跳转到发地址态
                        shr_cnt     <= s_arlen;         // 锁存需要读取的总拍数(beats-1)
                        shr_arid    <= s_arid;          // 锁存读ID
                        shr_ra      <= (s_araddr - rd_base) >> 4; // 算出在SRAM中的起始字地址
                        shr_rvalid  <= 1'b0;
                        shr_rlast   <= 1'b0;
                    end
                    else begin
                        shr_rvalid  <= 1'b0;
                        shr_rlast   <= 1'b0;
                    end
                end
                R_ISSUE: begin
                    // 此周期 SRAM 读取地址 (shr_ra) 已经送到物理SRAM，数据将在下一个周期 R_STREAM 准备好
                    shr_state   <= R_STREAM; 
                    shr_ra      <= shr_ra + 1'b1; // 地址提前自增，准备读取第2拍(如果有的话)
                    shr_rvalid  <= 1'b0;
                    shr_rlast   <= 1'b0;
                end
                R_STREAM: begin // 数据流水读取态
                    shr_rdata   <= llb_rd; // 将从 SRAM 返回的数据锁存(SRAM读延时为1周期)
                    if (s_rready) begin    // 如果上游准备好接收读数据
                        shr_rvalid  <= 1'b1;                // 标记输出的数据有效
                        shr_rlast   <= (shr_cnt == 0);      // 检查当前是不是最后一拍 (cnt倒计为0)
                        shr_ra      <= shr_ra + 1'b1;       // 继续增加SRAM读取地址
                        // 注解指出：如果 shr_cnt 等于 0，应当有返回 R_IDLE 的逻辑处理。但当前提供的代码片段在此处被精简截断了。
                    end
                end
            endcase
        end
    end
```

为了匹配 SRAM 的读取延迟，设计了一个三段式状态机 (`R_IDLE` -> `R_ISSUE` -> `R_STREAM`)。它将 AXI 突发读请求转换为连续的 SRAM 读地址 (`shr_ra`)，然后通过 `shr_rdata` 和 `shr_rvalid` 将取出的数据按照 AXI R 通道的标准“吐”给上游。

## 2.5 数据流路口：输出信号复用与透传

```verilog
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
```

## 2.6 写数据流程

### 场景一：命中拦截区（Shadowed Write / 影子写）

当上游发起写请求，且地址命中 `cfg_addr_lpf` 对应的 CTU 范围（即 `lpf_wr_hit == 1`），模块会把数据截下来存入本地单端口 SRAM，并自己伪造成功响应返回给上游。下游（Master）对此一无所知。

完整的 AXI 三通道（AW、W、B）交互流程如下：

#### 第一阶段：写地址握手 (AW Channel)

1. **上游发起请求**：上游拉高 `s_awvalid`，并送出 `s_awaddr`、`s_awlen`、`s_awid` 等写地址信息。
2. **地址解码**：模块内部的组合逻辑立即计算出该地址命中了缓存区，`lpf_wr_hit` 变为 `1`。
3. **仲裁与准备**：模块检查当前本地 SRAM 是否空闲（写状态机未被占用 `~shw_busy`，且读状态机也在空闲态 `shr_state == R_IDLE`）。如果空闲，则拉高 `s_awready`，与上游完成握手（此动作称为 `shw_aw_arm`）。
4. **锁存信息**：在时钟上升沿，模块将上游的写地址信息（地址、长度、ID）锁存到内部寄存器（`shw_awaddr`等），并把突发计数器 `shw_nbeats` 清零，同时拉高 `shw_armed`，宣告**进入影子写的数据接收状态**。

#### 第二阶段：写数据握手与存入 SRAM (W Channel)

1. **阻断下游**：因为是本地拦截，发往下游的有效信号 `m_wvalid` 会被强制拉低（屏蔽），防止数据泄漏给主存。
2. **接收数据**：由于处于影子写状态（`in_shadow_wr` 为 1），模块强制向上一级拉高 `s_wready = 1`，表示本地随时可以吞吐数据。
3. **计算 SRAM 地址**：
    - 模块内部会计算：`基址偏移量 = 锁存地址 - 该CTU的起始地址`。
    - 然后将偏移量除以 16（右移 4 位，因为 SRAM 位宽是 128-bit/16-byte），得到 SRAM 的起始词地址。
    - **当前 SRAM 写入地址** = `起始词地址 + 当前已收的拍数 (shw_nbeats)`。
4. **写入 SRAM**：每当上游送来一个有效数据（`s_wvalid == 1`），就会触发 `llb_we = 1`。数据直接被打入本地单端口 SRAM。同时 `shw_nbeats` 加 1。
5. **突发结束**：当上游发送最后一拍数据，并伴随 `s_wlast == 1` 时，模块识别到传输结束，在下一个时钟周期将 `shw_armed` 清零，退出写数据接收状态。
    - *(注：代码极其严谨，专门处理了 AW 握手和第一拍 W 数据在同一个时钟周期到达的极限情况，使用了 `shw_aw_arm` 组合逻辑直接放行第一拍数据。)*

#### 第三阶段：本地生成写响应 (B Channel)

1. **触发响应**：在接收到最后一拍数据（`shw_w_ok & s_wlast`）的时钟边沿，触发响应状态机。
2. **伪造响应包**：模块内部拉高 `shw_bvalid`，将响应状态 `shw_bresp` 设为 `2'b00` (OKAY，表示写入成功)，并将前面锁存的 `shw_awid` 赋给 `shw_bid`。
3. **返回上游**：通过输出 Mux，将上述本地生成的 B 通道信号发给上游（`s_bvalid` 拉高）。
4. **响应握手**：当上游准备好接收，拉高 `s_bready` 时，握手完成。下一个时钟周期，模块清零 `shw_bvalid`。**至此，一次完整的影子写传输完美结束。**

### 场景二：未命中 / 直通模式（Forwarded Write / 透传写）

如果写地址不在 LPF 范围内（`lpf_wr_hit == 0`），或者开启了全局直通（`cfg_bypass == 1`），本模块会退化为一根“透明的导线”，完全由下游设备（例如 DDR 内存控制器）来决定流程。

整个流程如下：

#### 第一阶段：写地址穿透 (AW Channel)

1. 模块将上游的写地址信息原封不动地赋给 `m_aw_xxx` 端口，发往下游。
2. 模块将下游返回的 `m_awready` 直接连给上游的 `s_awready`。
3. 握手完全由上游和下游协商，本模块不干预。

#### 第二阶段：写数据穿透 (W Channel)

1. 模块的屏蔽逻辑失效（因为 `sh_en & in_shadow_wr` 为 0）。
2. 上游的数据和 `s_wvalid` 毫无保留地透传给下游（`m_wvalid`）。
3. 下游的 `m_wready` 直接连给上游的 `s_wready`。上游和下游直接进行数据传输。本地 SRAM 完全不动作。

#### 第三阶段：写响应穿透 (B Channel)

1. 写入完毕后，下游（Master端）会发出写响应（`m_bvalid`、`m_bid`、`m_bresp`）。
2. 模块的输出 Mux 将这些信号直接透传给上游的 `s_b_xxx` 接口。
3. 上游的 `s_bready` 透传给 `m_bready`，完成响应通道的握手。

## 2.7 读数据

### 场景一：命中拦截区（Shadowed Read / 影子读）

当上游发起读请求，且地址命中 LPF 区域（`lpf_rd_hit == 1`）时，模块**不会将读请求发给下游**，而是直接从刚才存好的本地单端口 SRAM 中把数据挖出来，自己组装成 AXI 读响应返回给上游。

完整的流程由内部的 **读状态机（3个状态：R_IDLE -> R_ISSUE -> R_STREAM）** 驱动：

#### 第一阶段：读地址握手 (AR Channel) - `R_IDLE` 状态

1. **上游发起请求**：上游拉高 `s_arvalid`，送来读取首地址 `s_araddr` 和突发长度 `s_arlen`。
2. **冲突检测**：模块检测到地址命中，此时必须检查**本地 SRAM 是否空闲**。因为 SRAM 是单端口的（不能同时读写），所以代码里写了 `shr_idle & (~shw_busy)`。只有在影子写操作不在进行时，才允许拉高 `s_arready` 进行握手。
3. **锁存与计算**：握手成功的时钟边沿，状态机从 `R_IDLE` 跳转到 `R_ISSUE`。同时：
    - 锁存读 ID (`shr_arid = s_arid`) 和剩余拍数 (`shr_cnt = s_arlen`)。
    - **计算 SRAM 首地址**：将 `(s_araddr - rd_base) >> 4` 赋给 SRAM 读地址 `shr_ra`。
4. **屏蔽下游**：发往下游的 `m_arvalid` 被逻辑强行屏蔽为 0，下游对此毫不知情。

#### 第二阶段：送出地址与等待 (SRAM 物理延时) - `R_ISSUE` 状态

1. **为什么需要这个状态？** 同步 SRAM 有一个物理特性：**读延迟（Read Latency）**。你在第 1 个时钟周期把地址（`shr_ra`）喂给它，它要在第 2 个时钟周期才能把数据吐出来。
2. **动作**：在 `R_ISSUE` 状态下，SRAM 正在内部寻址。为了不浪费时间，状态机顺手把读地址加了 1 (`shr_ra <= shr_ra + 1`)，**提前为下一拍数据做准备**。
3. **跳转**：无条件跳转到 `R_STREAM` 状态，准备迎接 SRAM 吐出的第一拍数据。

#### 第三阶段：数据流水线返回 (R Channel) - `R_STREAM` 状态

1. **捕获数据**：由于上一拍（或 `R_ISSUE` 时）已经送入了地址，此刻 SRAM 的数据端口 `llb_rd` 吐出了真实数据。代码通过 `shr_rdata <= llb_rd` 将其捕获。
2. **向上游发送数据**：模块拉高本地生成的读有效信号 `shr_rvalid = 1`，将 `shr_rdata`、状态码 OKAY (`2'b00`) 以及读 ID 送给上游（通过多路选择器输出到 `s_r_xxx` 接口）。
3. **完成一拍握手**：
    - 如果上游准备好了（`s_rready == 1`），这一拍数据成功传给上游。
    - 地址继续递增 `shr_ra <= shr_ra + 1`，去 SRAM 里面预读下一拍。
    - 检查是不是最后一拍数据：`shr_rlast <= (shr_cnt == 0)`。
4. **突发结束**：
    - （*注：你提供的代码在这里写了一行注释 `// Note: if shr_cnt hits 0...` 被截断了。完整的逻辑应当是：如果在握手时 `shr_cnt == 0`，代表这是最后一拍，握手完成后，状态机回到 `R_IDLE` 状态，结束本次影子读。*）

### 场景二：未命中 / 直通模式（Forwarded Read / 透传读）

如果读地址不在范围内（`lpf_rd_hit == 0`）或开启了全局直通（`cfg_bypass == 1`），过程非常简单，完全变成了一根物理导线。

#### 第一阶段：读地址穿透 (AR Channel)

1. 拦截逻辑不生效。上游的 `s_arvalid`、`s_araddr` 等所有控制信号全部直通下游 `m_ar_xxx`。
2. 下游的 `m_arready` 直通上游的 `s_arready`，握手成功，读请求交给了 DDR 等外部主存。

#### 第二阶段：读数据穿透 (R Channel)

1. 外部主存寻址完毕后，将数据通过 `m_rvalid`、`m_rdata` 返回。
2. 模块内部的 Mux（多路选择器）发现当前不是影子读状态（`shr_rvalid == 0`），直接切换通道，将 `m_rdata` 透传给上游的 `s_rdata`。
3. 上游拉高 `s_rready` 进行接收，`s_rready` 透传给下游的 `m_rready`，双方完成握手。

### 💡 核心设计亮点总结

纵观整个读过程，有三个非常优秀的硬件设计细节：

1. **流水线读机制 (Pipelined Read)**：在 `R_STREAM` 状态下，模块是一边把当前数据给上游（握手），一边把 `shr_ra+1` 的新地址喂给 SRAM。这样就能实现**连续的背靠背读出**，每一拍只占一个时钟周期，吞吐率拉满。
2. **读写互斥锁 (`~shw_busy`)**：本地只用了一块**单端口 SRAM**（比双端口省一半面积）。单端口意味着同一时刻只能读或者写。因此在 AR 握手时，代码严格要求 `~shw_busy`，确保写入完成后，才允许开启读状态机，完美避免了 SRAM 端口冲突。
3. **数据缓存锁存 (Idempotent Capture)**：在 `R_STREAM` 里有一句 `shr_rdata <= llb_rd;`。如果上游突然卡住（`s_rready == 0`），SRAM 的地址不会增加，下个周期 SRAM 吐出的还是这个数据。寄存器重新捕获同一个数据，保证了哪怕上游反压（Backpressure），数据也绝不会丢失。