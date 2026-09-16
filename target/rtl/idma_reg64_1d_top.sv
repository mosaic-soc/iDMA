// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Michael Rogenmoser <michaero@iis.ee.ethz.ch>
// - Thomas Benz <tbenz@iis.ee.ethz.ch>

`include "obi/typedef.svh"

/// Description: Register-based front-end for iDMA
module idma_reg64_1d #(
  /// Number of configuration register ports
  parameter int unsigned NumRegs        = 32'd1,
  /// Number of streams (max 16)
  parameter int unsigned NumStreams     = 32'd1,
  /// Width of the transfer id (max 32-bit)
  parameter int unsigned IdCounterWidth = 32'd32,
  /// Dependent parameter: Stream Idx
  parameter int unsigned StreamWidth     = cc_pkg::idx_width(NumStreams),
  /// Number of launches buffered between the register ports and request output; zero bypasses it
  parameter int unsigned LaunchFifoDepth = NumRegs,
  /// OBI request type
  parameter type         obi_req_t      = logic,
  /// OBI response type
  parameter type         obi_rsp_t      = logic,
  /// DMA 1d or ND burst request type
  parameter type         dma_req_t      = logic,
  /// Dependent type for IdCounterWidth
  parameter type         cnt_width_t    = logic [IdCounterWidth-1:0],
  /// Dependent type for StreamWidth
  parameter type         stream_t       = logic [StreamWidth-1:0]
) (
  input  logic clk_i,
  input  logic rst_ni,
  /// Configuration control slave (obi-flat)
  input  obi_req_t [NumRegs-1:0] dma_ctrl_req_i,
  output obi_rsp_t [NumRegs-1:0] dma_ctrl_rsp_o,
  /// Request signals
  output dma_req_t   dma_req_o,
  output logic       req_valid_o,
  input  logic       req_ready_i,
  /// Current unallocated transfer ID
  input  cnt_width_t next_id_i,
  /// Transfer ID carried with dma_req_o
  output cnt_width_t req_id_o,
  /// Pulse indicating that next_id_i was allocated to a launch
  output logic       id_alloc_o,
  output stream_t    stream_idx_o,
  /// Status signals
  input  cnt_width_t           [NumStreams-1:0] done_id_i,
  input  idma_pkg::idma_busy_t [NumStreams-1:0] busy_i,
  input  logic                 [NumStreams-1:0] midend_busy_i
);

  /// Maximum number of streams is set to 16. It can be enlarged, but the register file
  /// needs to be adapted too.
  localparam int unsigned MaxNumStreams = 32'd16;
  localparam int unsigned RegAddrWidth  = idma_reg64_1d_reg_pkg::IDMA_REG64_1D_REG_TOP_MIN_ADDR_WIDTH;

  // register connections
  idma_reg64_1d_reg_pkg::idma_reg__out_t dma_reg2hw [NumRegs-1:0];
  idma_reg64_1d_reg_pkg::idma_reg__in_t  dma_hw2reg [NumRegs-1:0];

  // A next_id read atomically allocates an ID and queues the corresponding descriptor.  Keeping
  // all three values in one ordered FIFO ensures that IDs are issued in allocation order even
  // when several independent register ports launch transfers.
  typedef struct packed {
    dma_req_t req;
    stream_t  stream;
  } launch_candidate_t;

  typedef struct packed {
    dma_req_t   req;
    stream_t    stream;
    cnt_width_t id;
  } launch_entry_t;

  launch_candidate_t [NumRegs-1:0] launch_candidate;
  logic              [NumRegs-1:0] launch_valid;
  logic              [NumRegs-1:0] launch_grant;
  launch_candidate_t               selected_launch;
  launch_entry_t                   launch_fifo_in, launch_fifo_out;
  logic                            selected_launch_valid;
  logic                            launch_fifo_ready;

  // generate the registers
  for (genvar i = 0; i < NumRegs; i++) begin : gen_core_regs


    // override the reg_top ID width so s_obi_aid/s_obi_rid match the OBI bus id width
    idma_reg64_1d_reg_top #(
      .ID_WIDTH ( $bits(dma_ctrl_req_i[i].a.aid) )
    ) i_idma_reg64_1d_reg_top (
      .clk    ( clk_i ),
      .arst_n ( rst_ni ),

      .s_obi_req     ( dma_ctrl_req_i[i].req                     ),
      .s_obi_gnt     ( dma_ctrl_rsp_o[i].gnt                     ),
      .s_obi_addr    ( dma_ctrl_req_i[i].a.addr[RegAddrWidth-1:0] ),
      .s_obi_we      ( dma_ctrl_req_i[i].a.we                    ),
      .s_obi_be      ( dma_ctrl_req_i[i].a.be                    ),
      .s_obi_wdata   ( dma_ctrl_req_i[i].a.wdata                 ),
      .s_obi_aid     ( dma_ctrl_req_i[i].a.aid                   ),
      .s_obi_rvalid  ( dma_ctrl_rsp_o[i].rvalid                  ),
      .s_obi_rready  ( dma_ctrl_req_i[i].rready                  ),
      .s_obi_rdata   ( dma_ctrl_rsp_o[i].r.rdata                 ),
      .s_obi_err     ( dma_ctrl_rsp_o[i].r.err                   ),
      .s_obi_rid     ( dma_ctrl_rsp_o[i].r.rid                   ),

      .hwif_out  ( dma_reg2hw       [i] ),
      .hwif_in   ( dma_hw2reg       [i] )
    );

    // A next_id rd_swacc strobe attempts a launch; arbitration decides its returned ID.
    logic     read_happens;
    stream_t  read_stream;
    dma_req_t nxt_dma_req;

    always_comb begin : proc_launch
        read_happens = 1'b0;
        read_stream  = '0;
        for (int c = 0; c < NumStreams; c++) begin
            if (dma_reg2hw[i].next_id[c].next_id.rd_swacc) begin
                read_happens = 1'b1;
                read_stream  = c;
            end
        end
    end

    assign launch_valid[i]     = read_happens;
    assign launch_candidate[i] = '{req: nxt_dma_req, stream: read_stream};

    // Combinational descriptor snapshot presented to the centralized launch allocator.
    always_comb begin : proc_hw_req_conv
      // all fields are zero per default
      nxt_dma_req = '0;

      // address and length
      nxt_dma_req.length   = {dma_reg2hw[i].length[1].length.value,     dma_reg2hw[i].length[0].length.value};
      nxt_dma_req.src_addr = {dma_reg2hw[i].src_addr[1].src_addr.value, dma_reg2hw[i].src_addr[0].src_addr.value};
      nxt_dma_req.dst_addr = {dma_reg2hw[i].dst_addr[1].dst_addr.value, dma_reg2hw[i].dst_addr[0].dst_addr.value};

      // Protocols
      nxt_dma_req.opt.src_protocol = idma_pkg::protocol_e'(dma_reg2hw[i].conf.src_protocol.value);
      nxt_dma_req.opt.dst_protocol = idma_pkg::protocol_e'(dma_reg2hw[i].conf.dst_protocol.value);

      // Current backend only supports incremental burst
      nxt_dma_req.opt.src.burst = axi_pkg::BURST_INCR;
      nxt_dma_req.opt.dst.burst = axi_pkg::BURST_INCR;
        // this frontend currently does not support cache variations
      nxt_dma_req.opt.src.cache = axi_pkg::CACHE_MODIFIABLE;
      nxt_dma_req.opt.dst.cache = axi_pkg::CACHE_MODIFIABLE;

      // Backend options
      nxt_dma_req.opt.beo.decouple_aw    = dma_reg2hw[i].conf.decouple_aw.value;
      nxt_dma_req.opt.beo.decouple_rw    = dma_reg2hw[i].conf.decouple_rw.value;
      nxt_dma_req.opt.beo.src_max_llen   = dma_reg2hw[i].conf.src_max_llen.value;
      nxt_dma_req.opt.beo.dst_max_llen   = dma_reg2hw[i].conf.dst_max_llen.value;
      nxt_dma_req.opt.beo.src_reduce_len = dma_reg2hw[i].conf.src_reduce_len.value;
      nxt_dma_req.opt.beo.dst_reduce_len = dma_reg2hw[i].conf.dst_reduce_len.value;
      nxt_dma_req.opt.beo.deadlock_free  = dma_reg2hw[i].conf.deadlock_free.value;

      // Optional on-the-fly compute settings are part of the transfer descriptor and
      // are captured together with the address/stride fields when next_id is read.
      nxt_dma_req.opt.compute.enable                    =
          dma_reg2hw[i].compute_cfg.compute_enable.value;
      nxt_dma_req.opt.compute.op                        =
          idma_pkg::compute_op_e'(dma_reg2hw[i].compute_cfg.compute_op.value);
      nxt_dma_req.opt.compute.params.transpose.mode     =
          dma_reg2hw[i].compute_cfg.transpose_mode.value;
      nxt_dma_req.opt.compute.params.transpose.tensor_m =
          dma_reg2hw[i].compute_cfg.transpose_tensor_m.value;
      nxt_dma_req.opt.compute.params.transpose.tensor_n =
          dma_reg2hw[i].compute_cfg.transpose_tensor_n.value;
      // Compact mode removes tile padding from destination rows.
      nxt_dma_req.opt.compute.params.transpose.compact  =
          dma_reg2hw[i].compute_cfg.transpose_compact.value;

    end

    // observational registers: drive .next (read-side launch is the rd_swacc strobe above)
    for (genvar c = 0; c < NumStreams; c++) begin : gen_hw2reg_connections
        assign dma_hw2reg[i].status[c].busy.next     = {midend_busy_i[c], busy_i[c]};
        // ID zero reports that this launch lost arbitration or that the launch FIFO was full.
        assign dma_hw2reg[i].next_id[c].next_id.next = launch_grant[i] ? next_id_i : '0;
        assign dma_hw2reg[i].done_id[c].done_id.next = done_id_i[c];
    end

    // tie-off unused channels
    for (genvar c = NumStreams; c < MaxNumStreams; c++) begin : gen_hw2reg_unused
        assign dma_hw2reg[i].status[c].busy.next     = '0;
        assign dma_hw2reg[i].next_id[c].next_id.next = '0;
        assign dma_hw2reg[i].done_id[c].done_id.next = '0;
    end

  end

  // At most one register port allocates an ID per cycle.  Other simultaneous reads complete with
  // ID zero and may be retried, keeping the register interfaces non-blocking without requiring a
  // multi-write FIFO or duplicated ID-counter arithmetic.
  cc_rr_arb_tree #(
    .NumIn     ( NumRegs   ),
    .data_t    ( launch_candidate_t ),
    .ExtPrio   ( 0         ),
    // Launch strobes are one-cycle events rather than held valid/ready streams. A request that
    // cannot be granted immediately completes with ID zero, so the arbiter must not lock it in.
    .AxiVldRdy ( 0         ),
    .LockIn    ( 0         )
  ) i_rr_arb_tree (
    .clk_i,
    .rst_ni,
    .clr_i   ( 1'b0        ),
    .rr_i    ( '0          ),
    .req_i   ( launch_valid          ),
    .gnt_o   ( launch_grant          ),
    .data_i  ( launch_candidate      ),
    .gnt_i   ( launch_fifo_ready     ),
    .req_o   ( selected_launch_valid ),
    .data_o  ( selected_launch       ),
    .idx_o   ( /* unused */          )
  );

  always_comb begin : proc_launch_fifo_input
    launch_fifo_in        = '0;
    launch_fifo_in.req    = selected_launch.req;
    launch_fifo_in.stream = selected_launch.stream;
    launch_fifo_in.id     = next_id_i;
  end

  assign id_alloc_o = selected_launch_valid & launch_fifo_ready;

  if (LaunchFifoDepth == 0) begin : gen_launch_bypass
    // Without frontend buffering, a launch is allocated only if the downstream request interface
    // accepts it immediately. Otherwise its next_id read returns zero and software retries it.
    assign launch_fifo_ready = req_ready_i;
    assign launch_fifo_out   = launch_fifo_in;
    assign req_valid_o       = selected_launch_valid;
  end else begin : gen_launch_fifo
    cc_stream_fifo #(
      .FallThrough ( 1'b0            ),
      .Depth       ( LaunchFifoDepth ),
      .data_t      ( launch_entry_t  )
    ) i_launch_fifo (
      .clk_i,
      .rst_ni,
      .clr_i   ( 1'b0                  ),
      .flush_i ( 1'b0                  ),
      .usage_o ( /* unused */          ),
      .data_i  ( launch_fifo_in        ),
      .valid_i ( selected_launch_valid ),
      .ready_o ( launch_fifo_ready     ),
      .data_o  ( launch_fifo_out       ),
      .valid_o ( req_valid_o           ),
      .ready_i ( req_ready_i           )
    );
  end

  assign dma_req_o    = launch_fifo_out.req;
  assign stream_idx_o = launch_fifo_out.stream;
  assign req_id_o     = launch_fifo_out.id;

  `ASSERT(OneLaunchAllocated, $onehot0(launch_grant), clk_i, !rst_ni,
      "At most one register port may allocate a transfer ID per cycle")

endmodule

