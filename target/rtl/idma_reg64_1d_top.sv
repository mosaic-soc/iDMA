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
  parameter int unsigned StreamWidth    = cf_math_pkg::idx_width(NumStreams),
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
  input  cnt_width_t next_id_i,
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

  // arbitration output
  dma_req_t [NumRegs-1:0] arb_dma_req_q;
  logic     [NumRegs-1:0] arb_valid;
  logic     [NumRegs-1:0] arb_ready;
  logic [cf_math_pkg::idx_width(NumRegs)-1:0] arb_idx;

  // per-port launch-pending latch
  logic    [NumRegs-1:0] launch_pending_q;
  stream_t [NumRegs-1:0] held_stream_q;

  // stream of the arbitrated winner, not the last pending port
  assign stream_idx_o = req_valid_o ? held_stream_q[arb_idx] : '0;

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

    // a next_id rd_swacc strobe launches a transfer; latched until the arbiter accepts
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

    // set on the read strobe (or an accept-and-reload in the same cycle), clear on accept
    always_ff @(posedge clk_i or negedge rst_ni) begin : proc_launch_pending
        if (!rst_ni) begin
            launch_pending_q[i] <= 1'b0;
            held_stream_q   [i] <= '0;
            arb_dma_req_q   [i] <= '0;
        end else begin
            if (read_happens && (!launch_pending_q[i] || arb_ready[i])) begin
                launch_pending_q[i] <= 1'b1;
                held_stream_q   [i] <= read_stream;
                arb_dma_req_q   [i] <= nxt_dma_req;
            end else if (launch_pending_q[i] && arb_ready[i]) begin
                launch_pending_q[i] <= 1'b0;
            end
        end
    end

    assign arb_valid[i] = launch_pending_q[i];

    // combinational request struct, captured into arb_dma_req_q at launch time
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

    end

    // observational registers: drive .next (read-side launch is the rd_swacc strobe above)
    for (genvar c = 0; c < NumStreams; c++) begin : gen_hw2reg_connections
        assign dma_hw2reg[i].status[c].busy.next     = {midend_busy_i[c], busy_i[c]};
        assign dma_hw2reg[i].next_id[c].next_id.next = next_id_i;
        assign dma_hw2reg[i].done_id[c].done_id.next = done_id_i[c];
    end

    // tie-off unused channels
    for (genvar c = NumStreams; c < MaxNumStreams; c++) begin : gen_hw2reg_unused
        assign dma_hw2reg[i].status[c].busy.next     = '0;
        assign dma_hw2reg[i].next_id[c].next_id.next = '0;
        assign dma_hw2reg[i].done_id[c].done_id.next = '0;
    end

  end

  // arbitration
  rr_arb_tree #(
    .NumIn     ( NumRegs   ),
    .DataType  ( dma_req_t ),
    .ExtPrio   ( 0         ),
    .AxiVldRdy ( 1         ),
    .LockIn    ( 1         )
  ) i_rr_arb_tree (
    .clk_i,
    .rst_ni,
    .flush_i ( 1'b0        ),
    .rr_i    ( '0          ),
    .req_i   ( arb_valid   ),
    .gnt_o   ( arb_ready   ),
    .data_i  ( arb_dma_req_q ),
    .gnt_i   ( req_ready_i ),
    .req_o   ( req_valid_o ),
    .data_o  ( dma_req_o   ),
    .idx_o   ( arb_idx     )
  );

endmodule

