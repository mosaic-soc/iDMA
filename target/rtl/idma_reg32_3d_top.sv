// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Michael Rogenmoser <michaero@iis.ee.ethz.ch>
// - Thomas Benz <tbenz@iis.ee.ethz.ch>

`include "obi/typedef.svh"

/// Description: Register-based front-end for iDMA
module idma_reg32_3d #(
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
  localparam int unsigned RegAddrWidth  = idma_reg32_3d_reg_pkg::IDMA_REG32_3D_REG_TOP_MIN_ADDR_WIDTH;

  // register connections
  idma_reg32_3d_reg_pkg::idma_reg__out_t dma_reg2hw [NumRegs-1:0];
  idma_reg32_3d_reg_pkg::idma_reg__in_t  dma_hw2reg [NumRegs-1:0];

  // arbitration output
  typedef struct packed {
    dma_req_t req;
    stream_t  stream_idx;
  } arb_payload_t;

  arb_payload_t [NumRegs-1:0] arb_payload;
  arb_payload_t               arb_payload_out;
  stream_t      [NumRegs-1:0] arb_stream_idx;
  logic         [NumRegs-1:0] arb_valid;
  logic         [NumRegs-1:0] arb_ready;

  assign dma_req_o    = arb_payload_out.req;
  assign stream_idx_o = arb_payload_out.stream_idx;

  // generate the registers
  for (genvar i = 0; i < NumRegs; i++) begin : gen_core_regs


    // override the reg_top ID width so s_obi_aid/s_obi_rid match the OBI bus id width
    idma_reg32_3d_reg_top #(
      .ID_WIDTH ( $bits(dma_ctrl_req_i[i].a.aid) )
    ) i_idma_reg32_3d_reg_top (
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

    logic read_happens;
    // launch-stall: hold the reg read-ack until the arbiter accepts the request
    // (protocol-agnostic — driven into hwif rd_ack below, see gen_hw2reg_connections)

    always_comb begin : proc_launch
        read_happens = 1'b0;
        arb_stream_idx[i] = '0;
        for (int c = 0; c < NumStreams; c++) begin
            if (dma_reg2hw[i].next_id[c].req & ~dma_reg2hw[i].next_id[c].req_is_wr) begin
                read_happens = 1'b1;
                arb_stream_idx[i] = stream_t'(c);
            end
        end
        arb_valid[i] = read_happens;
    end

    // assign request struct
    always_comb begin : proc_hw_req_conv
      // all fields are zero per default
      arb_payload[i] = '0;
      arb_payload[i].stream_idx = arb_stream_idx[i];

      // address and length
      arb_payload[i].req.burst_req.length   = dma_reg2hw[i].length[0].length.value;
      arb_payload[i].req.burst_req.src_addr = dma_reg2hw[i].src_addr[0].src_addr.value;
      arb_payload[i].req.burst_req.dst_addr = dma_reg2hw[i].dst_addr[0].dst_addr.value;

      // Protocols
      arb_payload[i].req.burst_req.opt.src_protocol = idma_pkg::protocol_e'(dma_reg2hw[i].conf.src_protocol.value);
      arb_payload[i].req.burst_req.opt.dst_protocol = idma_pkg::protocol_e'(dma_reg2hw[i].conf.dst_protocol.value);

      // Current backend only supports incremental burst
      arb_payload[i].req.burst_req.opt.src.burst = axi_pkg::BURST_INCR;
      arb_payload[i].req.burst_req.opt.dst.burst = axi_pkg::BURST_INCR;
        // this frontend currently does not support cache variations
      arb_payload[i].req.burst_req.opt.src.cache = axi_pkg::CACHE_MODIFIABLE;
      arb_payload[i].req.burst_req.opt.dst.cache = axi_pkg::CACHE_MODIFIABLE;

      // Backend options
      arb_payload[i].req.burst_req.opt.beo.decouple_aw    = dma_reg2hw[i].conf.decouple_aw.value;
      arb_payload[i].req.burst_req.opt.beo.decouple_rw    = dma_reg2hw[i].conf.decouple_rw.value;
      arb_payload[i].req.burst_req.opt.beo.src_max_llen   = dma_reg2hw[i].conf.src_max_llen.value;
      arb_payload[i].req.burst_req.opt.beo.dst_max_llen   = dma_reg2hw[i].conf.dst_max_llen.value;
      arb_payload[i].req.burst_req.opt.beo.src_reduce_len = dma_reg2hw[i].conf.src_reduce_len.value;
      arb_payload[i].req.burst_req.opt.beo.dst_reduce_len = dma_reg2hw[i].conf.dst_reduce_len.value;

      // ND connections
      arb_payload[i].req.d_req[0].reps = dma_reg2hw[i].dim[0].reps[0].reps.value;
      arb_payload[i].req.d_req[0].src_strides = dma_reg2hw[i].dim[0].src_stride[0].src_stride.value;
      arb_payload[i].req.d_req[0].dst_strides = dma_reg2hw[i].dim[0].dst_stride[0].dst_stride.value;
      arb_payload[i].req.d_req[1].reps = dma_reg2hw[i].dim[1].reps[0].reps.value;
      arb_payload[i].req.d_req[1].src_strides = dma_reg2hw[i].dim[1].src_stride[0].src_stride.value;
      arb_payload[i].req.d_req[1].dst_strides = dma_reg2hw[i].dim[1].dst_stride[0].dst_stride.value;

      // Disable higher dimensions
      if ( dma_reg2hw[i].conf.enable_nd.value == 0) begin
        arb_payload[i].req.d_req[0].reps = '0;
        arb_payload[i].req.d_req[1].reps = 'd1;
      end
      else if ( dma_reg2hw[i].conf.enable_nd.value == 1) begin
        arb_payload[i].req.d_req[1].reps = 'd1;
      end
    end

    // observational registers
    for (genvar c = 0; c < NumStreams; c++) begin : gen_hw2reg_connections
        assign dma_hw2reg[i].status[c].rd_data.busy  = {midend_busy_i[c], busy_i[c]};
        assign dma_hw2reg[i].status[c].rd_ack = dma_reg2hw[i].status[c].req
                                              & ~dma_reg2hw[i].status[c].req_is_wr;
        assign dma_hw2reg[i].next_id[c].rd_data.next_id = next_id_i;
        assign dma_hw2reg[i].next_id[c].rd_ack = dma_reg2hw[i].next_id[c].req
                                               & ~dma_reg2hw[i].next_id[c].req_is_wr
                                               & arb_ready[i];
        assign dma_hw2reg[i].done_id[c].rd_data.done_id = done_id_i[c];
        assign dma_hw2reg[i].done_id[c].rd_ack = dma_reg2hw[i].done_id[c].req
                                               & ~dma_reg2hw[i].done_id[c].req_is_wr;
    end

    // tie-off unused channels
    for (genvar c = NumStreams; c < MaxNumStreams; c++) begin : gen_hw2reg_unused
        assign dma_hw2reg[i].status[c].rd_data = '0;
        assign dma_hw2reg[i].status[c].rd_ack  = '0;
        assign dma_hw2reg[i].next_id[c].rd_data.next_id = '0;
        assign dma_hw2reg[i].next_id[c].rd_ack = '0;
        assign dma_hw2reg[i].done_id[c].rd_data.done_id = '0;
        assign dma_hw2reg[i].done_id[c].rd_ack = '0;
    end

  end

  // arbitration
  rr_arb_tree #(
    .NumIn     ( NumRegs   ),
    .DataType  ( arb_payload_t ),
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
    .data_i  ( arb_payload ),
    .gnt_i   ( req_ready_i ),
    .req_o   ( req_valid_o ),
    .data_o  ( arb_payload_out ),
    .idx_o   ( /* NC */    )
  );

endmodule

