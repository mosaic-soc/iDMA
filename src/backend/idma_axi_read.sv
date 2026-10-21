// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Thomas Benz <tbenz@iis.ee.ethz.ch>
// - Tobias Senti <tsenti@ethz.ch>

`include "common_cells/registers.svh"
`include "common_cells/assertions.svh"

/// Implementing the AXI4 read task in the iDMA transport layer.
module idma_axi_read #(
    /// Stobe width
    parameter int unsigned StrbWidth = 32'd16,

    /// Byte type
    parameter type byte_t = logic,
    /// Offset type
    parameter type strb_t = logic,
    /// Byte-lane index type
    parameter type offset_t = logic,
    /// Payload byte-count type
    parameter type tf_len_t = logic,

    /// AXI 4 Request channel type
    parameter type read_req_t = logic,
    /// AXI 4 Response channel type
    parameter type read_rsp_t = logic,

    /// `r_dp_req_t` type:
    parameter type r_dp_req_t = logic,
    /// `r_dp_rsp_t` type:
    parameter type r_dp_rsp_t = logic,
    /// AXI 4 `AR` channel type
    parameter type ar_chan_t = logic
)(
    /// Clock
    input  logic clk_i,
    /// Asynchronous reset, active low
    input  logic rst_ni,

    /// Read datapath request
    input  r_dp_req_t r_dp_req_i,
    /// Read datapath request valid
    input  logic r_dp_valid_i,
    /// Read datapath request ready
    output logic r_dp_ready_o,

    /// Read datapath response
    output r_dp_rsp_t r_dp_rsp_o,
    /// Read datapath response valid
    output logic r_dp_valid_o,
    /// Read datapath response valid
    input  logic r_dp_ready_i,

    /// Read meta request
    input  ar_chan_t ar_req_i,
    /// Read meta request valid
    input  logic ar_valid_i,
    /// Read meta request ready
    output logic ar_ready_o,

    /// AXI4+ATOP read manager port request
    output read_req_t read_req_o,
    /// AXI4+ATOP read manager port response
    input  read_rsp_t read_rsp_i,

    /// Raw read data; the transport layer applies buffer_shift_o
    output byte_t [StrbWidth-1:0] buffer_in_o,
    /// Current byte rotation, shared by the data and valid-mask shifters
    output offset_t buffer_shift_o,
    /// Valid mask in FIFO-lane coordinates, after rotation
    output strb_t buffer_in_valid_o,
    /// A response beat the read side can take, before the buffer masks and readiness
    output logic buffer_in_beat_o,
    /// Ready from Buffer
    input  strb_t buffer_in_ready_i
);
    `include "idma/byte_lane_functions.svh"
    // Physical AXI lanes carrying payload in the current beat, before FIFO alignment.
    strb_t read_aligned_in_mask;

    // On the first beat, use the queued request directly to avoid a setup cycle.
    logic first_r_d, first_r_q;
    offset_t bus_lane_d, bus_lane_q, bus_lane;
    offset_t fifo_lane_d, fifo_lane_q, fifo_lane;
    tf_len_t bytes_left_d, bytes_left_q, bytes_left;
    // One extra bit represents a full bus word, which does not fit in an index.
    idma_beat_bytes_t bytes_to_size_boundary, payload_bytes;

    // FIFO lane mask, rotated identically to the data in the transport layer.
    strb_t mask_in;

    // inbound control signals to the read buffer: controlled by the read process
    logic  in_valid;
    logic  in_ready;

    //--------------------------------------
    // Read meta channel
    //--------------------------------------
    // connect the ar requests to the AXI bus
    assign read_req_o.ar       = ar_req_i.axi.ar_chan;
    assign read_req_o.ar_valid = ar_valid_i;
    assign ar_ready_o          = read_rsp_i.ar_ready;


    //--------------------------------------
    // In mask generation
    //--------------------------------------
    assign bus_lane   = first_r_q ? r_dp_req_i.offset          : bus_lane_q;
    assign fifo_lane  = first_r_q ? r_dp_req_i.fifo_start_lane : fifo_lane_q;
    assign bytes_left = first_r_q ? r_dp_req_i.num_bytes       : bytes_left_q;
    assign bytes_to_size_boundary = idma_bytes_to_size_boundary(
        bus_lane, r_dp_req_i.size);
    assign payload_bytes = bytes_left < tf_len_t'(bytes_to_size_boundary)
        ? idma_beat_bytes_t'(bytes_left) : bytes_to_size_boundary;

    // Each beat uses only its size-selected AXI lanes. The first beat can have a
    // prefix and the final beat excess bytes; neither belongs to the FIFO stream.
    assign read_aligned_in_mask = idma_payload_mask(bus_lane, int'(payload_bytes));


    //--------------------------------------
    // Barrel shifter
    //--------------------------------------
    // The transport layer rotates the raw data with this same shift. FIXED beats
    // repeat their bus lanes while their payload advances through the FIFO lanes.
    assign buffer_in_o = read_rsp_i.r.data;
    assign buffer_shift_o = idma_lane_shift(bus_lane, fifo_lane);
    assign mask_in = strb_t'({read_aligned_in_mask, read_aligned_in_mask} >> buffer_shift_o);


    //--------------------------------------
    // Read control
    //--------------------------------------
    always_comb begin : proc_read_state
        first_r_d = first_r_q;
        bus_lane_d = bus_lane_q;
        fifo_lane_d = fifo_lane_q;
        bytes_left_d = bytes_left_q;
        if (read_rsp_i.r_valid && read_req_o.r_ready) begin
            first_r_d = read_rsp_i.r.last;
            bus_lane_d = bus_lane;
            if (r_dp_req_i.burst == axi_pkg::BURST_INCR) begin
                bus_lane_d = bus_lane + offset_t'(bytes_to_size_boundary);
            end
            fifo_lane_d = fifo_lane + offset_t'(payload_bytes);
            bytes_left_d = bytes_left - tf_len_t'(payload_bytes);
        end
    end

    // the buffer can be pushed to if all the masked FIFO buffers (mask_in) are ready.
    assign in_ready = &(buffer_in_ready_i | ~mask_in);
    // the read can accept data if the buffer is ready and the response channel is ready
    assign read_req_o.r_ready = r_dp_valid_i & in_ready & r_dp_ready_i;

    // once valid data is applied, it can be pushed in all the selected (mask_in) buffers
    // be sure the response channel is ready
    assign in_valid          = read_rsp_i.r_valid & read_req_o.r_ready;
    assign buffer_in_valid_o = in_valid ? mask_in : '0;
    assign buffer_in_beat_o  = r_dp_valid_i & read_rsp_i.r_valid & r_dp_ready_i;

    // r_dp_ready_o is triggered by the last element arriving from the read
    assign r_dp_ready_o = r_dp_valid_i & r_dp_ready_i &
                          read_rsp_i.r.last & read_rsp_i.r_valid & in_ready;

    // connect r_dp response payload
    assign r_dp_rsp_o.resp  = read_rsp_i.r.resp;
    assign r_dp_rsp_o.last  = read_rsp_i.r.last;
    assign r_dp_rsp_o.first = first_r_q;

    // r_dp_valid_o is triggered once the last element is here or an error occurs
    assign r_dp_valid_o = r_dp_valid_i & read_rsp_i.r_valid & in_ready & (read_rsp_i.r.last
        | (|read_rsp_i.r.resp));

    //--------------------------------------
    // Unused AXI signals
    //--------------------------------------
    assign read_req_o.aw_valid = 1'b0;
    assign read_req_o.w_valid  = 1'b0;
    assign read_req_o.b_ready  = 1'b0;

    assign read_req_o.aw = '0;
    assign read_req_o.w  = '0;

    //--------------------------------------
    // State
    //--------------------------------------
    `FF(first_r_q,     first_r_d,     '1, clk_i, rst_ni)
    `FF(bus_lane_q,    bus_lane_d,    '0, clk_i, rst_ni)
    `FF(fifo_lane_q,   fifo_lane_d,   '0, clk_i, rst_ni)
    `FF(bytes_left_q,  bytes_left_d,  '0, clk_i, rst_ni)

    `ASSERT(ReadSizeFits, r_dp_valid_i |-> int'(r_dp_req_i.size) <= $clog2(StrbWidth),
            clk_i, !rst_ni, "AXI read size must not exceed the physical bus width")
    `ASSERT(ReadPayloadNonzero, read_rsp_i.r_valid && read_req_o.r_ready |-> payload_bytes != '0,
            clk_i, !rst_ni, "Accepted AXI read beats must contain payload bytes")
    `ASSERT(ReadLastMatchesLength, read_rsp_i.r_valid && read_req_o.r_ready |->
                read_rsp_i.r.last == (bytes_left == tf_len_t'(payload_bytes)),
            clk_i, !rst_ni, "AXI RLAST must coincide with the final payload byte")
    // Count selected bytes, not AXI beat width: prefix and final excess bytes must
    // never become FIFO entries or consume reservation credits.
    `ASSERT(ReadByteConservation, in_valid |->
                $countones(buffer_in_valid_o) == int'(payload_bytes) &&
                tf_len_t'(payload_bytes) <= bytes_left,
            clk_i, !rst_ni, "An AXI read beat did not push exactly its remaining payload bytes")
    `ASSERT(ReadPushOnlyOnHandshake, !in_valid |-> buffer_in_valid_o == '0,
            clk_i, !rst_ni, "AXI read bytes entered the FIFO without an R handshake")
    `ASSERT(ReadPayloadWithinSize, r_dp_valid_i |->
                int'(beat_offset) + int'(payload_bytes) <= int'(beat_bytes),
            clk_i, !rst_ni, "AXI read payload extends beyond the selected-size access")
    `ASSERT(ReadCursorProgress, in_valid |=>
                fifo_lane_q == offset_t'($past(fifo_lane) + $past(payload_bytes)) &&
                bytes_left_q == $past(bytes_left) - tf_len_t'($past(payload_bytes)) &&
                bus_lane_q == ($past(r_dp_req_i.burst) == axi_pkg::BURST_INCR
                    ? offset_t'($past(bus_lane) + $past(bytes_to_size_boundary)) : $past(bus_lane)),
            clk_i, !rst_ni, "AXI read FIFO cursor or remaining byte count advanced incorrectly")
    `ASSERT(ReadStateStableWhenStalled,
            !in_valid |=>
                $stable({first_r_q, bus_lane_q, fifo_lane_q, bytes_left_q}),
            clk_i, !rst_ni, "AXI read packing state changed without accepting a beat")

endmodule
