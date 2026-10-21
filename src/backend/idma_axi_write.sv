// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Thomas Benz <tbenz@iis.ee.ethz.ch>
// - Tobias Senti <tsenti@ethz.ch>

`include "common_cells/registers.svh"
`include "common_cells/assertions.svh"

/// Implementing the AXI4 write task in the iDMA transport layer.
module idma_axi_write #(
    /// Stobe width
    parameter int unsigned StrbWidth = 32'd16,
    /// Mask invalid data on the manager interface
    parameter bit MaskInvalidData = 1'b1,

    /// Byte type
    parameter type byte_t = logic,
    /// Data type
    parameter type data_t = logic,
    /// Offset type
    parameter type strb_t = logic,
    /// Physical bus and FIFO lane index type
    parameter type offset_t = logic,
    /// Payload byte-count type
    parameter type tf_len_t = logic,

    /// AXI 4 Request channel type
    parameter type write_req_t = logic,
    /// AXI 4 Response channel type
    parameter type write_rsp_t = logic,

    /// `w_dp_req_t` type:
    parameter type w_dp_req_t = logic,
    /// `w_dp_rsp_t` type:
    parameter type w_dp_rsp_t = logic,
    /// AXI 4 `AW` channel type
    parameter type aw_chan_t = logic
)(
    /// Clock
    input  logic clk_i,
    /// Asynchronous reset, active low
    input  logic rst_ni,

    /// Write datapath request
    input  w_dp_req_t w_dp_req_i,
    /// Write datapath request valid
    input  logic w_dp_valid_i,
    /// Write datapath request ready
    output logic w_dp_ready_o,

    /// Datapath poison signal
    input  logic dp_poison_i,

    /// Write datapath response
    output w_dp_rsp_t w_dp_rsp_o,
    /// Write datapath response valid
    output logic w_dp_valid_o,
    /// Write datapath response valid
    input  logic w_dp_ready_i,

    /// Write meta request
    input  aw_chan_t aw_req_i,
    /// Write meta request valid
    input  logic aw_valid_i,
    /// Write meta request ready
    output logic aw_ready_o,

    /// AXI4+ATOP write manager port request
    output write_req_t write_req_o,
    /// AXI4+ATOP write manager port response
    input  write_rsp_t write_rsp_i,

    output logic w_chan_valid_o,
    output logic w_chan_ready_o,
    output logic w_chan_first_o,

    /// Data from buffer
    input  byte_t [StrbWidth-1:0] buffer_out_i,
    /// Valid from buffer
    input  strb_t buffer_out_valid_i,
    /// Ready to buffer
    output strb_t buffer_out_ready_o,
    /// Lanes the next write beat pops, from the request and the burst state only
    output strb_t buffer_out_offer_o,
    /// Logical byte positions consumed, before applying the external write-strobe mask
    output strb_t buffer_out_consumed_o,
    /// Rotation from FIFO positions to this beat's physical bus lanes
    output offset_t buffer_shift_o,
    /// External write-strobe mask (ANDed into wstrb); tie to '1 when unused
    input  strb_t mask_ext_i
);
    `include "idma/byte_lane_functions.svh"
    // corresponds to the strobe: the write aligned data that is currently valid in the buffer
    strb_t mask_out;
    // Byte positions belonging to this write beat, before suppressing compute-invalid bytes.
    strb_t logical_mask_out;

    // write signals: is this the first / last element in a burst?
    logic first_w_d, first_w_q;
    logic last_w;

    // buffer is ready to write the requested data
    logic ready_to_write;
    // write happens
    logic write_happening;

    // A temporary signal required to write the output of the buffer to before assigning it to
    // the AXI bus. This is required to be compatible with some of the Questasim Versions and some
    // of the parametrizations (e.g. DataWidth = 16)
    data_t buffer_data_masked;

    // The queued request supplies the first beat without a setup cycle. Thereafter,
    // only accepted W beats advance the physical lane, FIFO lane, and byte count.
    offset_t bus_lane_d, bus_lane_q, bus_lane;
    offset_t fifo_lane_d, fifo_lane_q, fifo_lane;
    tf_len_t bytes_left_d, bytes_left_q, bytes_left;
    idma_beat_bytes_t bytes_to_size_boundary, payload_bytes;

    //--------------------------------------
    // Mask pre-calculation
    //--------------------------------------
    assign bus_lane   = first_w_q ? w_dp_req_i.offset          : bus_lane_q;
    assign fifo_lane  = first_w_q ? w_dp_req_i.fifo_start_lane : fifo_lane_q;
    assign bytes_left = first_w_q ? w_dp_req_i.num_bytes       : bytes_left_q;
    assign bytes_to_size_boundary = idma_bytes_to_size_boundary(
        bus_lane, w_dp_req_i.size);
    assign payload_bytes = bytes_left < tf_len_t'(bytes_to_size_boundary)
        ? idma_beat_bytes_t'(bytes_left) : bytes_to_size_boundary;
    // Data and valid rotate forward in the transport layer; ready/offer/consumed
    // rotate back by the same amount. FIXED repeats bus lanes, not FIFO lanes.
    assign buffer_shift_o = idma_lane_shift(fifo_lane, bus_lane);

    //--------------------------------------
    // Write meta channel
    //--------------------------------------
    // connect the aw requests to the AXI bus
    assign write_req_o.aw       = aw_req_i.axi.aw_chan;
    assign write_req_o.aw_valid = aw_valid_i;
    assign aw_ready_o           = write_rsp_i.aw_ready;


    //--------------------------------------
    // Out mask generation -> (wstrb mask)
    //--------------------------------------
    // only pop the data actually needed for write from the buffer,
    // determine valid data to pop by calculation the wstrb
    assign logical_mask_out = idma_payload_mask(bus_lane, int'(payload_bytes));
    // external mask (OTF compute)
    assign mask_out = logical_mask_out & mask_ext_i;


    //--------------------------------------
    // Write control
    //--------------------------------------
    // write is decoupled from read, due to misalignment in the read/write
    // addresses, page crossing can be encountered at any time.
    // To handle this efficiently, a 2-to-1 or 1-to-2 mapping of r/w beats
    // is required. The write unit independently tracks progress through
    // its remaining logical bytes and cannot use `r last` for that.

    // Only the current beat's payload lanes must be available. For compute, the
    // external mask may suppress every lane; still require an offered engine beat
    // so zero-strobe writes cannot run ahead of the compute stream.
    assign ready_to_write = w_dp_valid_i & ((buffer_out_valid_i & mask_out) == mask_out)
        & (buffer_out_valid_i != '0);

    // write happening: both the bus (w_ready) and the buffer (ready_to_write) is high
    assign write_happening = ready_to_write & write_rsp_i.w_ready;

    // the main buffer is conditionally to the write mask popped
    assign buffer_out_ready_o = write_happening ? mask_out : '0;
    // Independent of FIFO occupancy, allowing its lookahead logic to accept bytes
    // for precisely those lanes that the current beat can drain.
    assign buffer_out_offer_o = w_dp_valid_i ? mask_out : '0;
    // Compute beats track all logical positions, including those suppressed by their strobe.
    assign buffer_out_consumed_o = write_happening ? logical_mask_out : '0;

    // signal the bus that we are ready
    assign write_req_o.w_valid = ready_to_write;

    // connect data and strobe either directly or mask invalid data
    if (MaskInvalidData) begin : gen_mask_invalid_data

        // always_comb process implements masking of invalid data
        always_comb begin : proc_mask
            // defaults
            write_req_o.w.data = '0;
            write_req_o.w.strb = '0;
            buffer_data_masked = '0;
            // control the write to the bus apply data to the bus only if data should be written
            if (ready_to_write == 1'b1 & !dp_poison_i) begin
                // assign data from buffers, mask non valid entries
                for (int i = 0; i < StrbWidth; i++) begin
                    buffer_data_masked[i*8 +: 8] = mask_out[i] ? buffer_out_i[i] : 8'b0;
                end
                // assign the output
                write_req_o.w.data = buffer_data_masked;
                // assign the out mask to the strobe
                write_req_o.w.strb = mask_out;
            end
        end

    end else begin : gen_direct_connect
        // not used signal
        assign buffer_data_masked = '0;
        // simpler: direct connection
        assign write_req_o.w.data = buffer_out_i;
        assign write_req_o.w.strb = dp_poison_i ? '0 : mask_out;
    end

    // the w last signal should only be applied to the bus if an actual transfer happens
    assign write_req_o.w.last = last_w & ready_to_write;

    // we are ready for the next transfer internally, once the w last signal is applied
    assign w_dp_ready_o = last_w & write_happening;

    // Completion follows logical payload bytes, including compute positions whose
    // external strobe is zero. No separate beat counter or single-beat case is needed.
    assign last_w = bytes_left == tf_len_t'(payload_bytes);

    always_comb begin : proc_write_positions
        first_w_d = first_w_q;
        bus_lane_d = bus_lane_q;
        fifo_lane_d = fifo_lane_q;
        bytes_left_d = bytes_left_q;
        if (write_happening) begin
            first_w_d = last_w;
            bus_lane_d = bus_lane;
            if (w_dp_req_i.burst == axi_pkg::BURST_INCR) begin
                bus_lane_d = bus_lane + offset_t'(bytes_to_size_boundary);
            end
            fifo_lane_d = fifo_lane + offset_t'(payload_bytes);
            bytes_left_d = bytes_left - tf_len_t'(payload_bytes);
        end
    end

    // Channel management signals
    assign w_chan_valid_o = write_req_o.w_valid;
    assign w_chan_ready_o = write_rsp_i.w_ready;
    assign w_chan_first_o = first_w_q;

    //--------------------------------------
    // Write response
    //--------------------------------------
    // connect w_dp response payload
    assign w_dp_rsp_o.resp = write_rsp_i.b.resp;
    assign w_dp_rsp_o.user = write_rsp_i.b.user;

    // w_dp_valid_o is triggered once the write answer is here
    assign w_dp_valid_o = write_rsp_i.b_valid;

    // create back pressure on the b channel if the higher parts of the DMA cannot accept more
    // write responses
    assign write_req_o.b_ready = w_dp_ready_i;


    //--------------------------------------
    // Write user signals
    //--------------------------------------
    // in the default implementation: no need for the write user signals
    assign write_req_o.w.user = '0;

    //--------------------------------------
    // Unused AXI signals
    //--------------------------------------
    assign write_req_o.ar       = '0;
    assign write_req_o.ar_valid = 1'b0;
    assign write_req_o.r_ready  = 1'b0;

    //--------------------------------------
    // State
    //--------------------------------------
    `FF(first_w_q,     first_w_d,     '1, clk_i, rst_ni)
    `FF(bus_lane_q,    bus_lane_d,    '0, clk_i, rst_ni)
    `FF(fifo_lane_q,   fifo_lane_d,   '0, clk_i, rst_ni)
    `FF(bytes_left_q,  bytes_left_d,  '0, clk_i, rst_ni)

    `ASSERT(WriteSizeFits, w_dp_valid_i |-> int'(w_dp_req_i.size) <= $clog2(StrbWidth),
            clk_i, !rst_ni, "AXI write size must not exceed the physical bus width")
    `ASSERT(WritePayloadNonzero, write_happening |-> payload_bytes != '0,
            clk_i, !rst_ni, "Accepted AXI write beats must contain logical payload bytes")
    `ASSERT(WriteRemainingNonzero, w_dp_valid_i |-> bytes_left != '0,
            clk_i, !rst_ni, "Pending AXI writes must have logical payload bytes remaining")
    // Compute may suppress strobes, but its logical positions still count toward
    // burst completion and must be reported as consumed even for a zero-strobe beat.
    `ASSERT(WriteByteConservation, write_happening |->
                $countones(buffer_out_consumed_o) == int'(payload_bytes) &&
                tf_len_t'(payload_bytes) <= bytes_left,
            clk_i, !rst_ni, "An AXI write beat did not consume exactly its logical payload bytes")
    `ASSERT(WriteStrobesWithinPayload, write_req_o.w_valid |->
                (write_req_o.w.strb & ~logical_mask_out) == '0,
            clk_i, !rst_ni, "AXI WSTRB selects bytes outside the current logical payload")
    `ASSERT(WritePayloadWithinSize, w_dp_valid_i |->
                int'(beat_offset) + int'(payload_bytes) <= int'(beat_bytes),
            clk_i, !rst_ni, "AXI write payload extends beyond the selected-size access")
    `ASSERT(WritePopOnlyOnHandshake, !write_happening |->
                buffer_out_ready_o == '0 && buffer_out_consumed_o == '0,
            clk_i, !rst_ni, "AXI write bytes were consumed without a W handshake")
    `ASSERT(WriteCursorProgress, write_happening |=>
                fifo_lane_q == offset_t'($past(fifo_lane) + $past(payload_bytes)) &&
                bytes_left_q == $past(bytes_left) - tf_len_t'($past(payload_bytes)) &&
                bus_lane_q == ($past(w_dp_req_i.burst) == axi_pkg::BURST_INCR
                    ? offset_t'($past(bus_lane) + $past(bytes_to_size_boundary)) : $past(bus_lane)),
            clk_i, !rst_ni, "AXI write FIFO cursor or remaining byte count advanced incorrectly")
    `ASSERT(WriteStateStableWhenStalled, !write_happening |=>
                $stable({first_w_q, bus_lane_q, fifo_lane_q, bytes_left_q}),
            clk_i, !rst_ni, "AXI write position changed without accepting a beat")
    `ASSERT(WriteStableWhenStalled, write_req_o.w_valid && !write_rsp_i.w_ready |=>
                write_req_o.w_valid && $stable(write_req_o.w),
            clk_i, !rst_ni, "AXI write payload changed while waiting for WREADY")

endmodule
