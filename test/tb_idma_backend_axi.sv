// Copyright 2026 Mosaic SoC.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Luca Rufer <luca@mosaic-soc.com>

// AXI backend regression: full-width FIXED bursts and runtime-sized byte streams.
// Checks payload ordering, untouched destination bytes, and AXI address/size/strobe metadata.

`timescale 1ns/1ns
`include "axi/typedef.svh"
`include "idma/typedef.svh"

module tb_idma_backend_axi import idma_pkg::*; #(
    parameter int unsigned BufferDepth      = 3,
    parameter int unsigned NumAxInFlight    = 3,
    parameter int unsigned DataWidth        = 32,
    parameter int unsigned AddrWidth        = 32,
    parameter int unsigned UserWidth        = 1,
    parameter int unsigned AxiIdWidth       = 3,
    parameter int unsigned TFLenWidth       = 32,
    parameter int unsigned MemSysDepth      = 0,
    parameter bit          CombinedShifter  = 1'b0,
    parameter int unsigned WatchDogNumCycles = 2000,
    parameter int unsigned StallPct = 30
);

    localparam time TA  = 1ns;
    localparam time TT  = 9ns;
    localparam time TCK = 10ns;

    localparam int unsigned StrbWidth   = DataWidth / 8;
    localparam idma_pkg::error_cap_e ErrorCap = idma_pkg::NO_ERROR_HANDLING;

    typedef logic [7:0]             byte_t;
    typedef logic [AddrWidth-1:0]   addr_t;
    typedef logic [DataWidth-1:0]   data_t;
    typedef logic [StrbWidth-1:0]   strb_t;
    typedef logic [UserWidth-1:0]   user_t;
    typedef logic [AxiIdWidth-1:0]  id_t;
    typedef logic [TFLenWidth-1:0]  tf_len_t;

    `AXI_TYPEDEF_AW_CHAN_T(axi_aw_chan_t, addr_t, id_t, user_t)
    `AXI_TYPEDEF_W_CHAN_T(axi_w_chan_t, data_t, strb_t, user_t)
    `AXI_TYPEDEF_B_CHAN_T(axi_b_chan_t, id_t, user_t)
    `AXI_TYPEDEF_AR_CHAN_T(axi_ar_chan_t, addr_t, id_t, user_t)
    `AXI_TYPEDEF_R_CHAN_T(axi_r_chan_t, data_t, id_t, user_t)
    `AXI_TYPEDEF_REQ_T(axi_req_t, axi_aw_chan_t, axi_w_chan_t, axi_ar_chan_t)
    `AXI_TYPEDEF_RESP_T(axi_rsp_t, axi_b_chan_t, axi_r_chan_t)

    `IDMA_TYPEDEF_FULL_REQ_T(idma_req_t, id_t, addr_t, tf_len_t)
    `IDMA_TYPEDEF_FULL_RSP_T(idma_rsp_t, addr_t)

    typedef struct packed { axi_ar_chan_t ar_chan; } axi_read_meta_channel_t;
    typedef struct packed { axi_read_meta_channel_t axi; } read_meta_channel_t;
    typedef struct packed { axi_aw_chan_t aw_chan; } axi_write_meta_channel_t;
    typedef struct packed { axi_write_meta_channel_t axi; } write_meta_channel_t;

    // Clock / reset
    logic clk;
    logic rst_n;

    // DMA request / response
    idma_req_t idma_req;
    logic req_valid, req_ready;
    idma_rsp_t idma_rsp;
    logic rsp_valid, rsp_ready;
    idma_eh_req_t idma_eh_req;
    logic eh_req_valid, eh_req_ready;
    idma_busy_t busy;

    // Single read master bus, single write master bus
    axi_req_t axi_read_req, axi_write_req;
    axi_rsp_t axi_read_rsp, axi_write_rsp;
    axi_req_t read_mem_req, write_mem_req;
    axi_rsp_t read_mem_rsp, write_mem_rsp;

    // A channel that has offered VALID must stay open until its handshake. Random
    // pauses may delay a new beat, but must never retract an already offered one.
    bit allow_ar = 1, allow_r = 1, allow_aw = 1, allow_w = 1, allow_b = 1;
    bit check_enabled = 0, peripheral_reads = 0;
    int unsigned peripheral_beat = 0, peripheral_base = 0;
    int unsigned peripheral_size = 0, peripheral_lane = 0;

    function automatic byte_t peripheral_byte(input int unsigned beat, lane);
        return byte_t'(17 + 29 * beat + lane);
    endfunction

    always_comb begin
        read_mem_req = axi_read_req;
        read_mem_req.ar_valid &= allow_ar;
        read_mem_req.r_ready &= allow_r;
        axi_read_rsp = read_mem_rsp;
        axi_read_rsp.ar_ready &= allow_ar;
        axi_read_rsp.r_valid &= allow_r;
        // A FIXED source behaves like a peripheral, not an immutable memory word.
        // Every accepted access supplies new bytes; unused bus lanes contain junk.
        if (check_enabled && peripheral_reads) begin
            for (int lane = 0; lane < StrbWidth; lane++)
                if (lane >= peripheral_lane && lane < peripheral_lane + (1 << peripheral_size))
                    axi_read_rsp.r.data[8*lane +: 8] =
                        peripheral_byte(peripheral_beat - peripheral_base, lane - peripheral_lane);
                else axi_read_rsp.r.data[8*lane +: 8] = byte_t'(211 + lane);
        end
        write_mem_req = axi_write_req;
        write_mem_req.aw_valid &= allow_aw;
        write_mem_req.w_valid &= allow_w;
        write_mem_req.b_ready &= allow_b;
        axi_write_rsp = write_mem_rsp;
        axi_write_rsp.aw_ready &= allow_aw;
        axi_write_rsp.w_ready &= allow_w;
        axi_write_rsp.b_valid &= allow_b;
    end

    always @(posedge clk) begin
        if (rst_n) begin
            if (!(axi_read_req.ar_valid && allow_ar && !read_mem_rsp.ar_ready))
                allow_ar <= #TA ($urandom_range(0, 99) >= StallPct);
            if (!(read_mem_rsp.r_valid && allow_r && !axi_read_req.r_ready))
                allow_r <= #TA ($urandom_range(0, 99) >= StallPct);
            if (!(axi_write_req.aw_valid && allow_aw && !write_mem_rsp.aw_ready))
                allow_aw <= #TA ($urandom_range(0, 99) >= StallPct);
            if (!(axi_write_req.w_valid && allow_w && !write_mem_rsp.w_ready))
                allow_w <= #TA ($urandom_range(0, 99) >= StallPct);
            if (!(write_mem_rsp.b_valid && allow_b && !axi_write_req.b_ready))
                allow_b <= #TA ($urandom_range(0, 99) >= StallPct);
        end
    end

    assign rsp_ready = 1'b1;
    assign idma_eh_req = '0;
    assign eh_req_valid = 1'b0;

    // Sample completed handshakes before the edge; apply requests after the edge.
    clocking cb @(posedge clk);
        default input #1step output #TA;
        input req_ready, rsp_valid, idma_rsp, busy;
        output idma_req, req_valid;
    endclocking

    clk_rst_gen #(
        .ClkPeriod    (TCK),
        .RstClkCycles (1)
    ) i_clk_rst_gen (
        .clk_o  (clk),
        .rst_no (rst_n)
    );

    // Monitor taps used to check the FIXED side's beat count and address
    // constancy. Transaction (burst) counts are derived from the AR/AW
    // handshakes directly below instead, since they're immune to W-channel
    // stalls and don't depend on axi_sim_mem's `last` monitor output.
    logic          mon_r_valid;
    addr_t         mon_r_addr;
    logic          mon_w_valid;
    addr_t         mon_w_addr;

    // Read-side memory (Source)
    axi_sim_mem #(
        .AddrWidth         (AddrWidth),
        .DataWidth         (DataWidth),
        .IdWidth           (AxiIdWidth),
        .UserWidth         (UserWidth),
        .axi_req_t         (axi_req_t),
        .axi_rsp_t         (axi_rsp_t),
        .WarnUninitialized (1'b0),
        .ClearErrOnAccess  (1'b1),
        .ApplDelay         (TA),
        .AcqDelay          (TT)
    ) i_read_mem (
        .clk_i              (clk),
        .rst_ni             (rst_n),
        .axi_req_i          (read_mem_req),
        .axi_rsp_o          (read_mem_rsp),
        .mon_r_valid_o      (mon_r_valid),
        .mon_r_addr_o       (mon_r_addr),
        .mon_r_last_o       (),
        .mon_r_beat_count_o (),
        .mon_r_user_o       (),
        .mon_r_id_o         (),
        .mon_r_data_o       (),
        .mon_w_last_o       (),
        .mon_w_beat_count_o (),
        .mon_w_user_o       (),
        .mon_w_id_o         (),
        .mon_w_data_o       (),
        .mon_w_addr_o       (),
        .mon_w_valid_o      ()
    );

    // Write-side memory (Destination)
    axi_sim_mem #(
        .AddrWidth         (AddrWidth),
        .DataWidth         (DataWidth),
        .IdWidth           (AxiIdWidth),
        .UserWidth         (UserWidth),
        .axi_req_t         (axi_req_t),
        .axi_rsp_t         (axi_rsp_t),
        .WarnUninitialized (1'b0),
        .ClearErrOnAccess  (1'b1),
        .ApplDelay         (TA),
        .AcqDelay          (TT)
    ) i_write_mem (
        .clk_i              (clk),
        .rst_ni             (rst_n),
        .axi_req_i          (write_mem_req),
        .axi_rsp_o          (write_mem_rsp),
        .mon_r_last_o       (),
        .mon_r_beat_count_o (),
        .mon_r_user_o       (),
        .mon_r_id_o         (),
        .mon_r_data_o       (),
        .mon_r_addr_o       (),
        .mon_r_valid_o      (),
        .mon_w_valid_o      (mon_w_valid),
        .mon_w_addr_o       (mon_w_addr),
        .mon_w_last_o       (),
        .mon_w_beat_count_o (),
        .mon_w_user_o       (),
        .mon_w_id_o         (),
        .mon_w_data_o       ()
    );

    idma_backend_rw_axi #(
        .CombinedShifter      (CombinedShifter),
        .DataWidth            (DataWidth),
        .AddrWidth            (AddrWidth),
        .AxiIdWidth           (AxiIdWidth),
        .UserWidth            (UserWidth),
        .TFLenWidth           (TFLenWidth),
        .BufferDepth          (BufferDepth),
        .RAWCouplingAvail     (1'b0),
        .MaskInvalidData      (1'b1),
        .HardwareLegalizer    (1'b1),
        .RejectZeroTransfers  (1'b1),
        .ErrorCap             (ErrorCap),
        .PrintFifoInfo        (1'b0),
        .NumAxInFlight        (NumAxInFlight),
        .MemSysDepth          (MemSysDepth),
        .idma_req_t           (idma_req_t),
        .idma_rsp_t           (idma_rsp_t),
        .idma_eh_req_t        (idma_eh_req_t),
        .idma_busy_t          (idma_busy_t),
        .axi_req_t            (axi_req_t),
        .axi_rsp_t            (axi_rsp_t),
        .write_meta_channel_t (write_meta_channel_t),
        .read_meta_channel_t  (read_meta_channel_t)
    ) i_idma_backend (
        .clk_i           (clk),
        .rst_ni          (rst_n),
        .idma_req_i      (idma_req),
        .req_valid_i     (req_valid),
        .req_ready_o     (req_ready),
        .idma_rsp_o      (idma_rsp),
        .rsp_valid_o     (rsp_valid),
        .rsp_ready_i     (rsp_ready),
        .idma_eh_req_i   (idma_eh_req),
        .eh_req_valid_i  (eh_req_valid),
        .eh_req_ready_o  (eh_req_ready),
        .axi_read_req_o  (axi_read_req),
        .axi_read_rsp_i  (axi_read_rsp),
        .axi_write_req_o (axi_write_req),
        .axi_write_rsp_i (axi_write_rsp),
        .busy_o          (busy)
    );

    // ----------------------------------------------------------------------
    // Stimulus
    // ----------------------------------------------------------------------
    int unsigned errors = 0;

    // Test number used to generate a unique seed for each test
    int unsigned test_no = 1;
    int unsigned launched_count = 0, completed_count = 0, size_case_count = 0;

    /// Submit a fully specified request, holding it until the sampled handshake.
    task automatic launch_request(input idma_req_t request);
        int unsigned cycles = 0;
        @(cb);
        cb.idma_req <= request;
        cb.req_valid <= 1'b1;
        do begin
            @(cb);
            cycles++;
            assert (cycles < WatchDogNumCycles)
                else $fatal(1, "DMA request was not accepted");
        end while (!cb.req_ready);
        launched_count++;
        cb.req_valid <= 1'b0;
    endtask

    /// Construct the legacy full-width requests used by the original directed tests.
    task automatic launch_tf(input tf_len_t length, input addr_t src_addr, dst_addr,
                             input protocol_e src_protocol, dst_protocol,
                             input bit decouple_aw,
                             input logic [2:0] src_max_llen, dst_max_llen,
                             input bit src_reduce_len, dst_reduce_len, input id_t id,
                             input axi_pkg::burst_t src_burst, dst_burst);
        idma_req_t request = '0;
        request.length = length;
        request.src_addr = src_addr;
        request.dst_addr = dst_addr;
        request.opt.src_protocol = src_protocol;
        request.opt.dst_protocol = dst_protocol;
        request.opt.src.burst = src_burst;
        request.opt.dst.burst = dst_burst;
        request.opt.axi_id = id;
        request.opt.beo.decouple_aw = decouple_aw;
        request.opt.beo.src_max_llen = src_max_llen;
        request.opt.beo.dst_max_llen = dst_max_llen;
        request.opt.beo.src_reduce_len = src_reduce_len;
        request.opt.beo.dst_reduce_len = dst_reduce_len;
        launch_request(request);
    endtask

    // Independently track logical progress at AR/AW, not the DUT's FIFO masks.
    // One check entry describes a transfer; one write entry describes an accepted AW.
    typedef struct packed {
        addr_t addr;
        int unsigned remaining, size, max_beats;
        bit fixed_addr;
    } transfer_check_t;
    typedef struct packed {
        addr_t addr;
        int unsigned remaining, size, beats;
        bit fixed_addr;
    } write_check_t;
    transfer_check_t read_checks[$], write_checks[$];
    write_check_t write_bursts[$];
    byte_t expected_stream[$];
    byte_t expected_memory[addr_t];

    /// Check an address request and advance its transfer's logical byte position.
    task automatic check_address(input bit is_write, input addr_t addr,
                                 input axi_pkg::size_t size, input axi_pkg::len_t len,
                                 input axi_pkg::burst_t burst);
        transfer_check_t spec;
        write_check_t write_spec;
        int unsigned bytes_per_beat, prefix, payload, capacity;
        assert (is_write ? write_checks.size() != 0 : read_checks.size() != 0)
            else $fatal(1, "Unexpected AXI address request");
        spec = is_write ? write_checks[0] : read_checks[0];
        bytes_per_beat = 1 << spec.size;
        prefix = int'(spec.addr) % bytes_per_beat;
        capacity = (int'(len) + 1) * bytes_per_beat - prefix;
        payload = capacity < spec.remaining ? capacity : spec.remaining;
        assert (int'(size) == spec.size &&
                addr == (spec.addr & ~addr_t'(bytes_per_beat - 1)) &&
                burst == (spec.fixed_addr ? axi_pkg::BURST_FIXED : axi_pkg::BURST_INCR))
            else $fatal(1, "AXI address/size/burst mismatch (write=%0b)", is_write);
        assert (int'(len) + 1 <= spec.max_beats && payload != 0 &&
                int'(len) == (prefix + payload - 1) / bytes_per_beat)
            else $fatal(1, "AXI burst length exceeds its limit or logical payload");
        if (!spec.fixed_addr)
            assert (int'(addr[11:0]) + (int'(len) + 1) * bytes_per_beat <= 4096)
                else $fatal(1, "AXI burst crosses a 4 KiB boundary");
        if (is_write) begin
            write_spec = '{spec.addr, payload, spec.size, int'(len) + 1, spec.fixed_addr};
            write_bursts.push_back(write_spec);
        end
        spec.remaining -= payload;
        if (!spec.fixed_addr) spec.addr += addr_t'(payload);
        if (is_write) begin
            if (spec.remaining == 0) void'(write_checks.pop_front());
            else write_checks[0] = spec;
        end else begin
            if (spec.remaining == 0) void'(read_checks.pop_front());
            else read_checks[0] = spec;
        end
    endtask

    /// Run one isolated case; queued tests deliberately use prepare/launch separately.
    task automatic run_size_request(input idma_req_t request);
        peripheral_reads = request.opt.src.burst == axi_pkg::BURST_FIXED;
        peripheral_base = peripheral_beat;
        peripheral_size = request.opt.beo.src_size_enable
            ? int'(request.opt.beo.src_size) : $clog2(StrbWidth);
        peripheral_lane = int'(request.src_addr) % StrbWidth;
        clear_mem();
        prepare_size_case(request);
        launch_request(request);
        wait_done();
        check_size_results();
    endtask

    // Capture every W payload byte, including FIXED writes that memory later overwrites.
    // AW may run ahead of W, so retain its metadata until WLAST rather than using live AW.
    always @(posedge clk) begin
        if (rst_n) begin
            if (rsp_valid && rsp_ready) begin
                assert (!idma_rsp.error) else $fatal(1, "DMA returned an error response");
                completed_count++;
            end
            if (check_enabled) begin
                if (axi_read_req.ar_valid && axi_read_rsp.ar_ready)
                    check_address(0, axi_read_req.ar.addr, axi_read_req.ar.size,
                                  axi_read_req.ar.len, axi_read_req.ar.burst);
                if (axi_write_req.aw_valid && axi_write_rsp.aw_ready)
                    check_address(1, axi_write_req.aw.addr, axi_write_req.aw.size,
                                  axi_write_req.aw.len, axi_write_req.aw.burst);
                if (axi_read_rsp.r_valid && axi_read_req.r_ready)
                    peripheral_beat <= #TA peripheral_beat + 1;
                if (axi_write_req.w_valid && axi_write_rsp.w_ready) begin
                    write_check_t spec;
                    strb_t expected_strb;
                    int unsigned lane, count, beat_bytes;
                    assert (write_bursts.size() != 0)
                        else $fatal(1, "W handshake without an accepted AW");
                    spec = write_bursts[0];
                    expected_strb = '0;
                    lane = int'(spec.addr) % StrbWidth;
                    beat_bytes = (1 << spec.size) - int'(spec.addr) % (1 << spec.size);
                    count = spec.remaining < beat_bytes ? spec.remaining : beat_bytes;
                    for (int unsigned byte_idx = 0; byte_idx < count; byte_idx++) begin
                        byte_t expected;
                        assert (expected_stream.size() != 0)
                            else $fatal(1, "DMA wrote more payload bytes than requested");
                        expected = expected_stream.pop_front();
                        expected_strb[lane + byte_idx] = 1'b1;
                        assert (axi_write_req.w.data[8*(lane + byte_idx) +: 8] === expected)
                            else $fatal(1, "Case %0d W byte mismatch: got %h, expected %h",
                                size_case_count, axi_write_req.w.data[8*(lane + byte_idx) +: 8], expected);
                    end
                    assert (axi_write_req.w.strb == expected_strb &&
                            axi_write_req.w.last == (spec.beats == 1))
                        else $fatal(1, "Case %0d: WSTRB %h expected %h, WLAST %0b expected %0b",
                            size_case_count, axi_write_req.w.strb, expected_strb,
                            axi_write_req.w.last, spec.beats == 1);
                    spec.remaining -= count;
                    spec.beats--;
                    if (!spec.fixed_addr) spec.addr += addr_t'(count);
                    if (spec.beats == 0) begin
                        assert (spec.remaining == 0) else $fatal(1, "WLAST before payload end");
                        void'(write_bursts.pop_front());
                    end else write_bursts[0] = spec;
                end
            end
        end
    end

    /// Prepare a byte-stream oracle and canaries before submitting the request.
    /// FIXED reads advance their data on every access; FIXED writes overwrite the
    /// same size-selected lanes. Neither behavior can be checked by a memcpy model.
    task automatic prepare_size_case(input idma_req_t request);
        transfer_check_t spec;
        int unsigned rsize, wsize, rbytes, wbytes;
        byte_t seed = byte_t'(test_no++);
        rsize = request.opt.beo.src_size_enable ? int'(request.opt.beo.src_size) : $clog2(StrbWidth);
        wsize = request.opt.beo.dst_size_enable ? int'(request.opt.beo.dst_size) : $clog2(StrbWidth);
        rbytes = 1 << rsize;
        wbytes = 1 << wsize;
        spec = '{request.src_addr, int'(request.length), rsize,
                 request.opt.src.burst == axi_pkg::BURST_FIXED ? 16 : 256,
                 request.opt.src.burst == axi_pkg::BURST_FIXED};
        if (request.opt.beo.src_reduce_len && (1 << request.opt.beo.src_max_llen) < spec.max_beats)
            spec.max_beats = 1 << request.opt.beo.src_max_llen;
        read_checks.push_back(spec);
        spec = '{request.dst_addr, int'(request.length), wsize,
                 request.opt.dst.burst == axi_pkg::BURST_FIXED ? 16 : 256,
                 request.opt.dst.burst == axi_pkg::BURST_FIXED};
        if (request.opt.beo.dst_reduce_len && (1 << request.opt.beo.dst_max_llen) < spec.max_beats)
            spec.max_beats = 1 << request.opt.beo.dst_max_llen;
        write_checks.push_back(spec);
        for (int unsigned i = 0; i < request.length + 2 * StrbWidth; i++) begin
            addr_t src = request.src_addr - addr_t'(StrbWidth) + addr_t'(i);
            addr_t dst = request.dst_addr - addr_t'(StrbWidth) + addr_t'(i);
            i_read_mem.mem[src] = gen_data(src, seed);
            i_write_mem.mem[dst] = 8'ha5;
            expected_memory[dst] = 8'ha5;
        end
        for (int unsigned i = 0; i < request.length; i++) begin
            byte_t value;
            addr_t dst = request.dst_addr + addr_t'(spec.fixed_addr ? i % wbytes : i);
            if (request.opt.src.burst == axi_pkg::BURST_FIXED)
                value = peripheral_byte(i / rbytes, i % rbytes);
            else value = gen_data(request.src_addr + addr_t'(i), seed);
            expected_stream.push_back(value);
            expected_memory[dst] = value;
        end
        size_case_count++;
    endtask

    /// Verify all requested bytes and the surrounding, deliberately untouched memory.
    task automatic check_size_results();
        assert (read_checks.size() == 0 && write_checks.size() == 0 &&
                write_bursts.size() == 0 && expected_stream.size() == 0)
            else $fatal(1, "DMA completed with unprocessed requests or payload bytes");
        foreach (expected_memory[addr])
            assert (i_write_mem.mem[addr] === expected_memory[addr])
                else $fatal(1, "Destination/canary mismatch at %h", addr);
        expected_memory.delete();
    endtask

    /// Cross endpoint sizes and burst modes, legal bus offsets, partial payloads,
    /// safe/unsafe admission, and small user burst limits near a page boundary.
    task automatic run_size_tests();
        check_enabled = 1;
        for (int rsize = 0; rsize <= $clog2(StrbWidth); rsize++)
            for (int wsize = 0; wsize <= $clog2(StrbWidth); wsize++)
                for (int modes = 0; modes < 4; modes++)
                    for (int roff = 0; roff < StrbWidth; roff++)
                        for (int woff = 0; woff < StrbWidth; woff++) begin
                            int lengths[11] = '{1, (1 << rsize) - 1, (1 << rsize),
                                               (1 << rsize) + 1, (1 << wsize) - 1,
                                               (1 << wsize), (1 << wsize) + 1,
                                               StrbWidth - 1, StrbWidth, StrbWidth + 1,
                                               2 * StrbWidth + 3};
                            // Exhaust offset pairs through 64-bit buses. On wider buses
                            // retain every lane with equal offsets and either endpoint at zero.
                            if (StrbWidth > 8 && roff != 0 && woff != 0 && roff != woff) continue;
                            if ((modes[0] && roff % (1 << rsize) != 0) ||
                                (modes[1] && woff % (1 << wsize) != 0)) continue;
                            foreach (lengths[idx]) begin
                                bit duplicate = 0;
                                idma_req_t request = '0;
                                for (int prev = 0; prev < idx; prev++)
                                    duplicate |= lengths[prev] == lengths[idx];
                                if (lengths[idx] == 0 || duplicate) continue;
                                // Combined shifting retains full-width operation only.
                                if (CombinedShifter && (rsize != $clog2(StrbWidth) ||
                                                       wsize != $clog2(StrbWidth))) continue;
                                request.length = tf_len_t'(lengths[idx]);
                                request.src_addr = addr_t'('h3000 - StrbWidth + roff);
                                request.dst_addr = addr_t'('h6000 - StrbWidth + woff);
                                request.opt.src_protocol = AXI;
                                request.opt.dst_protocol = AXI;
                                request.opt.src.burst = modes[0] ? axi_pkg::BURST_FIXED : axi_pkg::BURST_INCR;
                                request.opt.dst.burst = modes[1] ? axi_pkg::BURST_FIXED : axi_pkg::BURST_INCR;
                                request.opt.beo.src_size_enable = !CombinedShifter;
                                request.opt.beo.dst_size_enable = !CombinedShifter;
                                request.opt.beo.src_size = axi_pkg::size_t'(rsize);
                                request.opt.beo.dst_size = axi_pkg::size_t'(wsize);
                                request.opt.beo.src_reduce_len = 1;
                                request.opt.beo.dst_reduce_len = 1;
                                request.opt.beo.src_max_llen = 1;
                                request.opt.beo.dst_max_llen = 2;
                                for (int safe = 0; safe < 2; safe++) begin
                                    request.opt.beo.deadlock_free = 1'(safe);
                                    run_size_request(request);
                                end
                            end
                        end
        peripheral_reads = 0;
    endtask

    /// Force the AXI protocol limits (256 INCR beats and 16 FIXED beats), without
    /// user LLEN restrictions or safe-mode clamping hiding those boundaries.
    task automatic run_burst_limit_tests();
        idma_req_t request = '0;
        int unsigned size = CombinedShifter ? $clog2(StrbWidth) : 0;
        request.src_addr = addr_t'('h11000);
        request.dst_addr = addr_t'('h21000);
        request.opt.src_protocol = AXI;
        request.opt.dst_protocol = AXI;
        request.opt.src.burst = axi_pkg::BURST_INCR;
        request.opt.dst.burst = axi_pkg::BURST_INCR;
        request.opt.beo.src_size_enable = !CombinedShifter;
        request.opt.beo.dst_size_enable = !CombinedShifter;
        request.opt.beo.src_size = 3'(size);
        request.opt.beo.dst_size = 3'(size);
        request.length = tf_len_t'((257 << size) + 1);
        reset_read_monitor(request.src_addr);
        reset_write_monitor(request.dst_addr);
        run_size_request(request);
        assert (r_txns >= 2 && w_txns >= 2)
            else $fatal(1, "INCR limit test did not split both endpoints into multiple bursts");
        request.length = tf_len_t'((17 << size) + 1);
        request.opt.src.burst = axi_pkg::BURST_FIXED;
        run_size_request(request);
        request.opt.src.burst = axi_pkg::BURST_INCR;
        request.opt.dst.burst = axi_pkg::BURST_FIXED;
        run_size_request(request);
        peripheral_reads = 0;
    endtask

    /// Queue transfers without waiting for completion, changing size, alignment,
    /// burst limits and safe mode. Disabled size fields must be ignored as well.
    task automatic run_queued_tests();
        bit overlap_seen = 0;
        clear_mem();
        for (int i = 0; i < 8; i++) begin
            idma_req_t request = '0;
            request.length = tf_len_t'(17 * StrbWidth + i);
            request.src_addr = addr_t'('h10004 + i * 'h1000);
            request.dst_addr = addr_t'('h30003 + i * 'h1000);
            request.opt.src_protocol = AXI;
            request.opt.dst_protocol = AXI;
            request.opt.src.burst = axi_pkg::BURST_INCR;
            request.opt.dst.burst = axi_pkg::BURST_INCR;
            request.opt.beo.src_size_enable = !CombinedShifter && i % 3 != 0;
            request.opt.beo.dst_size_enable = !CombinedShifter && i % 4 != 0;
            request.opt.beo.src_size = 3'(i % ($clog2(StrbWidth) + 1));
            request.opt.beo.dst_size = 3'((i + 1) % ($clog2(StrbWidth) + 1));
            // Even an out-of-bus-width encoding must be ignored when its enable is zero.
            if (!request.opt.beo.src_size_enable) request.opt.beo.src_size = 3'd7;
            if (!request.opt.beo.dst_size_enable) request.opt.beo.dst_size = 3'd7;
            if (i[1]) begin
                int unsigned size = request.opt.beo.dst_size_enable
                    ? int'(request.opt.beo.dst_size) : $clog2(StrbWidth);
                request.opt.dst.burst = axi_pkg::BURST_FIXED;
                request.dst_addr &= ~addr_t'((1 << size) - 1);
            end
            request.opt.beo.deadlock_free = i[0];
            request.opt.beo.src_reduce_len = 1;
            request.opt.beo.src_max_llen = 2;
            prepare_size_case(request);
            if (i != 0 && completed_count < launched_count) overlap_seen = 1;
            launch_request(request);
        end
        assert (overlap_seen) else $fatal(1, "Queued test did not exercise overlapping transfers");
        wait_done();
        check_size_results();
        $display("[tb_idma_backend_axi] %0d size/queued cases passed", size_case_count);
    endtask

    // Clear both simulation memories
    task automatic clear_mem ();
        i_read_mem.mem.delete();
        i_write_mem.mem.delete();
    endtask

    function automatic byte_t gen_data(input addr_t addr, input byte seed);
        return seed ^ addr[7:0];
    endfunction

    // Preload read memory with a deterministic pattern
    task automatic preload (input addr_t base, input int unsigned len, input byte_t seed);
        for (int unsigned i = 0; i < len; i++)
            i_read_mem.mem[base + i] = gen_data(i, seed);
    endtask

    // Check that write memory at `base` expected pattern for an continuous write
    task automatic check_fixed_to_fixed (input string label, input addr_t base,
                                         input int unsigned len, input byte_t seed);
        byte_t got, exp;
        for (int unsigned i = 0; i < len; i++) begin
            exp = gen_data(i, seed);
            if (!i_write_mem.mem.exists(base + i)) begin
                errors++;
                $error("[%s] Write @0x%h: Address never written", label, base + i);
                continue;
            end
            got = i_write_mem.mem[base + i];
            if (got !== exp) begin
                errors++;
                $error("[%s] Write @0x%h: got 0x%h exp 0x%h", label, base + i, got, exp);
            end
        end
        $display("[%s] Write @0x%h len %0d: OK", label, base, len);
    endtask

    // Check that write memory at `base` holds the expected pattern for a FIXED write, a repeating
    // pattern of `StrbWidth` bytes.
    task automatic check_fixed_to_incr (input string label, input addr_t base,
                                        input int unsigned length, input byte_t seed);
        byte_t got, exp;
        for (int unsigned i = 0; i < length; i++) begin
            exp = gen_data(i % StrbWidth, seed);
            if (!i_write_mem.mem.exists(base + i)) begin
                errors++;
                $error("[%s] Write @0x%h: Address never written", label, base + i);
                continue;
            end
            got = i_write_mem.mem[base + i];
            if (got !== exp) begin
                errors++;
                $error("[%s] Write @0x%h: got 0x%h exp 0x%h", label, base + i, got, exp);
            end
        end
        $display("[%s] Write @0x%h len %0d: OK", label, base, length);
    endtask

    // Check that write memory at `base` holds the expected pattern for a FIXED
    // write fed by an INCR source of `length` bytes.
    // The expected pattern is the last beat of the INCR source, and possibly
    // the second-to-last beat if the last beat is partial.
    task automatic check_incr_to_fixed (input string label, input addr_t base,
                                        input int unsigned length, input byte_t seed);
        int unsigned exp_beats, tailer, checked_len;
        byte_t got, exp;
        exp_beats   = (length + StrbWidth - 1) / StrbWidth;
        tailer      = length % StrbWidth;
        checked_len = (tailer == 0) ? StrbWidth : ((exp_beats == 1) ? tailer : StrbWidth);
        for (int unsigned i = 0; i < checked_len; i++) begin
            int unsigned from_beat = (tailer == 0 || i < tailer) ? exp_beats - 1 : exp_beats - 2;
            exp = gen_data(from_beat * StrbWidth + i, seed);
            if (!i_write_mem.mem.exists(base + i)) begin
                errors++;
                $error("[%s] Write @0x%h: Address never written", label, base + i);
                continue;
            end
            got = i_write_mem.mem[base + i];
            if (got !== exp) begin
                errors++;
                $error("[%s] Write @0x%h: got 0x%h exp 0x%h", label, base + i, got, exp);
            end
        end
        $display("[%s] Write @0x%h len %0d: OK", label, base, checked_len);
    endtask

    // Background monitors to check beat count, transaction count, and
    // addresses for FIXED side bursts.
    int unsigned r_beats, w_beats;
    int unsigned r_txns, w_txns;
    logic        r_addr_ok, w_addr_ok;
    addr_t       r_exp_addr, w_exp_addr;

    function automatic void reset_read_monitor (input addr_t exp_addr);
        r_beats    = 0;
        r_txns     = 0;
        r_addr_ok  = 1'b1;
        r_exp_addr = exp_addr;
    endfunction

    function automatic void reset_write_monitor (input addr_t exp_addr);
        w_beats    = 0;
        w_txns     = 0;
        w_addr_ok  = 1'b1;
        w_exp_addr = exp_addr;
    endfunction

    initial begin
        r_beats   = 0;
        r_txns    = 0;
        r_addr_ok = 1'b1;

        @(posedge rst_n);

        forever begin
            @(posedge clk);
            if (axi_read_req.ar_valid && axi_read_rsp.ar_ready) r_txns++;
            #(TT);
            if (mon_r_valid) begin
                r_beats++;
                if (mon_r_addr != r_exp_addr) r_addr_ok = 1'b0;
            end
        end
    end

    initial begin
        w_beats   = 0;
        w_txns    = 0;
        w_addr_ok = 1'b1;

        @(posedge rst_n);

        forever begin
            @(posedge clk);
            if (axi_write_req.aw_valid && axi_write_rsp.aw_ready) w_txns++;
            #(TT);
            if (mon_w_valid) begin
                w_beats++;
                if (mon_w_addr != w_exp_addr) w_addr_ok = 1'b0;
            end
        end
    end

    // Wait for the transfer to finish. When `mon_read` (resp. `mon_write`) is
    // set, checks that the read (resp. write) monitor saw exactly `exp_beats`
    // FIXED-side beats, split into exactly `exp_txns` AR/AW transactions, with
    // the address held constant throughout.
    task automatic wait_done (input string label = "", input int unsigned exp_beats = 0,
                              input bit mon_read = 1'b0, input bit mon_write = 1'b0,
                              input int unsigned exp_txns = 1);
        int unsigned cycles = 0;
        // Count responses, rather than sampling busy before a new request has taken effect.
        do begin
            @(cb);
            cycles++;
            assert (cycles < WatchDogNumCycles)
                else $fatal(1, "[%s] DMA completion timeout", label);
        end while (completed_count < launched_count);
        repeat (5) @(cb);

        if (mon_read) begin
            if (r_beats !== exp_beats) begin
                errors++;
                $error("[%s] AR: got %0d beats, exp %0d", label, r_beats, exp_beats);
            end
            if (r_txns !== exp_txns) begin
                errors++;
                $error("[%s] AR: got %0d transaction(s), exp %0d", label, r_txns, exp_txns);
            end
            if (!r_addr_ok) begin
                errors++;
                $error("[%s] AR: address was not constant across the FIXED burst", label);
            end
        end
        if (mon_write) begin
            if (w_beats !== exp_beats) begin
                errors++;
                $error("[%s] AW: got %0d beats, exp %0d", label, w_beats, exp_beats);
            end
            if (w_txns !== exp_txns) begin
                errors++;
                $error("[%s] AW: got %0d transaction(s), exp %0d", label, w_txns, exp_txns);
            end
            if (!w_addr_ok) begin
                errors++;
                $error("[%s] AW: address was not constant across the FIXED burst", label);
            end
        end
    endtask

    // FIXED src (one word, repeatedly read) -> INCR dst (length bytes, sequential).
    task automatic fixed_to_incr_test (input string label, input int unsigned length,
                                       input addr_t src = 32'h0000_1000,
                                       input addr_t dst = 32'h0000_2000,
                                       input int unsigned exp_txns = 1,
                                       input logic [2:0] src_max_llen = 3'd0,
                                       input bit src_reduce_len = 1'b0);
        int unsigned exp_beats;
        byte_t seed;
        seed = byte_t'(test_no++);
        exp_beats = (length + StrbWidth - 1) / StrbWidth;
        clear_mem();
        preload(src, StrbWidth, seed);
        reset_read_monitor(src);
        launch_tf(.length (length), .src_addr (src), .dst_addr (dst),
                      .src_protocol (idma_pkg::AXI), .dst_protocol (idma_pkg::AXI),
                      .decouple_aw (1'b0),
                      .src_max_llen (src_max_llen), .dst_max_llen (3'd0),
                      .src_reduce_len (src_reduce_len), .dst_reduce_len (1'b0), .id ('0),
                      .src_burst (axi_pkg::BURST_FIXED), .dst_burst (axi_pkg::BURST_INCR));
        wait_done(.label (label), .exp_beats (exp_beats), .mon_read (1'b1), .exp_txns (exp_txns));
        check_fixed_to_incr(label, dst, length, seed);
    endtask

    // INCR src (length bytes, sequential) -> FIXED dst (one word: last beat remains).
    task automatic incr_to_fixed_test (input string label, input int unsigned length,
                                       input addr_t src = 32'h0000_1000,
                                       input addr_t dst = 32'h0000_2000,
                                       input int unsigned exp_txns = 1,
                                       input logic [2:0] dst_max_llen = 3'd0,
                                       input bit dst_reduce_len = 1'b0);
        int unsigned exp_beats;
        byte_t seed;
        seed = byte_t'(test_no++);
        exp_beats = (length + StrbWidth - 1) / StrbWidth;
        clear_mem();
        preload(src, length, seed);
        reset_write_monitor(dst);
        launch_tf(.length (length), .src_addr (src), .dst_addr (dst),
                      .src_protocol (idma_pkg::AXI), .dst_protocol (idma_pkg::AXI),
                      .decouple_aw (1'b0),
                      .src_max_llen (3'd0), .dst_max_llen (dst_max_llen),
                      .src_reduce_len (1'b0), .dst_reduce_len (dst_reduce_len), .id ('0),
                      .src_burst (axi_pkg::BURST_INCR), .dst_burst (axi_pkg::BURST_FIXED));
        wait_done(.label (label), .exp_beats (exp_beats), .mon_write (1'b1), .exp_txns (exp_txns));
        check_incr_to_fixed(label, dst, length, seed);
    endtask

    // FIXED src -> FIXED dst: same one word on both ends, repeated length/StrbWidth times.
    task automatic fixed_to_fixed_test (input string label, input int unsigned length,
                                        input addr_t src = 32'h0000_1000,
                                        input addr_t dst = 32'h0000_2000,
                                        input int unsigned exp_txns = 1,
                                        input logic [2:0] src_max_llen = 3'd0,
                                        input bit src_reduce_len = 1'b0,
                                        input logic [2:0] dst_max_llen = 3'd0,
                                        input bit dst_reduce_len = 1'b0);
        int unsigned exp_beats;
        byte_t seed;
        seed = byte_t'(test_no++);
        exp_beats = (length + StrbWidth - 1) / StrbWidth;
        clear_mem();
        preload(src, StrbWidth, seed);
        reset_read_monitor(src);
        reset_write_monitor(dst);
        launch_tf(.length (length), .src_addr (src), .dst_addr (dst),
                      .src_protocol (idma_pkg::AXI), .dst_protocol (idma_pkg::AXI),
                      .decouple_aw (1'b0),
                      .src_max_llen (src_max_llen), .dst_max_llen (dst_max_llen),
                      .src_reduce_len (src_reduce_len), .dst_reduce_len (dst_reduce_len), .id ('0),
                      .src_burst (axi_pkg::BURST_FIXED), .dst_burst (axi_pkg::BURST_FIXED));
        wait_done(.label (label), .exp_beats (exp_beats), .mon_read (1'b1), .mon_write (1'b1),
                  .exp_txns (exp_txns));
        check_fixed_to_fixed(label, dst, StrbWidth, seed);
    endtask

    // AXI4 caps a FIXED burst at 16 beats; use more to force the legalizer
    // to split into multiple same-address sub-bursts instead of a single burst.
    localparam int unsigned NumBeatsMulti = 40;

    // llen reduction used in the llen reduction tests.
    localparam logic [2:0] MaxLlenReduce = 3'd2; // 4 beats

    initial begin
        cb.req_valid <= 1'b0;
        cb.idma_req <= '0;
        @(posedge rst_n);
        repeat (4) @(posedge clk);

        // Single sub-burst smoke tests
        fixed_to_incr_test("fixed_to_incr_small", 4 * StrbWidth);
        incr_to_fixed_test("incr_to_fixed_small", 4 * StrbWidth);
        fixed_to_fixed_test("fixed_to_fixed", 4 * StrbWidth);

        // Multi sub-burst: exercises the burst length limiting for the FIXED side
        fixed_to_incr_test("fixed_to_incr_multi", NumBeatsMulti * StrbWidth,
                           .exp_txns ((NumBeatsMulti + 15) / 16));
        incr_to_fixed_test("incr_to_fixed_multi", NumBeatsMulti * StrbWidth,
                           .exp_txns ((NumBeatsMulti + 15) / 16));

        // The INCR side crosses a page boundary. Independent legalization leaves
        // the opposite FIXED side as one burst at its original address.
        fixed_to_incr_test("fixed_to_incr_pagesplit", 4 * StrbWidth,
                           .src (32'h0000_1000), .dst (32'h0000_2000 - 2 * StrbWidth),
                           .exp_txns (1));
        incr_to_fixed_test("incr_to_fixed_pagesplit", 4 * StrbWidth,
                           .src (32'h0000_1000 - 2 * StrbWidth), .dst (32'h0000_2000),
                           .exp_txns (1));

        // llen reduction on the FIXED side
        fixed_to_incr_test("fixed_to_incr_llen_reduce", 8 * StrbWidth,
                           .exp_txns (8 / (1 << MaxLlenReduce)),
                           .src_max_llen (MaxLlenReduce), .src_reduce_len (1'b1));
        incr_to_fixed_test("incr_to_fixed_llen_reduce", 8 * StrbWidth,
                           .exp_txns (8 / (1 << MaxLlenReduce)),
                           .dst_max_llen (MaxLlenReduce), .dst_reduce_len (1'b1));

        // Narrow tests where length < StrbWidth
        for (int unsigned i = 1; i < StrbWidth; i++) begin
            fixed_to_incr_test($sformatf("fixed_to_incr_narrow_%0d", i), i);
            incr_to_fixed_test($sformatf("incr_to_fixed_narrow_%0d", i), i);
        end

        // Partial tests where the length is not a multiple of StrbWidth
        fixed_to_incr_test("fixed_to_incr_partial", 2 * StrbWidth - 1);
        incr_to_fixed_test("incr_to_fixed_partial", 2 * StrbWidth - 1);

        run_size_tests();
        run_burst_limit_tests();
        run_queued_tests();

        if (errors == 0)
            $display("[tb_idma_backend_axi] ALL CHECKS PASSED");
        else
            $fatal(1, "[tb_idma_backend_axi] %0d errors", errors);
        $finish;
    end



endmodule
