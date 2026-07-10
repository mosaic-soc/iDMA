// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Thomas Benz <tbenz@iis.ee.ethz.ch>
// - Tobias Senti <tsenti@ethz.ch>

`include "idma/guard.svh"
`include "common_cells/registers.svh"

/// Implementing the transport layer in the iDMA backend.
module idma_transport_layer_rw_axi #(
    /// Number of transaction that can be in-flight concurrently
    parameter int unsigned NumAxInFlight = 32'd2,
    /// Data width
    parameter int unsigned DataWidth = 32'd16,
    /// The depth of the internal reorder buffer:
    /// - '2': minimal possible configuration
    /// - '3': efficiently handle misaligned transfers (recommended)
    parameter int unsigned BufferDepth = 32'd3,
    /// Mask invalid data on the manager interface
    parameter bit MaskInvalidData = 1'b1,
    /// Print the info of the FIFO configuration
    parameter bit PrintFifoInfo = 1'b0,
    /// `r_dp_req_t` type:
    parameter type r_dp_req_t = logic,
    /// `w_dp_req_t` type:
    parameter type w_dp_req_t = logic,
    /// `r_dp_rsp_t` type:
    parameter type r_dp_rsp_t = logic,
    /// `w_dp_rsp_t` type:
    parameter type w_dp_rsp_t = logic,
    /// Write Meta channel type
    parameter type write_meta_channel_t = logic,
    /// Read Meta channel type
    parameter type read_meta_channel_t = logic,
    /// AXI4+ATOP Request and Response channel type
    parameter type axi_req_t = logic,
    parameter type axi_rsp_t = logic
)(
    /// Clock
    input  logic clk_i,
    /// Asynchronous reset, active low
    input  logic rst_ni,
    /// Testmode in
    input  logic testmode_i,

    /// AXI4+ATOP read request
    output axi_req_t axi_read_req_o,
    /// AXI4+ATOP read response
    input  axi_rsp_t axi_read_rsp_i,

    /// AXI4+ATOP write request
    output axi_req_t axi_write_req_o,
    /// AXI4+ATOP write response
    input  axi_rsp_t axi_write_rsp_i,

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

    /// Write datapath request
    input  w_dp_req_t w_dp_req_i,
    /// Write datapath request valid
    input  logic w_dp_valid_i,
    /// Write datapath request ready
    output logic w_dp_ready_o,

    /// Write datapath response
    output w_dp_rsp_t w_dp_rsp_o,
    /// Write datapath response valid
    output logic w_dp_valid_o,
    /// Write datapath response valid
    input  logic w_dp_ready_i,

    /// Read meta request
    input  read_meta_channel_t ar_req_i,
    /// Read meta request valid
    input  logic ar_valid_i,
    /// Read meta request ready
    output logic ar_ready_o,

    /// Write meta request
    input  write_meta_channel_t aw_req_i,
    /// Write meta request valid
    input  logic aw_valid_i,
    /// Write meta request ready
    output logic aw_ready_o,

    /// Datapath poison signal
    input  logic dp_poison_i,

    /// Response channel valid and ready
    output logic r_chan_ready_o,
    output logic r_chan_valid_o,

    /// Read part of the datapath is busy
    output logic r_dp_busy_o,
    /// Write part of the datapath is busy
    output logic w_dp_busy_o,
    /// Buffer is busy
    output logic buffer_busy_o
);

    /// Stobe width
    localparam int unsigned StrbWidth   = DataWidth / 8;

    /// Data type
    typedef logic [DataWidth-1:0] data_t;
    /// Offset type
    typedef logic [StrbWidth-1:0] strb_t;
    /// Byte type
    typedef logic [7:0] byte_t;

    // inbound control signals to the read buffer: controlled by the read process
    strb_t buffer_in_valid;
    strb_t buffer_in_ready;

    // outbound control signals of the buffer: controlled by the write process
    strb_t buffer_out_valid;
    strb_t buffer_out_valid_shifted;
    strb_t buffer_out_ready;
    strb_t buffer_out_ready_shifted;

    // shifted data flowing into the buffer
    byte_t [StrbWidth-1:0] buffer_in;
    byte_t [StrbWidth-1:0] buffer_in_shifted;
    // Introduce this temporary signal to ease tool compatibility
    byte_t [2*StrbWidth-1:0] buffer_in_tmp;

    // aligned and coalesced data leaving the buffer
    byte_t [2*StrbWidth-1:0] buffer_out_tmp;
    byte_t [StrbWidth-1:0] buffer_out;
    byte_t [StrbWidth-1:0] buffer_out_shifted;
    byte_t [StrbWidth-1:0] wr_data;
    strb_t                 wr_valid, wr_strb, mask_ext_shifted, dataflow_ready_in;
    logic                  w_beat_done;


    //--------------------------------------
    // Read Ports
    //--------------------------------------

    idma_axi_read #(
        .StrbWidth  ( StrbWidth           ),
        .byte_t     ( byte_t              ),
        .strb_t     ( strb_t              ),
        .r_dp_req_t ( r_dp_req_t          ),
        .r_dp_rsp_t ( r_dp_rsp_t          ),
        .ar_chan_t  ( read_meta_channel_t ),
        .read_req_t ( axi_req_t           ),
        .read_rsp_t ( axi_rsp_t           )
    ) i_idma_axi_read (
        .clk_i             ( clk_i      ),
        .rst_ni            ( rst_ni     ),
        .r_dp_req_i        ( r_dp_req_i ),
        .r_dp_valid_i      ( r_dp_valid_i ),
        .r_dp_ready_o      ( r_dp_ready_o ),
        .r_dp_rsp_o        ( r_dp_rsp_o ),
        .r_dp_valid_o      ( r_dp_valid_o ),
        .r_dp_ready_i      ( r_dp_ready_i ),
        .ar_req_i          ( ar_req_i ),
        .ar_valid_i        ( ar_valid_i ),
        .ar_ready_o        ( ar_ready_o ),
        .read_req_o        ( axi_read_req_o ),
        .read_rsp_i        ( axi_read_rsp_i ),
        .r_chan_valid_o    ( r_chan_valid_o ),
        .r_chan_ready_o    ( r_chan_ready_o ),
        .buffer_in_o       ( buffer_in ),
        .buffer_in_valid_o ( buffer_in_valid ),
        .buffer_in_ready_i ( buffer_in_ready )
    );

    //--------------------------------------
    // Read Barrel shifter
    //--------------------------------------

    assign buffer_in_tmp = {buffer_in, buffer_in} >> (r_dp_req_i.shift * 8);
    assign buffer_in_shifted = buffer_in_tmp[$bits(buffer_in_shifted)/8-1:0];

    //--------------------------------------
    // Buffer
    //--------------------------------------

    idma_dataflow_element #(
        .BufferDepth   ( BufferDepth   ),
        .StrbWidth     ( StrbWidth     ),
        .PrintFifoInfo ( PrintFifoInfo ),
        .strb_t        ( strb_t        ),
        .byte_t        ( byte_t        )
    ) i_dataflow_element (
        .clk_i       ( clk_i                    ),
        .rst_ni      ( rst_ni                   ),
        .testmode_i  ( testmode_i               ),
        .data_i      ( buffer_in_shifted        ),
        .valid_i     ( buffer_in_valid          ),
        .ready_o     ( buffer_in_ready          ),
        .data_o      ( buffer_out               ),
        .valid_o     ( buffer_out_valid         ),
        .ready_i     ( dataflow_ready_in        )
    );

    //--------------------------------------
    // On-the-fly compute
    //--------------------------------------

    assign wr_data           = buffer_out;
    assign wr_valid          = buffer_out_valid;
    assign wr_strb           = '1;
    assign dataflow_ready_in = buffer_out_ready_shifted;

    //--------------------------------------
    // Write Barrel shifter
    //--------------------------------------

    assign buffer_out_tmp           = {wr_data, wr_data} >> (w_dp_req_i.shift*8);
    assign buffer_out_shifted       = buffer_out_tmp[$bits(buffer_out_shifted)/8-1:0];
    assign buffer_out_valid_shifted = strb_t'({wr_valid, wr_valid} >>   w_dp_req_i.shift);
    assign mask_ext_shifted         = strb_t'({wr_strb, wr_strb} >>   w_dp_req_i.shift);
    assign buffer_out_ready_shifted = strb_t'({buffer_out_ready, buffer_out_ready} >> - w_dp_req_i.shift);

    //--------------------------------------
    // Write Ports
    //--------------------------------------

    idma_axi_write #(
        .StrbWidth       ( StrbWidth            ),
        .MaskInvalidData ( MaskInvalidData      ),
        .byte_t          ( byte_t               ),
        .data_t          ( data_t               ),
        .strb_t          ( strb_t               ),
        .w_dp_req_t      ( w_dp_req_t           ),
        .w_dp_rsp_t      ( w_dp_rsp_t           ),
        .aw_chan_t       ( write_meta_channel_t ),
        .write_req_t     ( axi_req_t ),
        .write_rsp_t     ( axi_rsp_t )
    ) i_idma_axi_write (
        .clk_i              ( clk_i      ),
        .rst_ni             ( rst_ni     ),
        .w_dp_req_i         ( w_dp_req_i ),
        .w_dp_valid_i       ( w_dp_valid_i ),
        .w_dp_ready_o       ( w_dp_ready_o ),
        .dp_poison_i        ( dp_poison_i ),
        .w_dp_rsp_o         ( w_dp_rsp_o ),
        .w_dp_valid_o       ( w_dp_valid_o ),
        .w_dp_ready_i       ( w_dp_ready_i ),
        .aw_req_i           ( aw_req_i ),
        .aw_valid_i         ( aw_valid_i ),
        .aw_ready_o         ( aw_ready_o ),
        .write_req_o        ( axi_write_req_o ),
        .write_rsp_i        ( axi_write_rsp_i ),
        .buffer_out_i       ( buffer_out_shifted ),
        .buffer_out_valid_i ( buffer_out_valid_shifted ),
        .buffer_out_ready_o ( buffer_out_ready ),
        .mask_ext_i         ( mask_ext_shifted ),
        .w_beat_done_o      ( w_beat_done )
    );

    //--------------------------------------
    // Module Control
    //--------------------------------------
    assign r_dp_busy_o   = r_dp_valid_i;
    assign w_dp_busy_o   = w_dp_valid_i | w_dp_ready_o;
    assign buffer_busy_o = |buffer_out_valid;

endmodule

// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Thomas Benz <tbenz@iis.ee.ethz.ch>
// - Tobias Senti <tsenti@ethz.ch>

`include "common_cells/registers.svh"
`include "common_cells/assertions.svh"
`include "idma/guard.svh"

/// Legalizes a generic 1D transfer according to the rules given by the
/// used protocol.
module idma_legalizer_rw_axi #(
    /// Should both data shifts be done before the dataflow element?
    /// If this is enabled, then the data inserted into the dataflow element
    /// will no longer be word aligned, but only a single shifter is needed
    parameter bit          CombinedShifter = 1'b0,
    /// Data width
    parameter int unsigned DataWidth       = 32'd16,
    /// Address width
    parameter int unsigned AddrWidth       = 32'd24,
    /// Burst Len (for actual burst length do 8 byte * 2^(BurstLen))
    parameter int unsigned BurstLen = 4'd8,
    /// 1D iDMA request type:
    /// - `length`: the length of the transfer in bytes
    /// - `*_addr`: the source / target byte addresses of the transfer
    /// - `opt`: the options field
    parameter type idma_req_t        = logic,
    /// Read request type
    parameter type idma_r_req_t      = logic,
    /// Write request type
    parameter type idma_w_req_t      = logic,
    /// Mutable transfer type
    parameter type idma_mut_tf_t     = logic,
    /// Mutable options type
    parameter type idma_mut_tf_opt_t = logic
)(
    /// Clock
    input  logic clk_i,
    /// Asynchronous reset, active low
    input  logic rst_ni,

    /// 1D request
    input  idma_req_t req_i,
    /// 1D request valid
    input  logic valid_i,
    /// 1D request ready
    output logic ready_o,

    /// Read request; contains datapath and meta information
    output idma_r_req_t r_req_o,
    /// Read request valid
    output logic r_valid_o,
    /// Read request ready
    input  logic r_ready_i,

    /// Write request; contains datapath and meta information
    output idma_w_req_t w_req_o,
    /// Write request valid
    output logic w_valid_o,
    /// Write request ready
    input  logic w_ready_i,

    /// Invalidate the current burst transfer, stops emission of requests
    input  logic flush_i,
    /// Kill the active 1D transfer; reload a new transfer
    input  logic kill_i,

    /// Read machine of the legalizer is busy
    output logic r_busy_o,
    /// Write machine of the legalizer is busy
    output logic w_busy_o
);
    /// Stobe width
    localparam int unsigned StrbWidth     = DataWidth / 8;
    /// Offset width
    localparam int unsigned OffsetWidth   = $clog2(StrbWidth);
    /// The size of a page in byte
    localparam int unsigned PageSize      = 256 * StrbWidth > 4096 ? 4096 : 256 * StrbWidth;
    /// The width of page offset byte addresses
    localparam int unsigned PageAddrWidth = $clog2(PageSize);

    /// Offset type
    typedef logic [  OffsetWidth-1:0] offset_t;
    /// Address type
    typedef logic [    AddrWidth-1:0] addr_t;
    /// Page address type
    typedef logic [PageAddrWidth-1:0] page_addr_t;
    /// Page length type
    typedef logic [  PageAddrWidth:0] page_len_t;


    // state: internally hold one transfer, this is mutated
    idma_mut_tf_t     r_tf_d,   r_tf_q;
    idma_mut_tf_t     w_tf_d,   w_tf_q;
    idma_mut_tf_opt_t opt_tf_d, opt_tf_q;

    // enable signals for next mutable transfer storage
    logic r_tf_ena;
    logic w_tf_ena;

    // page boundaries
    page_len_t r_page_num_bytes_to_pb;
    page_len_t r_num_bytes_to_pb;
    page_len_t w_page_num_bytes_to_pb;
    page_len_t w_num_bytes_to_pb;
    page_len_t c_num_bytes_to_pb;

    // read process
    page_len_t r_num_bytes_possible;
    page_len_t r_num_bytes;
    offset_t   r_addr_offset;
    logic      r_done;

    // write process
    page_len_t w_num_bytes_possible;
    page_len_t w_num_bytes;
    offset_t   w_addr_offset;
    logic      w_done;


    //--------------------------------------
    // read boundary check
    //--------------------------------------
    idma_legalizer_page_splitter #(
        .BurstLen      ( BurstLen      ),
        .OffsetWidth   ( OffsetWidth   ),
        .PageAddrWidth ( PageAddrWidth ),
        .addr_t        ( addr_t        ),
        .page_len_t    ( page_len_t    ),
        .page_addr_t   ( page_addr_t   )
    ) i_read_page_splitter (
        .not_bursting_i    ( 1'b0 ),

        .reduce_len_i      ( opt_tf_q.src_reduce_len ),
        .max_llen_i        ( opt_tf_q.src_max_llen   ),

        .addr_i            ( r_tf_q.addr             ),
        .num_bytes_to_pb_o ( r_page_num_bytes_to_pb  )
    );

    assign r_num_bytes_to_pb = r_page_num_bytes_to_pb;

    //--------------------------------------
    // write boundary check
    //--------------------------------------
    idma_legalizer_page_splitter #(
        .BurstLen      ( BurstLen      ),
        .OffsetWidth   ( OffsetWidth   ),
        .PageAddrWidth ( PageAddrWidth ),
        .addr_t        ( addr_t        ),
        .page_len_t    ( page_len_t    ),
        .page_addr_t   ( page_addr_t   )
    ) i_write_page_splitter (
        .not_bursting_i    ( 1'b0 ),

        .reduce_len_i      ( opt_tf_q.dst_reduce_len ),
        .max_llen_i        ( opt_tf_q.dst_max_llen   ),

        .addr_i            ( w_tf_q.addr             ),
        .num_bytes_to_pb_o ( w_page_num_bytes_to_pb  )
    );

    assign w_num_bytes_to_pb = w_page_num_bytes_to_pb;

    //--------------------------------------
    // page boundary check
    //--------------------------------------
    // how many transfers are remaining when concerning both r/w pages?
    // take the boundary that is closer
    assign c_num_bytes_to_pb = (r_num_bytes_to_pb > w_num_bytes_to_pb) ?
                                w_num_bytes_to_pb : r_num_bytes_to_pb;


    //--------------------------------------
    // Synchronized R/W process
    //--------------------------------------
    always_comb begin : proc_num_bytes_possible
        // Default: Coupled
        r_num_bytes_possible = c_num_bytes_to_pb;
        w_num_bytes_possible = c_num_bytes_to_pb;

        if (opt_tf_q.decouple_rw) begin
            r_num_bytes_possible = r_num_bytes_to_pb;
            w_num_bytes_possible = w_num_bytes_to_pb;
        end
    end

    assign r_addr_offset = r_tf_q.addr[OffsetWidth-1:0];
    assign w_addr_offset = w_tf_q.addr[OffsetWidth-1:0];

    // legalization process -> read and write is coupled together
    always_comb begin : proc_read_write_transaction

        // default: keep state
        r_tf_d   = r_tf_q;
        w_tf_d   = w_tf_q;
        opt_tf_d = opt_tf_q;

        // default: not done
        r_done = 1'b0;
        w_done = 1'b0;

        //--------------------------------------
        // Legalize read transaction
        //--------------------------------------
        // more bytes remaining than we can read
        if (r_tf_q.length > r_num_bytes_possible) begin
            r_num_bytes = r_num_bytes_possible;
            // calculate remainder
            r_tf_d.length = r_tf_q.length - r_num_bytes_possible;
            // next address
            r_tf_d.addr = r_tf_q.addr + r_num_bytes;

        // remaining bytes fit in one burst
        end else begin
            r_num_bytes = r_tf_q.length[PageAddrWidth:0];
            // finished
            r_tf_d.valid = 1'b0;
            r_done = 1'b1;
        end

        //--------------------------------------
        // Legalize write transaction
        //--------------------------------------
        // more bytes remaining than we can write
        if (w_tf_q.length > w_num_bytes_possible) begin
            w_num_bytes = w_num_bytes_possible;
            // calculate remainder
            w_tf_d.length = w_tf_q.length - w_num_bytes_possible;
            // next address
            w_tf_d.addr = w_tf_q.addr + w_num_bytes;

        // remaining bytes fit in one burst
        end else begin
            w_num_bytes = w_tf_q.length[PageAddrWidth:0];
            // finished
            w_tf_d.valid = 1'b0;
            w_done = 1'b1;
        end

        //--------------------------------------
        // Kill
        //--------------------------------------
        if (kill_i) begin
            // kill the current state
            r_tf_d = '0;
            w_tf_d = '0;
            r_done = 1'b1;
            w_done = 1'b1;
        end

        //--------------------------------------
        // Refill
        //--------------------------------------
        // new request is taken in if both r and w machines are ready.
        if (ready_o & valid_i) begin

            // load all three mutable objects (source, destination, option)
            // source or read
            r_tf_d = '{
                length: req_i.length,
                addr:   req_i.src_addr,
                valid:   1'b1,
                base_addr: req_i.src_addr,
                default: '0
            };
            // destination or write
            w_tf_d = '{
                length: req_i.length,
                addr:   req_i.dst_addr,
                valid:   1'b1,
                base_addr: req_i.dst_addr,
                user: req_i.user,
                default: '0
            };
            // options
            opt_tf_d = '{
                src_protocol:   req_i.opt.src_protocol,
                dst_protocol:   req_i.opt.dst_protocol,
                src_head:       req_i.opt.src_head,
                dst_head:       req_i.opt.dst_head,
                read_shift:     '0,
                write_shift:    '0,
                decouple_rw:    req_i.opt.beo.decouple_rw,
                decouple_aw:    req_i.opt.beo.decouple_aw,
                src_max_llen:   req_i.opt.beo.src_max_llen,
                dst_max_llen:   req_i.opt.beo.dst_max_llen,
                src_reduce_len: req_i.opt.beo.src_reduce_len,
                dst_reduce_len: req_i.opt.beo.dst_reduce_len,
                axi_id:         req_i.opt.axi_id,
                src_axi_opt:    req_i.opt.src,
                dst_axi_opt:    req_i.opt.dst,
                super_last:     req_i.opt.last,
                compute:        req_i.opt.compute
            };
            // determine shift amount
            if (CombinedShifter) begin
                opt_tf_d.read_shift  = req_i.src_addr[OffsetWidth-1:0] -
                                       req_i.dst_addr[OffsetWidth-1:0];
                opt_tf_d.write_shift = '0;
            end else begin
                opt_tf_d.read_shift  =   req_i.src_addr[OffsetWidth-1:0];
                opt_tf_d.write_shift = - req_i.dst_addr[OffsetWidth-1:0];
            end
        end
    end


    //--------------------------------------
    // Connect outputs
    //--------------------------------------

    // Read meta channel
    always_comb begin
        r_req_o.ar_req.axi.ar_chan = '{
            id: opt_tf_q.axi_id,
            addr: { r_tf_q.addr[AddrWidth-1:OffsetWidth], {{OffsetWidth}{1'b0}} },
            len: ((r_num_bytes + r_addr_offset - 'd1) >> OffsetWidth),
            size: axi_pkg::size_t'(OffsetWidth),
            burst: opt_tf_q.src_axi_opt.burst,
            lock: opt_tf_q.src_axi_opt.lock,
            cache: opt_tf_q.src_axi_opt.cache,
            prot: opt_tf_q.src_axi_opt.prot,
            qos: opt_tf_q.src_axi_opt.qos,
            region: opt_tf_q.src_axi_opt.region,
            user: '0
        };
    end

    // assign the signals needed to set-up the read data path
    assign r_req_o.r_dp_req = '{
        src_protocol: opt_tf_q.src_protocol,
        src_head:     opt_tf_q.src_head,
        offset:       r_addr_offset,
        tailer:       OffsetWidth'(r_num_bytes + r_addr_offset),
        shift:        opt_tf_q.read_shift,
        decouple_aw:  opt_tf_q.decouple_aw,
        is_single:    r_num_bytes <= StrbWidth
    };

    // Write meta channel and data path
    always_comb begin
        w_req_o.aw_req.axi.aw_chan = '{
            id: opt_tf_q.axi_id,
            addr: { w_tf_q.addr[AddrWidth-1:OffsetWidth], {{OffsetWidth}{1'b0}} },
            len: ((w_num_bytes + w_addr_offset - 'd1) >> OffsetWidth),
            size: axi_pkg::size_t'(OffsetWidth),
            burst: opt_tf_q.dst_axi_opt.burst,
            lock: opt_tf_q.dst_axi_opt.lock,
            cache: opt_tf_q.dst_axi_opt.cache,
            prot: opt_tf_q.dst_axi_opt.prot,
            qos: opt_tf_q.dst_axi_opt.qos,
            region: opt_tf_q.dst_axi_opt.region,
            user: w_tf_q.user,
            atop: '0
        };
        w_req_o.w_dp_req = '{
            dst_protocol: opt_tf_q.dst_protocol,
            dst_head: opt_tf_q.dst_head,
            offset: w_addr_offset,
            tailer: OffsetWidth'(w_num_bytes + w_addr_offset),
            shift: opt_tf_q.write_shift,
            num_beats: w_req_o.aw_req.axi.aw_chan.len,
            is_single: w_req_o.aw_req.axi.aw_chan.len == '0,
            compute: opt_tf_q.compute
        };
        
    end

    // last burst in generic 1D transfer?
    assign w_req_o.last = w_done;

    // last burst indicated by midend
    assign w_req_o.super_last = opt_tf_q.super_last;

    // assign aw decouple flag
    assign w_req_o.decouple_aw = opt_tf_q.decouple_aw;

    // busy output
    assign r_busy_o = r_tf_q.valid;
    assign w_busy_o = w_tf_q.valid;


    //--------------------------------------
    // Flow Control
    //--------------------------------------
    // only advance to next state if:
    // * rw_coupled: both machines advance
    // * rw_decoupled: either machine advances

    always_comb begin : proc_legalizer_flow_control
        if ( opt_tf_q.decouple_rw) begin
            r_tf_ena  = (r_ready_i & !flush_i) | kill_i;
            w_tf_ena  = (w_ready_i & !flush_i) | kill_i;

            r_valid_o = r_tf_q.valid & r_ready_i & !flush_i;
            w_valid_o = w_tf_q.valid & w_ready_i & !flush_i;
        end else begin
            r_tf_ena  = (r_ready_i & w_ready_i & !flush_i) | kill_i;
            w_tf_ena  = (r_ready_i & w_ready_i & !flush_i) | kill_i;

            r_valid_o = r_tf_q.valid & w_ready_i & r_ready_i & !flush_i;
            w_valid_o = w_tf_q.valid & r_ready_i & w_ready_i & !flush_i;
        end
    end

    // load next idma request: if both machines are done!
    assign ready_o = r_done & w_done & r_ready_i & w_ready_i & !flush_i;


    //--------------------------------------
    // State
    //--------------------------------------
    `FF (opt_tf_q, opt_tf_d,           '0, clk_i, rst_ni)
    `FFL(r_tf_q,   r_tf_d,   r_tf_ena, '0, clk_i, rst_ni)
    `FFL(w_tf_q,   w_tf_d,   w_tf_ena, '0, clk_i, rst_ni)


    //--------------------------------------
    // Assertions
    //--------------------------------------
    // only support the decomposition of incremental bursts
    `ASSERT_NEVER(OnlyIncrementalBurstsSRC, (ready_o & valid_i &
                  req_i.opt.src.burst != axi_pkg::BURST_INCR), clk_i, !rst_ni)
    `ASSERT_NEVER(OnlyIncrementalBurstsDST, (ready_o & valid_i &
                  req_i.opt.dst.burst != axi_pkg::BURST_INCR), clk_i, !rst_ni)

endmodule

// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Thomas Benz <tbenz@iis.ee.ethz.ch>
// - Tobias Senti <tsenti@ethz.ch>

`include "axi/typedef.svh"
`include "idma/guard.svh"

/// The iDMA backend implements an arbitrary 1D copy engine
module idma_backend_rw_axi #(
    /// Data width
    parameter int unsigned DataWidth        = 32'd16,
    /// Address width
    parameter int unsigned AddrWidth        = 32'd24,
    /// AXI user width
    parameter int unsigned UserWidth        = 32'd1,
    /// AXI ID width
    parameter int unsigned AxiIdWidth       = 32'd1,
    /// Number of transaction that can be in-flight concurrently
    parameter int unsigned NumAxInFlight    = 32'd2,
    /// The depth of the internal reorder buffer:
    /// - '2': minimal possible configuration
    /// - '3': efficiently handle misaligned transfers (recommended)
    parameter int unsigned BufferDepth      = 32'd2,
    /// With of a transfer: max transfer size is `2**TFLenWidth` bytes
    parameter int unsigned TFLenWidth       = 32'd24,
    /// The depth of the memory system the backend is attached to
    parameter int unsigned MemSysDepth      = 32'd0,
    /// Burst Len (for actual burst length do 8 byte * 2^(BurstLen))
    parameter int unsigned BurstLen = 4'd8,
    /// Should both data shifts be done before the dataflow element?
    /// If this is enabled, then the data inserted into the dataflow element
    /// will no longer be word aligned, but only a single shifter is needed
    parameter bit          CombinedShifter  = 1'b0,
    /// Should the `R`-`AW` coupling hardware be present? (recommended)
    parameter bit          RAWCouplingAvail = 1'b1,
    /// Mask invalid data on the manager interface
    parameter bit MaskInvalidData            = 1'b1,
    /// Should hardware legalization be present? (recommended)
    /// If not, software legalization is required to ensure the transfers are
    /// AXI4-conformal
    parameter bit HardwareLegalizer          = 1'b1,
    /// Reject zero-length transfers
    parameter bit RejectZeroTransfers        = 1'b1,
    /// Should the error handler be present?
    parameter idma_pkg::error_cap_e ErrorCap = idma_pkg::NO_ERROR_HANDLING,
    /// Print the info of the FIFO configuration
    parameter bit PrintFifoInfo              = 1'b0,
    /// 1D iDMA request type
    parameter type idma_req_t                = logic,
    /// iDMA response type
    parameter type idma_rsp_t                = logic,
    /// Error Handler request type
    parameter type idma_eh_req_t             = logic,
    /// iDMA busy signal
    parameter type idma_busy_t               = logic,
    /// AXI4+ATOP Request and Response channel type
    parameter type axi_req_t = logic,
    parameter type axi_rsp_t = logic,
    /// Address Read Channel type
    parameter type read_meta_channel_t  = logic,
    /// Address Write Channel type
    parameter type write_meta_channel_t = logic,
    /// Strobe Width (do not override!)
    parameter int unsigned StrbWidth    = DataWidth / 8,
    /// Offset Width (do not override!)
    parameter int unsigned OffsetWidth  = $clog2(StrbWidth)
)(
    /// Clock
    input  logic clk_i,
    /// Asynchronous reset, active low
    input  logic rst_ni,
    /// Testmode in
    input  logic testmode_i,

    /// 1D iDMA request
    input  idma_req_t idma_req_i,
    /// 1D iDMA request valid
    input  logic req_valid_i,
    /// 1D iDMA request ready
    output logic req_ready_o,

    /// iDMA response
    output idma_rsp_t idma_rsp_o,
    /// iDMA response valid
    output logic rsp_valid_o,
    /// iDMA response ready
    input  logic rsp_ready_i,

    /// Error handler request
    input  idma_eh_req_t idma_eh_req_i,
    /// Error handler request valid
    input  logic eh_req_valid_i,
    /// Error handler request ready
    output logic eh_req_ready_o,

    /// AXI4+ATOP read request
    output axi_req_t axi_read_req_o,
    /// AXI4+ATOP read response
    input  axi_rsp_t axi_read_rsp_i,

    /// AXI4+ATOP write request
    output axi_req_t axi_write_req_o,
    /// AXI4+ATOP write response
    input  axi_rsp_t axi_write_rsp_i,

    /// iDMA busy flags
    output idma_busy_t busy_o
);

    /// Extra write-descriptor slots covering the compute (transpose) tile-fill latency
    localparam int unsigned ComputeFifoDepth = 32'd0;

    /// The localparam MetaFifoDepth holds the maximum number of transfers that can be
    /// in-flight under any circumstances.
    localparam int unsigned MetaFifoDepth = BufferDepth + NumAxInFlight + MemSysDepth + ComputeFifoDepth;

    /// Address type
    typedef logic [AddrWidth-1:0]   addr_t;
    /// DAta type
    typedef logic [DataWidth-1:0]   data_t;
    /// Strobe type
    typedef logic [StrbWidth-1:0]   strb_t;
    /// User type
    typedef logic [UserWidth-1:0]   user_t;
    /// ID type
    typedef logic [AxiIdWidth-1:0]  id_t;
    /// Offset type
    typedef logic [OffsetWidth-1:0] offset_t;
    /// Transfer length type
    typedef logic [TFLenWidth-1:0]  tf_len_t;

    /// The datapath read request type holds all the information required to configure the read
    /// part of the datapath. The type consists of:
    /// - `offset`: The bus offset of the read
    /// - `trailer`: How many empty bytes are required to pad the transfer to a multiple of the
    ///              bus width.
    /// - `shift`: The amount the data needs to be shifted
    /// - `decouple_aw`: If the transfer has the AW decoupled from the R
    /// - `is_single`: Is this transfer just one beat long? `(len == 0)`
    typedef struct packed {
        idma_pkg::protocol_e  src_protocol;
        idma_pkg::multihead_t src_head;  // ignored unless multi-head (one head: tied 0)
        offset_t              offset;
        offset_t              tailer;
        offset_t              shift;
        logic                 decouple_aw;
        logic                 is_single;
    } r_dp_req_t;

    /// The datapath read response type provides feedback from the read part of the datapath:
    /// - `resp`: The response from the R channel of the AXI4 manager interface
    /// - `last`: The last flag from the R channel of the AXI4 manager interface
    /// - `first`: Is the current item first beat in the burst
    typedef struct packed {
        axi_pkg::resp_t resp;
        logic           last;
        logic           first;
    } r_dp_rsp_t;

    /// The datapath write request type holds all the information required to configure the write
    /// part of the datapath. The type consists of:
    /// - `offset`: The bus offset of the write
    /// - `trailer`: How many empty bytes are required to pad the transfer to a multiple of the
    ///              bus width.
    /// - `shift`: The amount the data needs to be shifted
    /// - `num_beats`: The number of beats this burst consist of
    /// - `is_single`: Is this transfer just one beat long? `(len == 0)`
    typedef struct packed {
        idma_pkg::protocol_e  dst_protocol;
        idma_pkg::multihead_t dst_head;  // ignored unless multi-head (one head: tied 0)
        offset_t              offset;
        offset_t              tailer;
        offset_t              shift;
        axi_pkg::len_t        num_beats;
        logic                 is_single;
        idma_pkg::compute_options_t compute;
    } w_dp_req_t;

    /// The datapath write response type provides feedback from the write part of the datapath:
    /// - `resp`: The response from the B channel of the AXI4 manager interface
    /// - `user`: The user field from the B channel of the AXI4 manager interface
    typedef struct packed {
        axi_pkg::resp_t resp;
        user_t          user;
    } w_dp_rsp_t;

    /// The iDMA read request bundles an `AR` type and a datapath read response type together.
    typedef struct packed {
        r_dp_req_t          r_dp_req;
        read_meta_channel_t ar_req;
    } idma_r_req_t;

    /// The iDMA write request bundles an `AW` type and a datapath write response type together. It
    /// has an additional flags:
    /// - `last`: indicating the current burst is the last one of the generic 1D transfer currently
    ///    being processed
    /// - `midend_last`: The current transfer is marked by the controlling as last
    /// - `decouple_aw`: indicates this is an R-AW decoupled transfer
    typedef struct packed {
        w_dp_req_t           w_dp_req;
        write_meta_channel_t aw_req;
        logic                last;
        logic                super_last;
        logic                decouple_aw;
    } idma_w_req_t;

    /// The mutable transfer options type holds important information that is mutated by the
    /// `legalizer` block.
    typedef struct packed {
        idma_pkg::protocol_e    src_protocol;
        idma_pkg::protocol_e    dst_protocol;
        idma_pkg::multihead_t   src_head;
        idma_pkg::multihead_t   dst_head;
        offset_t                read_shift;
        offset_t                write_shift;
        logic                   decouple_rw;
        logic                   decouple_aw;
        logic [2:0]             src_max_llen;
        logic [2:0]             dst_max_llen;
        logic                   src_reduce_len;
        logic                   dst_reduce_len;
        id_t                    axi_id;
        idma_pkg::axi_options_t src_axi_opt;
        idma_pkg::axi_options_t dst_axi_opt;
        logic                   super_last;
        idma_pkg::compute_options_t compute;
    } idma_mut_tf_opt_t;

    /// The mutable transfer type holds important information that is mutated by the
    /// `legalizer` block.
    typedef struct packed {
        tf_len_t length;
        addr_t   addr;
        logic    valid;
        addr_t   base_addr;
        user_t   user;
    } idma_mut_tf_t;


    // datapath busy indicates the datapath is actively working on a transfer. It is composed of
    // the activity of the buffer as well as both the read and write machines
    logic dp_busy;
    // blanks invalid data
    logic dp_poison;

    // read and write requests and their handshaking signals
    idma_r_req_t r_req;
    idma_w_req_t w_req;
    logic        r_valid, w_valid;
    logic        r_ready, w_ready;

    // It the current transfer the last burst in the 1D transfer?
    logic w_last_burst;
    logic w_last_ready;

    // Super last flag: The current transfer is indicated as the last one by the controlling
    // unit; e.g. by a midend
    logic w_super_last;

    // Datapath FIFO signals -> used to decouple legalizer and datapath
    logic r_dp_req_in_ready,  w_dp_req_in_ready;
    logic r_dp_req_out_valid, w_dp_req_out_valid;
    logic r_dp_req_out_ready, w_dp_req_out_ready;
    r_dp_req_t r_dp_req_out;
    w_dp_req_t w_dp_req_out;

    // datapah responses
    r_dp_rsp_t r_dp_rsp;
    w_dp_rsp_t w_dp_rsp;
    logic r_dp_rsp_valid, w_dp_rsp_valid;
    logic r_dp_rsp_ready, w_dp_rsp_ready;

    // Ax handshaking
    logic ar_ready,    ar_ready_dp;
    logic aw_ready,    aw_ready_dp;
    logic aw_valid_dp, ar_valid_dp;

    // Ax request from R-AW coupler to datapath
    write_meta_channel_t aw_req_dp;

    // Ax request from the decoupling stage to the datapath
    read_meta_channel_t ar_req_dp;

    // flush and preemptively empty the legalizer
    logic legalizer_flush, legalizer_kill;

    /// intermediate signals to reject zero length transfers
    logic      is_length_zero;
    logic      req_valid;
    idma_rsp_t idma_rsp;
    logic      rsp_valid;
    logic      rsp_ready;

    // Respone Channel valid and ready -> needed for bursting
    logic r_chan_valid;
    logic r_chan_ready;

    //--------------------------------------
    // Reject Zero Length Transfers
    //--------------------------------------
    if (RejectZeroTransfers) begin : gen_reject_zero_transfers
        // is the current transfer length 0?
        assign is_length_zero = idma_req_i.length == '0;

        // bypass valid as long as length is not zero, otherwise suppress it
        assign req_valid = is_length_zero ? 1'b0 : req_valid_i;

        // modify response
        always_comb begin : proc_modify_response_zero_length
            // default: bypass
            idma_rsp_o  = idma_rsp;
            rsp_ready   = rsp_ready_i;
            rsp_valid_o = rsp_valid;

            // a zero transfer happens
            if (is_length_zero & req_valid_i & req_ready_o) begin
                // block backend
                rsp_ready = 1'b0;
                // generate new response
                rsp_valid_o             = 1'b1;
                idma_rsp_o              =  '0;
                idma_rsp_o.last         = 1'b1;
                idma_rsp_o.error        = 1'b1;
                idma_rsp_o.pld.err_type = idma_pkg::BACKEND;
            end
        end

    // just bypass signals
    end else begin : gen_bypass_zero_transfers
        // bypass
        assign req_valid   = req_valid_i;
        assign idma_rsp_o  = idma_rsp;
        assign rsp_ready   = rsp_ready_i;
        assign rsp_valid_o = rsp_valid;
    end


    //--------------------------------------
    // Legalization
    //--------------------------------------
    if (HardwareLegalizer) begin : gen_hw_legalizer
        // hardware legalizer is present
        idma_legalizer_rw_axi #(
            .CombinedShifter   ( CombinedShifter   ),
            .DataWidth         ( DataWidth         ),
            .AddrWidth         ( AddrWidth         ),
            .BurstLen          ( BurstLen          ),
            .idma_req_t        ( idma_req_t        ),
            .idma_r_req_t      ( idma_r_req_t      ),
            .idma_w_req_t      ( idma_w_req_t      ),
            .idma_mut_tf_t     ( idma_mut_tf_t     ),
            .idma_mut_tf_opt_t ( idma_mut_tf_opt_t )
        ) i_idma_legalizer (
            .clk_i     ( clk_i             ),
            .rst_ni    ( rst_ni            ),
            .req_i     ( idma_req_i        ),
            .valid_i   ( req_valid         ),
            .ready_o   ( req_ready_o       ),
            .r_req_o   ( r_req             ),
            .w_req_o   ( w_req             ),
            .r_valid_o ( r_valid           ),
            .w_valid_o ( w_valid           ),
            .r_ready_i ( r_ready           ),
            .w_ready_i ( w_ready           ),
            .flush_i   ( legalizer_flush   ),
            .kill_i    ( legalizer_kill    ),
            .r_busy_o  ( busy_o.r_leg_busy ),
            .w_busy_o  ( busy_o.w_leg_busy )
        );

    end else begin : gen_no_hw_legalizer
        // stream fork is used to synchronize the two decoupled channels without the need for a
        // FIFO here.
        stream_fork #(
            .N_OUP   ( 32'd2 )
        ) i_stream_fork (
            .clk_i   ( clk_i                ),
            .rst_ni  ( rst_ni               ),
            .valid_i ( req_valid            ),
            .ready_o ( req_ready_o          ),
            .valid_o ( { r_valid, w_valid } ),
            .ready_i ( { r_ready, w_ready } )
        );

        // local signal holding the length -> explicitly only doing the computation once
        axi_pkg::len_t len;
        assign len = ((idma_req_i.length + idma_req_i.src_addr[OffsetWidth-1:0] -
                     'd1) >> OffsetWidth);

        // assemble read datapath request
        assign r_req.r_dp_req = '{
            src_protocol: idma_req_i.opt.src_protocol,
            src_head:     idma_req_i.opt.src_head,
            offset:       idma_req_i.src_addr[OffsetWidth-1:0],
            tailer:       OffsetWidth'(idma_req_i.length + idma_req_i.src_addr[OffsetWidth-1:0]),
            shift:        OffsetWidth'(idma_req_i.src_addr[OffsetWidth-1:0]),
            decouple_aw:  idma_req_i.opt.beo.decouple_aw,
            is_single:    len == '0
        };

        // assemble write datapath request
        assign w_req.w_dp_req = '{
            dst_protocol: idma_req_i.opt.dst_protocol,
            dst_head:     idma_req_i.opt.dst_head,
            offset:       idma_req_i.dst_addr[OffsetWidth-1:0],
            tailer:       OffsetWidth'(idma_req_i.length + idma_req_i.dst_addr[OffsetWidth-1:0]),
            shift:        OffsetWidth'(- idma_req_i.dst_addr[OffsetWidth-1:0]),
            num_beats:    len,
            is_single:    len == '0,
            compute:      idma_req_i.opt.compute
        };

        // if the legalizer is bypassed; every burst is the last of the 1D transfer
        assign w_req.last = 1'b1;

        // assign the last flag of the controlling unit
        assign w_req.super_last = idma_req_i.opt.last;

        // bypass decouple signal
        assign w_req.decouple_aw = idma_req_i.opt.beo.decouple_aw;

        // there is no unit to be busy
        assign busy_o.r_leg_busy = 1'b0;
        assign busy_o.w_leg_busy = 1'b0;
    end

    // data path, meta channels, and last queues have to be ready for the legalizer to be ready
    assign r_ready = r_dp_req_in_ready & ar_ready;
    assign w_ready = w_dp_req_in_ready & aw_ready & w_last_ready;


    //--------------------------------------
    // Error handler
    //--------------------------------------
    if (ErrorCap == idma_pkg::ERROR_HANDLING) begin : gen_error_handler
        idma_error_handler #(
            .MetaFifoDepth ( MetaFifoDepth ),
            .PrintFifoInfo ( PrintFifoInfo ),
            .idma_rsp_t    ( idma_rsp_t    ),
            .idma_eh_req_t ( idma_eh_req_t ),
            .addr_t        ( addr_t        ),
            .r_dp_rsp_t    ( r_dp_rsp_t    ),
            .w_dp_rsp_t    ( w_dp_rsp_t    )
        ) i_idma_error_handler (
            .clk_i             ( clk_i              ),
            .rst_ni            ( rst_ni             ),
            .testmode_i        ( testmode_i         ),
            .rsp_o             ( idma_rsp           ),
            .rsp_valid_o       ( rsp_valid          ),
            .rsp_ready_i       ( rsp_ready          ),
            .req_valid_i       ( req_valid          ),
            .req_ready_i       ( req_ready_o        ),
            .eh_i              ( idma_eh_req_i      ),
            .eh_valid_i        ( eh_req_valid_i     ),
            .eh_ready_o        ( eh_req_ready_o     ),
            .r_addr_i          ( r_req.ar_req.axi.ar_chan.addr ),
            .w_addr_i          ( w_req.aw_req.axi.aw_chan.addr ),
            .r_consume_i       ( r_valid & r_ready  ),
            .w_consume_i       ( w_valid & w_ready  ),
            .legalizer_flush_o ( legalizer_flush    ),
            .legalizer_kill_o  ( legalizer_kill     ),
            .dp_busy_i         ( dp_busy            ),
            .dp_poison_o       ( dp_poison          ),
            .r_dp_rsp_i        ( r_dp_rsp           ),
            .r_dp_valid_i      ( r_dp_rsp_valid     ),
            .r_dp_ready_o      ( r_dp_rsp_ready     ),
            .w_dp_rsp_i        ( w_dp_rsp           ),
            .w_dp_valid_i      ( w_dp_rsp_valid     ),
            .w_dp_ready_o      ( w_dp_rsp_ready     ),
            .w_last_burst_i    ( w_last_burst       ),
            .w_super_last_i    ( w_super_last       ),
            .fsm_busy_o        ( busy_o.eh_fsm_busy ),
            .cnt_busy_o        ( busy_o.eh_cnt_busy )
        );
    end else if (ErrorCap == idma_pkg::NO_ERROR_HANDLING) begin : gen_no_error_handler
        // bypass the signals, assign their neutral values
        assign idma_rsp.error     = 1'b0;
        assign idma_rsp.pld       = 1'b0;
        assign idma_rsp.last      = w_super_last;
        assign rsp_valid          = w_dp_rsp_valid & w_last_burst;
        assign eh_req_ready_o     = 1'b0;
        assign legalizer_flush    = 1'b0;
        assign legalizer_kill     = 1'b0;
        assign dp_poison          = 1'b0;
        assign r_dp_rsp_ready     = rsp_ready;
        assign w_dp_rsp_ready     = rsp_ready;
        assign busy_o.eh_fsm_busy = 1'b0;
        assign busy_o.eh_cnt_busy = 1'b0;

    end else begin : gen_param_error
        `IDMA_NONSYNTH_BLOCK(
        initial begin
            $fatal(1, "Unexpected Error Capability");
        end
        )
    end


    //--------------------------------------
    // Datapath busy signal
    //--------------------------------------
    assign dp_busy = busy_o.buffer_busy |
                     busy_o.r_dp_busy   |
                     busy_o.w_dp_busy;


    //--------------------------------------
    // Datapath decoupling
    //--------------------------------------
    stream_fifo_optimal_wrap #(
        .Depth     ( NumAxInFlight ),
        .type_t    ( r_dp_req_t    ),
        .PrintInfo ( PrintFifoInfo )
    ) i_r_dp_req (
        .clk_i      ( clk_i               ),
        .rst_ni     ( rst_ni              ),
        .testmode_i ( testmode_i          ),
        .flush_i    ( 1'b0                ),
        .usage_o    ( /* NOT CONNECTED */ ),
        .data_i     ( r_req.r_dp_req      ),
        .valid_i    ( r_valid             ),
        .ready_o    ( r_dp_req_in_ready   ),
        .data_o     ( r_dp_req_out        ),
        .valid_o    ( r_dp_req_out_valid  ),
        .ready_i    ( r_dp_req_out_ready  )
    );

    stream_fifo_optimal_wrap #(
        .Depth     ( NumAxInFlight + ComputeFifoDepth ),
        .type_t    ( w_dp_req_t    ),
        .PrintInfo ( PrintFifoInfo )
    ) i_w_dp_req (
        .clk_i      ( clk_i               ),
        .rst_ni     ( rst_ni              ),
        .testmode_i ( testmode_i          ),
        .flush_i    ( 1'b0                ),
        .usage_o    ( /* NOT CONNECTED */ ),
        .data_i     ( w_req.w_dp_req      ),
        .valid_i    ( w_valid             ),
        .ready_o    ( w_dp_req_in_ready   ),
        .data_o     ( w_dp_req_out        ),
        .valid_o    ( w_dp_req_out_valid  ),
        .ready_i    ( w_dp_req_out_ready  )
    );

    // Add fall-through register to allow the input to be ready if the output is not. This
    // does not add a cycle of delay

    fall_through_register #(
        .T          ( read_meta_channel_t )
    ) i_ar_fall_through_register (
        .clk_i      ( clk_i             ),
        .rst_ni     ( rst_ni            ),
        .testmode_i ( testmode_i        ),
        .clr_i      ( 1'b0              ),
        .valid_i    ( r_valid           ),
        .ready_o    ( ar_ready          ),
        .data_i     ( r_req.ar_req ),
        .valid_o    ( ar_valid_dp       ),
        .ready_i    ( ar_ready_dp       ),
        .data_o     ( ar_req_dp         )
    );


    //--------------------------------------
    // Last flag store
    //--------------------------------------
    stream_fifo_optimal_wrap #(
        .Depth        ( MetaFifoDepth ),
        .type_t       ( logic [1:0]   ),
        .PrintInfo    ( PrintFifoInfo )
    ) i_w_last (
        .clk_i      ( clk_i                           ),
        .rst_ni     ( rst_ni                          ),
        .testmode_i ( testmode_i                      ),
        .flush_i    ( 1'b0                            ),
        .usage_o    ( /* NOT CONNECTED */             ),
        .data_i     ( {w_req.super_last, w_req.last}  ),
        .valid_i    ( w_valid & w_ready               ),
        .ready_o    ( w_last_ready                    ),
        .data_o     ( {w_super_last, w_last_burst}    ),
        .valid_o    ( /* NOT CONNECTED */             ),
        .ready_i    ( w_dp_rsp_valid & w_dp_rsp_ready )
    );

    //--------------------------------------
    // Transport Layer / Datapath
    //--------------------------------------
    idma_transport_layer_rw_axi #(
        .NumAxInFlight               ( NumAxInFlight               ),
        .DataWidth                   ( DataWidth                   ),
        .BufferDepth                 ( BufferDepth                 ),
        .MaskInvalidData             ( MaskInvalidData             ),
        .PrintFifoInfo               ( PrintFifoInfo               ),
        .r_dp_req_t                  ( r_dp_req_t                  ),
        .w_dp_req_t                  ( w_dp_req_t                  ),
        .r_dp_rsp_t                  ( r_dp_rsp_t                  ),
        .w_dp_rsp_t                  ( w_dp_rsp_t                  ),
        .write_meta_channel_t        ( write_meta_channel_t        ),
        .read_meta_channel_t         ( read_meta_channel_t         ),
        .axi_req_t                   ( axi_req_t                   ),
        .axi_rsp_t                   ( axi_rsp_t                   )
    ) i_idma_transport_layer (
        .clk_i           ( clk_i                ),
        .rst_ni          ( rst_ni               ),
        .testmode_i      ( testmode_i           ),
        .axi_read_req_o  ( axi_read_req_o       ),
        .axi_read_rsp_i  ( axi_read_rsp_i       ),
        .axi_write_req_o ( axi_write_req_o      ),
        .axi_write_rsp_i ( axi_write_rsp_i      ),
        .r_dp_req_i      ( r_dp_req_out         ),
        .r_dp_valid_i    ( r_dp_req_out_valid   ),
        .r_dp_ready_o    ( r_dp_req_out_ready   ),
        .r_dp_rsp_o      ( r_dp_rsp             ),
        .r_dp_valid_o    ( r_dp_rsp_valid       ),
        .r_dp_ready_i    ( r_dp_rsp_ready       ),
        .w_dp_req_i      ( w_dp_req_out         ),
        .w_dp_valid_i    ( w_dp_req_out_valid   ),
        .w_dp_ready_o    ( w_dp_req_out_ready   ),
        .w_dp_rsp_o      ( w_dp_rsp             ),
        .w_dp_valid_o    ( w_dp_rsp_valid       ),
        .w_dp_ready_i    ( w_dp_rsp_ready       ),
        .ar_req_i        ( ar_req_dp            ),
        .ar_valid_i      ( ar_valid_dp          ),
        .ar_ready_o      ( ar_ready_dp          ),
        .aw_req_i        ( aw_req_dp            ),
        .aw_valid_i      ( aw_valid_dp          ),
        .aw_ready_o      ( aw_ready_dp          ),
        .dp_poison_i     ( dp_poison            ),
        .r_dp_busy_o     ( busy_o.r_dp_busy     ),
        .w_dp_busy_o     ( busy_o.w_dp_busy     ),
        .buffer_busy_o   ( busy_o.buffer_busy   ),
        .r_chan_ready_o  ( r_chan_ready         ),
        .r_chan_valid_o  ( r_chan_valid         )
    );

    //--------------------------------------
    // R-AW channel coupler
    //--------------------------------------

    if (RAWCouplingAvail) begin : gen_r_aw_coupler
        // instantiate the channel coupler
        idma_channel_coupler #(
            .NumAxInFlight   ( NumAxInFlight               ),
            .AddrWidth       ( AddrWidth                   ),
            .UserWidth       ( UserWidth                   ),
            .AxiIdWidth      ( AxiIdWidth                  ),
            .PrintFifoInfo   ( PrintFifoInfo               ),
            .axi_aw_chan_t   ( write_meta_channel_t        )
        ) i_idma_channel_coupler (
            .clk_i            ( clk_i                       ),
            .rst_ni           ( rst_ni                      ),
            .testmode_i       ( testmode_i                  ),
            .r_rsp_valid_i    ( r_chan_valid                ),
            .r_rsp_ready_i    ( r_chan_ready                ),
            .r_rsp_first_i    ( r_dp_rsp.first              ),
            .r_decouple_aw_i  ( r_dp_req_out.decouple_aw    ),
            .aw_decouple_aw_i ( w_req.decouple_aw ),
            .aw_req_i         ( w_req.aw_req                ),
            .aw_valid_i       ( w_valid                     ),
            .aw_ready_o       ( aw_ready                    ),
            .aw_req_o         ( aw_req_dp                   ),
            .aw_valid_o       ( aw_valid_dp                 ),
            .aw_ready_i       ( aw_ready_dp                 ),
            .busy_o           ( busy_o.raw_coupler_busy     )
        );
    end else begin : gen_r_aw_bypass
        // Add fall-through register to allow the input to be ready if the output is not. This
        // does not add a cycle of delay
        fall_through_register #(
            .T          ( write_meta_channel_t        )
        ) i_aw_fall_through_register (
            .clk_i      ( clk_i             ),
            .rst_ni     ( rst_ni            ),
            .testmode_i ( testmode_i        ),
            .clr_i      ( 1'b0              ),
            .valid_i    ( w_valid           ),
            .ready_o    ( aw_ready          ),
            .data_i     ( w_req.aw_req      ),
            .valid_o    ( aw_valid_dp       ),
            .ready_i    ( aw_ready_dp       ),
            .data_o     ( aw_req_dp         )
        );

        // no unit: not busy
        assign busy_o.raw_coupler_busy = 1'b0;
    end


    //--------------------------------------
    // Assertions
    //--------------------------------------
    `IDMA_NONSYNTH_BLOCK(
    initial begin : proc_assert_params
        axi_addr_width : assert(AddrWidth >= 32'd12) else
            $fatal(1, "Parameter AddrWidth has to be >= 12!");
        axi_id_width   : assert(AxiIdWidth > 32'd0) else
            $fatal(1, "Parameter AxiIdWidth has to be > 0!");
        axi_data_width : assert(DataWidth inside {32'd16, 32'd32, 32'd64, 32'd128, 32'd256,
                                                  32'd512, 32'd1024}) else
            $fatal(1, "Parameter DataWidth has to be at least 16 and inside the AXI4 spec!");
        axi_user_width : assert(UserWidth > 32'd0) else
            $fatal(1, "Parameter UserWidth has to be > 0!");
        num_ax_in_flight : assert(NumAxInFlight > 32'd1) else
            $fatal(1, "Parameter NumAxInFlight has to be > 1!");
        buffer_depth : assert(BufferDepth > 32'd1) else
            $fatal(1, "Parameter BufferDepth has to be > 1!");
        tf_len_width : assert(TFLenWidth >= 32'd12) else
            $fatal(1, "Parameter BufferDepth has to be >= 12!");
        tf_len_width_max : assert(TFLenWidth <= AddrWidth) else
            $fatal(1, "Parameter TFLenWidth has to be <= AddrWidth!");
    end
    )

endmodule

// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Thomas Benz <tbenz@iis.ee.ethz.ch>
// - Tobias Senti <tsenti@ethz.ch>

`include "axi/typedef.svh"
`include "axi_stream/typedef.svh"
`include "idma/typedef.svh"
`include "obi/typedef.svh"
`include "tilelink/typedef.svh"

/// Synthesis wrapper for the iDMA backend. Unpacks all the interfaces to simple logic vectors
module idma_backend_synth_rw_axi #(
    /// Data width
    parameter int unsigned DataWidth           = 32'd32,
    /// Address width
    parameter int unsigned AddrWidth           = 32'd32,
    /// AXI user width
    parameter int unsigned UserWidth           = 32'd1,
    /// AXI ID width
    parameter int unsigned AxiIdWidth          = 32'd1,
    /// Number of transaction that can be in-flight concurrently
    parameter int unsigned NumAxInFlight       = 32'd3,
    /// The depth of the internal reorder buffer:
    /// - '2': minimal possible configuration
    /// - '3': efficiently handle misaligned transfers (recommended)
    parameter int unsigned BufferDepth         = 32'd3,
    /// With of a transfer: max transfer size is `2**TFLenWidth` bytes
    parameter int unsigned TFLenWidth          = 32'd32,
    /// The depth of the memory system the backend is attached to
    parameter int unsigned MemSysDepth         = 32'd0,
    /// Burst Len (for actual burst length do 8 byte * 2^(BurstLen))
    parameter int unsigned BurstLen = 4'd8,
    /// Should both data shifts be done before the dataflow element?
    /// If this is enabled, then the data inserted into the dataflow element
    /// will no longer be word aligned, but only a single shifter is needed
    parameter bit          CombinedShifter     = 1'b0,
    /// Mask invalid data on the manager interface
    parameter bit          MaskInvalidData     = 1'b1,
    /// Should the `R`-`AW` coupling hardware be present? (recommended)
    parameter bit          RAWCouplingAvail    = 1,
    /// Should hardware legalization be present? (recommended)
    /// If not, software legalization is required to ensure the transfers are
    /// AXI4-conformal
    parameter bit          HardwareLegalizer   = 1'b1,
    /// Reject zero-length transfers
    parameter bit          RejectZeroTransfers = 1'b1,
    /// Should the error handler be present?
    parameter bit          ErrorHandling       = 1'b1,
    // Dependent parameters; do not override!
    /// Strobe Width (do not override!)
    parameter int unsigned StrbWidth           = DataWidth / 8,
    /// Offset Width (do not override!)
    parameter int unsigned OffsetWidth         = $clog2(StrbWidth),
    /// Address type (do not override!)
    parameter type addr_t                      = logic[AddrWidth-1:0],
    /// Data type (do not override!)
    parameter type data_t                      = logic[DataWidth-1:0],
    /// Strobe type (do not override!)
    parameter type strb_t                      = logic[StrbWidth-1:0],
    /// User type (do not override!)
    parameter type user_t                      = logic[UserWidth-1:0],
    /// ID type (do not override!)
    parameter type id_t                        = logic[AxiIdWidth-1:0],
    /// Transfer length type (do not override!)
    parameter type tf_len_t                    = logic[TFLenWidth-1:0],
    /// Offset type (do not override!)
    parameter type offset_t                    = logic[OffsetWidth-1:0]
)(
    input  logic                   clk_i,
    input  logic                   rst_ni,
    input  logic                   test_i,

    input  logic                   req_valid_i,
    output logic                   req_ready_o,

    input  tf_len_t                req_length_i,
    input  addr_t                  req_src_addr_i,
    input  addr_t                  req_dst_addr_i,
    input  idma_pkg::protocol_e    req_src_protocol_i,
    input  idma_pkg::protocol_e    req_dst_protocol_i,
    input  id_t                    req_axi_id_i,
    input  axi_pkg::burst_t        req_src_burst_i,
    input  axi_pkg::cache_t        req_src_cache_i,
    input  logic                   req_src_lock_i,
    input  axi_pkg::prot_t         req_src_prot_i,
    input  axi_pkg::qos_t          req_src_qos_i,
    input  axi_pkg::region_t       req_src_region_i,
    input  axi_pkg::burst_t        req_dst_burst_i,
    input  axi_pkg::cache_t        req_dst_cache_i,
    input  logic                   req_dst_lock_i,
    input  axi_pkg::prot_t         req_dst_prot_i,
    input  axi_pkg::qos_t          req_dst_qos_i,
    input  axi_pkg::region_t       req_dst_region_i,
    input  logic                   req_decouple_aw_i,
    input  logic                   req_decouple_rw_i,
    input  logic [2:0]             req_src_max_llen_i,
    input  logic [2:0]             req_dst_max_llen_i,
    input  logic                   req_src_reduce_len_i,
    input  logic                   req_dst_reduce_len_i,
    input  logic                   req_last_i,

    output logic                   rsp_valid_o,
    input  logic                   rsp_ready_i,

    output axi_pkg::resp_t         rsp_cause_o,
    output idma_pkg::err_type_t    rsp_err_type_o,
    output addr_t                  rsp_burst_addr_o,
    output logic                   rsp_error_o,
    output logic                   rsp_last_o,

    input  logic                   eh_req_valid_i,
    output logic                   eh_req_ready_o,
    input  idma_pkg::idma_eh_req_t eh_req_i,

    output id_t                    axi_ar_id_o,
    output addr_t                  axi_ar_addr_o,
    output axi_pkg::len_t          axi_ar_len_o,
    output axi_pkg::size_t         axi_ar_size_o,
    output axi_pkg::burst_t        axi_ar_burst_o,
    output logic                   axi_ar_lock_o,
    output axi_pkg::cache_t        axi_ar_cache_o,
    output axi_pkg::prot_t         axi_ar_prot_o,
    output axi_pkg::qos_t          axi_ar_qos_o,
    output axi_pkg::region_t       axi_ar_region_o,
    output user_t                  axi_ar_user_o,
    output logic                   axi_ar_valid_o,
    input  logic                   axi_ar_ready_i,
    input  id_t                    axi_r_id_i,
    input  data_t                  axi_r_data_i,
    input  axi_pkg::resp_t         axi_r_resp_i,
    input  logic                   axi_r_last_i,
    input  user_t                  axi_r_user_i,
    input  logic                   axi_r_valid_i,
    output logic                   axi_r_ready_o,
    

    output id_t                    axi_aw_id_o,
    output addr_t                  axi_aw_addr_o,
    output axi_pkg::len_t          axi_aw_len_o,
    output axi_pkg::size_t         axi_aw_size_o,
    output axi_pkg::burst_t        axi_aw_burst_o,
    output logic                   axi_aw_lock_o,
    output axi_pkg::cache_t        axi_aw_cache_o,
    output axi_pkg::prot_t         axi_aw_prot_o,
    output axi_pkg::qos_t          axi_aw_qos_o,
    output axi_pkg::region_t       axi_aw_region_o,
    output axi_pkg::atop_t         axi_aw_atop_o,
    output user_t                  axi_aw_user_o,
    output logic                   axi_aw_valid_o,
    input  logic                   axi_aw_ready_i,
    output data_t                  axi_w_data_o,
    output strb_t                  axi_w_strb_o,
    output logic                   axi_w_last_o,
    output user_t                  axi_w_user_o,
    output logic                   axi_w_valid_o,
    input  logic                   axi_w_ready_i,
    input  id_t                    axi_b_id_i,
    input  axi_pkg::resp_t         axi_b_resp_i,
    input  user_t                  axi_b_user_i,
    input  logic                   axi_b_valid_i,
    output logic                   axi_b_ready_o,
    

    output idma_pkg::idma_busy_t   idma_busy_o
);

    /// Define the error handling capability
    localparam idma_pkg::error_cap_e ErrorCap = ErrorHandling ? idma_pkg::ERROR_HANDLING :
                                                                idma_pkg::NO_ERROR_HANDLING;

    // AXI4+ATOP typedefs
`AXI_TYPEDEF_AW_CHAN_T(axi_aw_chan_t, addr_t, id_t, user_t)
`AXI_TYPEDEF_W_CHAN_T(axi_w_chan_t, data_t, strb_t, user_t)
`AXI_TYPEDEF_B_CHAN_T(axi_b_chan_t, id_t, user_t)

`AXI_TYPEDEF_AR_CHAN_T(axi_ar_chan_t, addr_t, id_t, user_t)
`AXI_TYPEDEF_R_CHAN_T(axi_r_chan_t, data_t, id_t, user_t)

`AXI_TYPEDEF_REQ_T(axi_req_t, axi_aw_chan_t, axi_w_chan_t, axi_ar_chan_t)
`AXI_TYPEDEF_RESP_T(axi_rsp_t, axi_b_chan_t, axi_r_chan_t)


    // Meta Channel Widths
    localparam int unsigned axi_aw_chan_width = axi_pkg::aw_width(AddrWidth, AxiIdWidth, UserWidth);
    localparam int unsigned axi_ar_chan_width = axi_pkg::ar_width(AddrWidth, AxiIdWidth, UserWidth);

    /// Option struct: AXI4 id as well as AXI and backend options
    /// - `last`: a flag can be set if this transfer is the last of a set of transfers
    `IDMA_TYPEDEF_OPTIONS_T(options_t, id_t)

    /// 1D iDMA request type:
    /// - `length`: the length of the transfer in bytes
    /// - `*_addr`: the source / target byte addresses of the transfer
    /// - `opt`: the options field
    `IDMA_TYPEDEF_REQ_T(idma_req_t, tf_len_t, addr_t, options_t)

    /// 1D iDMA response payload:
    /// - `cause`: the AXI response
    /// - `err_type`: type of the error: read, write, internal, ...
    /// - `burst_addr`: the burst address where the issue error occurred
    `IDMA_TYPEDEF_ERR_PAYLOAD_T(err_payload_t, addr_t)

    /// 1D iDMA response type:
    /// - `last`: the response of the request that was marked with the `opt.last` flag
    /// - `error`: 1 if an error occurred
    /// - `pld`: the error payload
    `IDMA_TYPEDEF_RSP_T(idma_rsp_t, err_payload_t)


    typedef struct packed {
        axi_ar_chan_t ar_chan;
    } axi_read_meta_channel_t;

    typedef struct packed {
        axi_read_meta_channel_t axi;
    } read_meta_channel_t;

    typedef struct packed {
        axi_aw_chan_t aw_chan;
    } axi_write_meta_channel_t;

    typedef struct packed {
        axi_write_meta_channel_t axi;
    } write_meta_channel_t;

    // local types
    // AXI4+ATOP request and response
    axi_req_t axi_read_req;
    axi_rsp_t axi_read_rsp;

    axi_req_t axi_write_req;
    axi_rsp_t axi_write_rsp;

    idma_req_t idma_req;
    idma_rsp_t idma_rsp;

    idma_backend_rw_axi #(
        .CombinedShifter      ( CombinedShifter         ),
        .DataWidth            ( DataWidth               ),
        .AddrWidth            ( AddrWidth               ),
        .AxiIdWidth           ( AxiIdWidth              ),
        .UserWidth            ( UserWidth               ),
        .TFLenWidth           ( TFLenWidth              ),
        .MaskInvalidData      ( MaskInvalidData         ),
        .BufferDepth          ( BufferDepth             ),
        .NumAxInFlight        ( NumAxInFlight           ),
        .MemSysDepth          ( MemSysDepth             ),
        .RAWCouplingAvail     ( RAWCouplingAvail        ),
        .HardwareLegalizer    ( HardwareLegalizer       ),
        .RejectZeroTransfers  ( RejectZeroTransfers     ),
        .ErrorCap             ( ErrorCap                ),
        .idma_req_t           ( idma_req_t              ),
        .idma_rsp_t           ( idma_rsp_t              ),
        .idma_eh_req_t        ( idma_pkg::idma_eh_req_t ),
        .idma_busy_t          ( idma_pkg::idma_busy_t   ),
        .axi_req_t ( axi_req_t ),
        .axi_rsp_t ( axi_rsp_t ),
        .BurstLen             ( BurstLen                ),
        .write_meta_channel_t ( write_meta_channel_t    ),
        .read_meta_channel_t  ( read_meta_channel_t     )
    ) i_idma_backend (
        .clk_i                ( clk_i          ),
        .rst_ni               ( rst_ni         ),
        .testmode_i           ( test_i         ),
        .idma_req_i           ( idma_req       ),
        .req_valid_i          ( req_valid_i    ),
        .req_ready_o          ( req_ready_o    ),
        .idma_rsp_o           ( idma_rsp       ),
        .rsp_valid_o          ( rsp_valid_o    ),
        .rsp_ready_i          ( rsp_ready_i    ),
        .idma_eh_req_i        ( eh_req_i       ),
        .eh_req_valid_i       ( eh_req_valid_i ),
        .eh_req_ready_o       ( eh_req_ready_o ),
        .axi_read_req_o       ( axi_read_req   ),
        .axi_read_rsp_i       ( axi_read_rsp   ),
        .axi_write_req_o      ( axi_write_req  ),
        .axi_write_rsp_i      ( axi_write_rsp  ),
        .busy_o               ( idma_busy_o    )
    );

    // flatten structs
    assign idma_req.dst_addr               = req_dst_addr_i;
    assign idma_req.src_addr               = req_src_addr_i;
    assign idma_req.length                 = req_length_i;
    assign idma_req.opt.src_protocol       = req_src_protocol_i;
    assign idma_req.opt.dst_protocol       = req_dst_protocol_i;
    assign idma_req.opt.src_head           = '0;
    assign idma_req.opt.dst_head           = '0;
    assign idma_req.opt.axi_id             = req_axi_id_i;
    assign idma_req.opt.dst.cache          = req_dst_cache_i;
    assign idma_req.opt.dst.burst          = req_dst_burst_i;
    assign idma_req.opt.dst.qos            = req_dst_qos_i;
    assign idma_req.opt.dst.lock           = req_dst_lock_i;
    assign idma_req.opt.dst.prot           = req_dst_prot_i;
    assign idma_req.opt.dst.region         = req_dst_region_i;
    assign idma_req.opt.src.cache          = req_src_cache_i;
    assign idma_req.opt.src.burst          = req_src_burst_i;
    assign idma_req.opt.src.qos            = req_src_qos_i;
    assign idma_req.opt.src.lock           = req_src_lock_i;
    assign idma_req.opt.src.prot           = req_src_prot_i;
    assign idma_req.opt.src.region         = req_src_region_i;
    assign idma_req.opt.beo.dst_reduce_len = req_dst_reduce_len_i;
    assign idma_req.opt.beo.src_reduce_len = req_src_reduce_len_i;
    assign idma_req.opt.beo.dst_max_llen   = req_dst_max_llen_i;
    assign idma_req.opt.beo.src_max_llen   = req_src_max_llen_i;
    assign idma_req.opt.beo.decouple_rw    = req_decouple_rw_i;
    assign idma_req.opt.beo.decouple_aw    = req_decouple_aw_i;
    assign idma_req.opt.last               = req_last_i;

    assign rsp_cause_o      = idma_rsp.pld.cause;
    assign rsp_err_type_o   = idma_rsp.pld.err_type;
    assign rsp_burst_addr_o = idma_rsp.pld.burst_addr;
    assign rsp_error_o      = idma_rsp.error;
    assign rsp_last_o       = idma_rsp.last;


    // AXI4+ATOP Read
    assign axi_ar_id_o     = axi_read_req.ar.id;
    assign axi_ar_addr_o   = axi_read_req.ar.addr;
    assign axi_ar_len_o    = axi_read_req.ar.len;
    assign axi_ar_size_o   = axi_read_req.ar.size;
    assign axi_ar_burst_o  = axi_read_req.ar.burst;
    assign axi_ar_lock_o   = axi_read_req.ar.lock;
    assign axi_ar_cache_o  = axi_read_req.ar.cache;
    assign axi_ar_prot_o   = axi_read_req.ar.prot;
    assign axi_ar_qos_o    = axi_read_req.ar.qos;
    assign axi_ar_region_o = axi_read_req.ar.region;
    assign axi_ar_user_o   = axi_read_req.ar.user;
    assign axi_ar_valid_o  = axi_read_req.ar_valid;
    assign axi_r_ready_o   = axi_read_req.r_ready;
    
    assign axi_read_rsp.ar_ready = axi_ar_ready_i;
    assign axi_read_rsp.r.id     = axi_r_id_i;
    assign axi_read_rsp.r.data   = axi_r_data_i;
    assign axi_read_rsp.r.resp   = axi_r_resp_i;
    assign axi_read_rsp.r.last   = axi_r_last_i;
    assign axi_read_rsp.r.user   = axi_r_user_i;
    assign axi_read_rsp.r_valid  = axi_r_valid_i;
    


    // AXI4+ATOP Write
    assign axi_aw_id_o     = axi_write_req.aw.id;
    assign axi_aw_addr_o   = axi_write_req.aw.addr;
    assign axi_aw_len_o    = axi_write_req.aw.len;
    assign axi_aw_size_o   = axi_write_req.aw.size;
    assign axi_aw_burst_o  = axi_write_req.aw.burst;
    assign axi_aw_lock_o   = axi_write_req.aw.lock;
    assign axi_aw_cache_o  = axi_write_req.aw.cache;
    assign axi_aw_prot_o   = axi_write_req.aw.prot;
    assign axi_aw_qos_o    = axi_write_req.aw.qos;
    assign axi_aw_region_o = axi_write_req.aw.region;
    assign axi_aw_atop_o   = axi_write_req.aw.atop;
    assign axi_aw_user_o   = axi_write_req.aw.user;
    assign axi_aw_valid_o  = axi_write_req.aw_valid;
    assign axi_w_data_o    = axi_write_req.w.data;
    assign axi_w_strb_o    = axi_write_req.w.strb;
    assign axi_w_last_o    = axi_write_req.w.last;
    assign axi_w_user_o    = axi_write_req.w.user;
    assign axi_w_valid_o   = axi_write_req.w_valid;
    assign axi_b_ready_o   = axi_write_req.b_ready;
    
    assign axi_write_rsp.aw_ready = axi_aw_ready_i;
    assign axi_write_rsp.w_ready  = axi_w_ready_i;
    assign axi_write_rsp.b.id     = axi_b_id_i;
    assign axi_write_rsp.b.resp   = axi_b_resp_i;
    assign axi_write_rsp.b.user   = axi_b_user_i;
    assign axi_write_rsp.b_valid  = axi_b_valid_i;
    


endmodule

// Generated by PeakRDL-regblock - A free and open-source SystemVerilog generator
//  https://github.com/SystemRDL/PeakRDL-regblock

package idma_desc64_reg_pkg;

    localparam IDMA_DESC64_REG_TOP_DATA_WIDTH = 64;
    localparam IDMA_DESC64_REG_TOP_MIN_ADDR_WIDTH = 4;
    localparam IDMA_DESC64_REG_TOP_SIZE = 'h10;

    typedef struct {
        logic next;
    } idma_desc64_reg__status__busy__in_t;

    typedef struct {
        logic next;
    } idma_desc64_reg__status__fifo_full__in_t;

    typedef struct {
        idma_desc64_reg__status__busy__in_t busy;
        idma_desc64_reg__status__fifo_full__in_t fifo_full;
    } idma_desc64_reg__status__in_t;

    typedef struct {
        idma_desc64_reg__status__in_t status;
    } idma_desc64_reg__in_t;

    typedef struct {
        logic [63:0] value;
        logic swmod;
    } idma_desc64_reg__desc_addr__desc_addr__out_t;

    typedef struct {
        idma_desc64_reg__desc_addr__desc_addr__out_t desc_addr;
    } idma_desc64_reg__desc_addr__out_t;

    typedef struct {
        idma_desc64_reg__desc_addr__out_t desc_addr;
    } idma_desc64_reg__out_t;
endpackage
// Generated by PeakRDL-regblock - A free and open-source SystemVerilog generator
//  https://github.com/SystemRDL/PeakRDL-regblock

package idma_reg32_3d_reg_pkg;

    localparam IDMA_REG32_3D_REG_TOP_DATA_WIDTH = 32;
    localparam IDMA_REG32_3D_REG_TOP_MIN_ADDR_WIDTH = 8;
    localparam IDMA_REG32_3D_REG_TOP_SIZE = 'hf8;
    localparam SysAddrWidth = 'h20;
    localparam NumDims = 'h3;
    localparam Log2NumDims = 'h2;
    localparam NumProtBits = 'h3;

    typedef struct packed {
        logic [21:0] _reserved_31_10;
        logic [9:0] busy;
    } idma_reg_NumDims_3_Log2NumDims_2__status__external__fields__in_t;

    typedef struct {
        logic rd_ack;
        idma_reg_NumDims_3_Log2NumDims_2__status__external__fields__in_t rd_data;
    } idma_reg_NumDims_3_Log2NumDims_2__status__external__in_t;

    typedef struct packed {
        logic [31:0] next_id;
    } idma_reg_NumDims_3_Log2NumDims_2__next_id__external__fields__in_t;

    typedef struct {
        logic rd_ack;
        idma_reg_NumDims_3_Log2NumDims_2__next_id__external__fields__in_t rd_data;
    } idma_reg_NumDims_3_Log2NumDims_2__next_id__external__in_t;

    typedef struct packed {
        logic [31:0] done_id;
    } idma_reg_NumDims_3_Log2NumDims_2__done_id__external__fields__in_t;

    typedef struct {
        logic rd_ack;
        idma_reg_NumDims_3_Log2NumDims_2__done_id__external__fields__in_t rd_data;
    } idma_reg_NumDims_3_Log2NumDims_2__done_id__external__in_t;

    typedef struct {
        idma_reg_NumDims_3_Log2NumDims_2__status__external__in_t status[16];
        idma_reg_NumDims_3_Log2NumDims_2__next_id__external__in_t next_id[16];
        idma_reg_NumDims_3_Log2NumDims_2__done_id__external__in_t done_id[16];
    } idma_reg__in_t;

    typedef struct {
        logic value;
    } idma_reg_NumDims_3_Log2NumDims_2__conf__decouple_aw__out_t;

    typedef struct {
        logic value;
    } idma_reg_NumDims_3_Log2NumDims_2__conf__decouple_rw__out_t;

    typedef struct {
        logic value;
    } idma_reg_NumDims_3_Log2NumDims_2__conf__src_reduce_len__out_t;

    typedef struct {
        logic value;
    } idma_reg_NumDims_3_Log2NumDims_2__conf__dst_reduce_len__out_t;

    typedef struct {
        logic [2:0] value;
    } idma_reg_NumDims_3_Log2NumDims_2__conf__src_max_llen__out_t;

    typedef struct {
        logic [2:0] value;
    } idma_reg_NumDims_3_Log2NumDims_2__conf__dst_max_llen__out_t;

    typedef struct {
        logic [2:0] value;
    } idma_reg_NumDims_3_Log2NumDims_2__conf__enable_nd__out_t;

    typedef struct {
        logic [2:0] value;
    } idma_reg_NumDims_3_Log2NumDims_2__conf__src_protocol__out_t;

    typedef struct {
        logic [2:0] value;
    } idma_reg_NumDims_3_Log2NumDims_2__conf__dst_protocol__out_t;

    typedef struct {
        idma_reg_NumDims_3_Log2NumDims_2__conf__decouple_aw__out_t decouple_aw;
        idma_reg_NumDims_3_Log2NumDims_2__conf__decouple_rw__out_t decouple_rw;
        idma_reg_NumDims_3_Log2NumDims_2__conf__src_reduce_len__out_t src_reduce_len;
        idma_reg_NumDims_3_Log2NumDims_2__conf__dst_reduce_len__out_t dst_reduce_len;
        idma_reg_NumDims_3_Log2NumDims_2__conf__src_max_llen__out_t src_max_llen;
        idma_reg_NumDims_3_Log2NumDims_2__conf__dst_max_llen__out_t dst_max_llen;
        idma_reg_NumDims_3_Log2NumDims_2__conf__enable_nd__out_t enable_nd;
        idma_reg_NumDims_3_Log2NumDims_2__conf__src_protocol__out_t src_protocol;
        idma_reg_NumDims_3_Log2NumDims_2__conf__dst_protocol__out_t dst_protocol;
    } idma_reg_NumDims_3_Log2NumDims_2__conf__out_t;

    typedef struct {
        logic req;
        logic req_is_wr;
    } idma_reg_NumDims_3_Log2NumDims_2__status__external__out_t;

    typedef struct {
        logic req;
        logic req_is_wr;
    } idma_reg_NumDims_3_Log2NumDims_2__next_id__external__out_t;

    typedef struct {
        logic req;
        logic req_is_wr;
    } idma_reg_NumDims_3_Log2NumDims_2__done_id__external__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_NumDims_3_Log2NumDims_2__dst_addr__dst_addr__out_t;

    typedef struct {
        idma_reg_NumDims_3_Log2NumDims_2__dst_addr__dst_addr__out_t dst_addr;
    } idma_reg_NumDims_3_Log2NumDims_2__dst_addr__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_NumDims_3_Log2NumDims_2__src_addr__src_addr__out_t;

    typedef struct {
        idma_reg_NumDims_3_Log2NumDims_2__src_addr__src_addr__out_t src_addr;
    } idma_reg_NumDims_3_Log2NumDims_2__src_addr__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_NumDims_3_Log2NumDims_2__length__length__out_t;

    typedef struct {
        idma_reg_NumDims_3_Log2NumDims_2__length__length__out_t length;
    } idma_reg_NumDims_3_Log2NumDims_2__length__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_NumDims_3_Log2NumDims_2__dst_stride__dst_stride__out_t;

    typedef struct {
        idma_reg_NumDims_3_Log2NumDims_2__dst_stride__dst_stride__out_t dst_stride;
    } idma_reg_NumDims_3_Log2NumDims_2__dst_stride__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_NumDims_3_Log2NumDims_2__src_stride__src_stride__out_t;

    typedef struct {
        idma_reg_NumDims_3_Log2NumDims_2__src_stride__src_stride__out_t src_stride;
    } idma_reg_NumDims_3_Log2NumDims_2__src_stride__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_NumDims_3_Log2NumDims_2__reps__reps__out_t;

    typedef struct {
        idma_reg_NumDims_3_Log2NumDims_2__reps__reps__out_t reps;
    } idma_reg_NumDims_3_Log2NumDims_2__reps__out_t;

    typedef struct {
        idma_reg_NumDims_3_Log2NumDims_2__dst_stride__out_t dst_stride[1];
        idma_reg_NumDims_3_Log2NumDims_2__src_stride__out_t src_stride[1];
        idma_reg_NumDims_3_Log2NumDims_2__reps__out_t reps[1];
    } idma_reg_NumDims_3_Log2NumDims_2__dim__out_t;

    typedef struct {
        idma_reg_NumDims_3_Log2NumDims_2__conf__out_t conf;
        idma_reg_NumDims_3_Log2NumDims_2__status__external__out_t status[16];
        idma_reg_NumDims_3_Log2NumDims_2__next_id__external__out_t next_id[16];
        idma_reg_NumDims_3_Log2NumDims_2__done_id__external__out_t done_id[16];
        idma_reg_NumDims_3_Log2NumDims_2__dst_addr__out_t dst_addr[1];
        idma_reg_NumDims_3_Log2NumDims_2__src_addr__out_t src_addr[1];
        idma_reg_NumDims_3_Log2NumDims_2__length__out_t length[1];
        idma_reg_NumDims_3_Log2NumDims_2__dim__out_t dim[2];
    } idma_reg__out_t;
endpackage
// Generated by PeakRDL-regblock - A free and open-source SystemVerilog generator
//  https://github.com/SystemRDL/PeakRDL-regblock

package idma_reg64_2d_reg_pkg;

    localparam IDMA_REG64_2D_REG_TOP_DATA_WIDTH = 32;
    localparam IDMA_REG64_2D_REG_TOP_MIN_ADDR_WIDTH = 9;
    localparam IDMA_REG64_2D_REG_TOP_SIZE = 'h118;
    localparam SysAddrWidth = 'h40;
    localparam NumDims = 'h2;
    localparam Log2NumDims = 'h1;
    localparam NumProtBits = 'h3;

    typedef struct packed {
        logic [21:0] _reserved_31_10;
        logic [9:0] busy;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__status__external__fields__in_t;

    typedef struct {
        logic rd_ack;
        idma_reg_SysAddrWidth_40_Log2NumDims_1__status__external__fields__in_t rd_data;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__status__external__in_t;

    typedef struct packed {
        logic [31:0] next_id;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__next_id__external__fields__in_t;

    typedef struct {
        logic rd_ack;
        idma_reg_SysAddrWidth_40_Log2NumDims_1__next_id__external__fields__in_t rd_data;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__next_id__external__in_t;

    typedef struct packed {
        logic [31:0] done_id;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__done_id__external__fields__in_t;

    typedef struct {
        logic rd_ack;
        idma_reg_SysAddrWidth_40_Log2NumDims_1__done_id__external__fields__in_t rd_data;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__done_id__external__in_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_Log2NumDims_1__status__external__in_t status[16];
        idma_reg_SysAddrWidth_40_Log2NumDims_1__next_id__external__in_t next_id[16];
        idma_reg_SysAddrWidth_40_Log2NumDims_1__done_id__external__in_t done_id[16];
    } idma_reg__in_t;

    typedef struct {
        logic value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__decouple_aw__out_t;

    typedef struct {
        logic value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__decouple_rw__out_t;

    typedef struct {
        logic value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__src_reduce_len__out_t;

    typedef struct {
        logic value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__dst_reduce_len__out_t;

    typedef struct {
        logic [2:0] value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__src_max_llen__out_t;

    typedef struct {
        logic [2:0] value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__dst_max_llen__out_t;

    typedef struct {
        logic [1:0] value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__enable_nd__out_t;

    typedef struct {
        logic [2:0] value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__src_protocol__out_t;

    typedef struct {
        logic [2:0] value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__dst_protocol__out_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__decouple_aw__out_t decouple_aw;
        idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__decouple_rw__out_t decouple_rw;
        idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__src_reduce_len__out_t src_reduce_len;
        idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__dst_reduce_len__out_t dst_reduce_len;
        idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__src_max_llen__out_t src_max_llen;
        idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__dst_max_llen__out_t dst_max_llen;
        idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__enable_nd__out_t enable_nd;
        idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__src_protocol__out_t src_protocol;
        idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__dst_protocol__out_t dst_protocol;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__out_t;

    typedef struct {
        logic req;
        logic req_is_wr;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__status__external__out_t;

    typedef struct {
        logic req;
        logic req_is_wr;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__next_id__external__out_t;

    typedef struct {
        logic req;
        logic req_is_wr;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__done_id__external__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__dst_addr__dst_addr__out_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_Log2NumDims_1__dst_addr__dst_addr__out_t dst_addr;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__dst_addr__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__src_addr__src_addr__out_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_Log2NumDims_1__src_addr__src_addr__out_t src_addr;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__src_addr__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__length__length__out_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_Log2NumDims_1__length__length__out_t length;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__length__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__dst_stride__dst_stride__out_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_Log2NumDims_1__dst_stride__dst_stride__out_t dst_stride;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__dst_stride__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__src_stride__src_stride__out_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_Log2NumDims_1__src_stride__src_stride__out_t src_stride;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__src_stride__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__reps__reps__out_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_Log2NumDims_1__reps__reps__out_t reps;
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__reps__out_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_Log2NumDims_1__dst_stride__out_t dst_stride[2];
        idma_reg_SysAddrWidth_40_Log2NumDims_1__src_stride__out_t src_stride[2];
        idma_reg_SysAddrWidth_40_Log2NumDims_1__reps__out_t reps[2];
    } idma_reg_SysAddrWidth_40_Log2NumDims_1__dim__out_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_Log2NumDims_1__conf__out_t conf;
        idma_reg_SysAddrWidth_40_Log2NumDims_1__status__external__out_t status[16];
        idma_reg_SysAddrWidth_40_Log2NumDims_1__next_id__external__out_t next_id[16];
        idma_reg_SysAddrWidth_40_Log2NumDims_1__done_id__external__out_t done_id[16];
        idma_reg_SysAddrWidth_40_Log2NumDims_1__dst_addr__out_t dst_addr[2];
        idma_reg_SysAddrWidth_40_Log2NumDims_1__src_addr__out_t src_addr[2];
        idma_reg_SysAddrWidth_40_Log2NumDims_1__length__out_t length[2];
        idma_reg_SysAddrWidth_40_Log2NumDims_1__dim__out_t dim[1];
    } idma_reg__out_t;
endpackage
// Generated by PeakRDL-regblock - A free and open-source SystemVerilog generator
//  https://github.com/SystemRDL/PeakRDL-regblock

package idma_reg64_1d_reg_pkg;

    localparam IDMA_REG64_1D_REG_TOP_DATA_WIDTH = 32;
    localparam IDMA_REG64_1D_REG_TOP_MIN_ADDR_WIDTH = 9;
    localparam IDMA_REG64_1D_REG_TOP_SIZE = 'h118;
    localparam SysAddrWidth = 'h40;
    localparam NumDims = 'h1;
    localparam Log2NumDims = 'h1;
    localparam NumProtBits = 'h3;

    typedef struct packed {
        logic [21:0] _reserved_31_10;
        logic [9:0] busy;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__status__external__fields__in_t;

    typedef struct {
        logic rd_ack;
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__status__external__fields__in_t rd_data;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__status__external__in_t;

    typedef struct packed {
        logic [31:0] next_id;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__next_id__external__fields__in_t;

    typedef struct {
        logic rd_ack;
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__next_id__external__fields__in_t rd_data;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__next_id__external__in_t;

    typedef struct packed {
        logic [31:0] done_id;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__done_id__external__fields__in_t;

    typedef struct {
        logic rd_ack;
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__done_id__external__fields__in_t rd_data;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__done_id__external__in_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__status__external__in_t status[16];
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__next_id__external__in_t next_id[16];
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__done_id__external__in_t done_id[16];
    } idma_reg__in_t;

    typedef struct {
        logic value;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__decouple_aw__out_t;

    typedef struct {
        logic value;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__decouple_rw__out_t;

    typedef struct {
        logic value;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__src_reduce_len__out_t;

    typedef struct {
        logic value;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__dst_reduce_len__out_t;

    typedef struct {
        logic [2:0] value;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__src_max_llen__out_t;

    typedef struct {
        logic [2:0] value;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__dst_max_llen__out_t;

    typedef struct {
        logic [1:0] value;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__enable_nd__out_t;

    typedef struct {
        logic [2:0] value;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__src_protocol__out_t;

    typedef struct {
        logic [2:0] value;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__dst_protocol__out_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__decouple_aw__out_t decouple_aw;
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__decouple_rw__out_t decouple_rw;
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__src_reduce_len__out_t src_reduce_len;
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__dst_reduce_len__out_t dst_reduce_len;
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__src_max_llen__out_t src_max_llen;
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__dst_max_llen__out_t dst_max_llen;
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__enable_nd__out_t enable_nd;
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__src_protocol__out_t src_protocol;
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__dst_protocol__out_t dst_protocol;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__out_t;

    typedef struct {
        logic req;
        logic req_is_wr;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__status__external__out_t;

    typedef struct {
        logic req;
        logic req_is_wr;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__next_id__external__out_t;

    typedef struct {
        logic req;
        logic req_is_wr;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__done_id__external__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__dst_addr__dst_addr__out_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__dst_addr__dst_addr__out_t dst_addr;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__dst_addr__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__src_addr__src_addr__out_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__src_addr__src_addr__out_t src_addr;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__src_addr__out_t;

    typedef struct {
        logic [31:0] value;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__length__length__out_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__length__length__out_t length;
    } idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__length__out_t;

    typedef struct {
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__conf__out_t conf;
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__status__external__out_t status[16];
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__next_id__external__out_t next_id[16];
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__done_id__external__out_t done_id[16];
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__dst_addr__out_t dst_addr[2];
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__src_addr__out_t src_addr[2];
        idma_reg_SysAddrWidth_40_NumDims_1_Log2NumDims_1__length__out_t length[2];
    } idma_reg__out_t;
endpackage
// Generated by PeakRDL-regblock - A free and open-source SystemVerilog generator
//  https://github.com/SystemRDL/PeakRDL-regblock

module idma_desc64_reg_top (
        input wire clk,
        input wire arst_n,

        input wire s_apb_psel,
        input wire s_apb_penable,
        input wire s_apb_pwrite,
        input wire [2:0] s_apb_pprot,
        input wire [3:0] s_apb_paddr,
        input wire [63:0] s_apb_pwdata,
        input wire [7:0] s_apb_pstrb,
        output logic s_apb_pready,
        output logic [63:0] s_apb_prdata,
        output logic s_apb_pslverr,

        input idma_desc64_reg_pkg::idma_desc64_reg__in_t hwif_in,
        output idma_desc64_reg_pkg::idma_desc64_reg__out_t hwif_out
    );

    //--------------------------------------------------------------------------
    // CPU Bus interface logic
    //--------------------------------------------------------------------------
    logic cpuif_req;
    logic cpuif_req_is_wr;
    logic [3:0] cpuif_addr;
    logic [63:0] cpuif_wr_data;
    logic [63:0] cpuif_wr_biten;
    logic cpuif_req_stall_wr;
    logic cpuif_req_stall_rd;

    logic cpuif_rd_ack;
    logic cpuif_rd_err;
    logic [63:0] cpuif_rd_data;

    logic cpuif_wr_ack;
    logic cpuif_wr_err;

    // Request
    logic is_active;
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            is_active <= '0;
            cpuif_req <= '0;
            cpuif_req_is_wr <= '0;
            cpuif_addr <= '0;
            cpuif_wr_data <= '0;
            cpuif_wr_biten <= '0;
        end else begin
            if(~is_active) begin
                if(s_apb_psel) begin
                    is_active <= '1;
                    cpuif_req <= '1;
                    cpuif_req_is_wr <= s_apb_pwrite;
                    cpuif_addr <= {s_apb_paddr[3:3], 3'b0};
                    cpuif_wr_data <= s_apb_pwdata;
                    for(int i=0; i<8; i++) begin
                        cpuif_wr_biten[i*8 +: 8] <= {8{s_apb_pstrb[i]}};
                    end
                end
            end else begin
                cpuif_req <= '0;
                if(cpuif_rd_ack || cpuif_wr_ack) begin
                    is_active <= '0;
                end
            end
        end
    end

    // Response
    assign s_apb_pready = cpuif_rd_ack | cpuif_wr_ack;
    assign s_apb_prdata = cpuif_rd_data;
    assign s_apb_pslverr = cpuif_rd_err | cpuif_wr_err;

    logic cpuif_req_masked;

    // Read & write latencies are balanced. Stalls not required
    assign cpuif_req_stall_rd = '0;
    assign cpuif_req_stall_wr = '0;
    assign cpuif_req_masked = cpuif_req
                            & !(!cpuif_req_is_wr & cpuif_req_stall_rd)
                            & !(cpuif_req_is_wr & cpuif_req_stall_wr);

    //--------------------------------------------------------------------------
    // Address Decode
    //--------------------------------------------------------------------------
    typedef struct {
        logic desc_addr;
        logic status;
    } decoded_reg_strb_t;
    decoded_reg_strb_t decoded_reg_strb;
    logic decoded_err;
    logic [3:0] decoded_addr;
    logic decoded_req;
    logic decoded_req_is_wr;
    logic [63:0] decoded_wr_data;
    logic [63:0] decoded_wr_biten;

    always_comb begin
        automatic logic is_valid_addr;
        automatic logic is_valid_rw;
        is_valid_addr = '1; // No valid address check
        is_valid_rw = '1; // No valid RW check
        decoded_reg_strb.desc_addr = cpuif_req_masked & (cpuif_addr == 4'h0) & cpuif_req_is_wr;
        decoded_reg_strb.status = cpuif_req_masked & (cpuif_addr == 4'h8) & !cpuif_req_is_wr;
        decoded_err = '0;
    end

    // Pass down signals to next stage
    assign decoded_addr = cpuif_addr;
    assign decoded_req = cpuif_req_masked;
    assign decoded_req_is_wr = cpuif_req_is_wr;
    assign decoded_wr_data = cpuif_wr_data;
    assign decoded_wr_biten = cpuif_wr_biten;

    //--------------------------------------------------------------------------
    // Field logic
    //--------------------------------------------------------------------------
    typedef struct {
        struct {
            struct {
                logic [63:0] next;
                logic load_next;
            } desc_addr;
        } desc_addr;
    } field_combo_t;
    field_combo_t field_combo;

    typedef struct {
        struct {
            struct {
                logic [63:0] value;
            } desc_addr;
        } desc_addr;
    } field_storage_t;
    field_storage_t field_storage;

    // Field: idma_desc64_reg.desc_addr.desc_addr
    always_comb begin
        automatic logic [63:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.desc_addr.desc_addr.value;
        load_next_c = '0;
        if(decoded_reg_strb.desc_addr && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.desc_addr.desc_addr.value & ~decoded_wr_biten[63:0]) | (decoded_wr_data[63:0] & decoded_wr_biten[63:0]);
            load_next_c = '1;
        end
        field_combo.desc_addr.desc_addr.next = next_c;
        field_combo.desc_addr.desc_addr.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.desc_addr.desc_addr.value <= 64'hffffffffffffffff;
        end else begin
            if(field_combo.desc_addr.desc_addr.load_next) begin
                field_storage.desc_addr.desc_addr.value <= field_combo.desc_addr.desc_addr.next;
            end
        end
    end
    assign hwif_out.desc_addr.desc_addr.value = field_storage.desc_addr.desc_addr.value;
    assign hwif_out.desc_addr.desc_addr.swmod = decoded_reg_strb.desc_addr && decoded_req_is_wr && |(decoded_wr_biten[63:0]);

    //--------------------------------------------------------------------------
    // Write response
    //--------------------------------------------------------------------------
    assign cpuif_wr_ack = decoded_req & decoded_req_is_wr;
    // Writes are always granted with no error response
    assign cpuif_wr_err = '0;

    //--------------------------------------------------------------------------
    // Readback
    //--------------------------------------------------------------------------

    logic [3:0] rd_mux_addr;
    assign rd_mux_addr = decoded_addr;

    logic readback_err;
    logic readback_done;
    logic [63:0] readback_data;
    always_comb begin
        automatic logic [63:0] readback_data_var;
        readback_data_var = '0;
        if(rd_mux_addr == 4'h8) begin
            readback_data_var[0] = hwif_in.status.busy.next;
            readback_data_var[1] = hwif_in.status.fifo_full.next;
        end
        readback_data = readback_data_var;
        readback_done = decoded_req & ~decoded_req_is_wr;
        readback_err = '0;
    end

    assign cpuif_rd_ack = readback_done;
    assign cpuif_rd_data = readback_data;
    assign cpuif_rd_err = readback_err;
endmodule
// Generated by PeakRDL-regblock - A free and open-source SystemVerilog generator
//  https://github.com/SystemRDL/PeakRDL-regblock

module idma_reg32_3d_reg_top #(
        parameter ID_WIDTH = 1
    ) (
        input wire clk,
        input wire arst_n,

        input wire s_obi_req,
        output logic s_obi_gnt,
        input wire [7:0] s_obi_addr,
        input wire s_obi_we,
        input wire [3:0] s_obi_be,
        input wire [31:0] s_obi_wdata,
        input wire [ID_WIDTH-1:0] s_obi_aid,
        output logic s_obi_rvalid,
        input wire s_obi_rready,
        output logic [31:0] s_obi_rdata,
        output logic s_obi_err,
        output logic [ID_WIDTH-1:0] s_obi_rid,

        input idma_reg32_3d_reg_pkg::idma_reg__in_t hwif_in,
        output idma_reg32_3d_reg_pkg::idma_reg__out_t hwif_out
    );

    //--------------------------------------------------------------------------
    // CPU Bus interface logic
    //--------------------------------------------------------------------------
    logic cpuif_req;
    logic cpuif_req_is_wr;
    logic [7:0] cpuif_addr;
    logic [31:0] cpuif_wr_data;
    logic [31:0] cpuif_wr_biten;
    logic cpuif_req_stall_wr;
    logic cpuif_req_stall_rd;

    logic cpuif_rd_ack;
    logic cpuif_rd_err;
    logic [31:0] cpuif_rd_data;

    logic cpuif_wr_ack;
    logic cpuif_wr_err;

    // State & holding regs
    logic is_active; // A request is being served (not yet fully responded)
    logic gnt_q; // one-cycle grant for A-channel
    logic rsp_pending; // response ready but not yet accepted by manager
    logic [31:0] rsp_rdata_q;
    logic rsp_err_q;
    logic [$bits(s_obi_rid)-1:0] rid_q;

    // Latch AID on accept to echo back the response
    always_ff @(posedge clk or negedge arst_n) begin
        if (~arst_n) begin
            is_active <= 1'b0;
            gnt_q <= 1'b0;
            rsp_pending <= 1'b0;
            rsp_rdata_q <= '0;
            rsp_err_q <= 1'b0;
            rid_q <= '0;

            cpuif_req <= '0;
            cpuif_req_is_wr <= '0;
            cpuif_addr <= '0;
            cpuif_wr_data <= '0;
            cpuif_wr_biten <= '0;
        end else begin
            // defaults
            cpuif_req <= 1'b0;
            gnt_q <= s_obi_req & ~is_active;

            // Accept new request when idle
            if (~is_active) begin
                if (s_obi_req) begin
                    is_active <= 1'b1;
                    cpuif_req <= 1'b1;
                    cpuif_req_is_wr <= s_obi_we;
                    cpuif_addr <= {s_obi_addr[7:2], 2'b0};
                    cpuif_wr_data <= s_obi_wdata;
                    rid_q <= s_obi_aid;
                    for (int i = 0; i < 4; i++) begin
                        cpuif_wr_biten[i*8 +: 8] <= {8{ s_obi_be[i] }};
                    end
                end
            end

            // Capture response
            if (is_active && (cpuif_rd_ack || cpuif_wr_ack)) begin
                rsp_pending <= 1'b1;
                rsp_rdata_q <= cpuif_rd_data;
                rsp_err_q <= cpuif_rd_err | cpuif_wr_err;
                // NOTE: Keep 'is_active' asserted until the external R handshake completes
            end

            // Complete external R-channel handshake only if manager ready
            if (rsp_pending && s_obi_rvalid && s_obi_rready) begin
                rsp_pending <= 1'b0;
                is_active <= 1'b0; // free to accept the next request
            end
        end
    end

    // R-channel outputs (held stable while rsp_pending=1)
    assign s_obi_rvalid = rsp_pending;
    assign s_obi_rdata = rsp_rdata_q;
    assign s_obi_err = rsp_err_q;
    assign s_obi_rid = rid_q;

    // A-channel grant (registered one-cycle pulse when we accept a request)
    assign s_obi_gnt = gnt_q;

    logic cpuif_req_masked;
    logic external_pending;

    // Read & write latencies are balanced. Stalls not required
    // except if external
    assign cpuif_req_stall_rd = external_pending;
    assign cpuif_req_stall_wr = external_pending;
    assign cpuif_req_masked = cpuif_req
                            & !(!cpuif_req_is_wr & cpuif_req_stall_rd)
                            & !(cpuif_req_is_wr & cpuif_req_stall_wr);

    //--------------------------------------------------------------------------
    // Address Decode
    //--------------------------------------------------------------------------
    typedef struct {
        logic conf;
        logic status[16];
        logic next_id[16];
        logic done_id[16];
        logic dst_addr[1];
        logic src_addr[1];
        logic length[1];
        struct {
            logic dst_stride[1];
            logic src_stride[1];
            logic reps[1];
        } dim[2];
    } decoded_reg_strb_t;
    decoded_reg_strb_t decoded_reg_strb;
    logic decoded_err;
    logic decoded_req_is_external;

    logic [7:0] decoded_addr;
    logic decoded_req;
    logic decoded_req_is_wr;
    logic [31:0] decoded_wr_data;
    logic [31:0] decoded_wr_biten;

    always_comb begin
        automatic logic is_valid_addr;
        automatic logic is_valid_rw;
        automatic logic is_external;
        is_external = '0;
        is_valid_addr = '1; // No valid address check
        is_valid_rw = '1; // No valid RW check
        decoded_reg_strb.conf = cpuif_req_masked & (cpuif_addr == 8'h0);
        for(int i0=0; i0<16; i0++) begin
            decoded_reg_strb.status[i0] = cpuif_req_masked & (cpuif_addr == 8'h4 + (8)'(i0) * 8'h4) & !cpuif_req_is_wr;
            is_external |= cpuif_req_masked & (cpuif_addr == 8'h4 + (8)'(i0) * 8'h4) & !cpuif_req_is_wr;
        end
        for(int i0=0; i0<16; i0++) begin
            decoded_reg_strb.next_id[i0] = cpuif_req_masked & (cpuif_addr == 8'h44 + (8)'(i0) * 8'h4) & !cpuif_req_is_wr;
            is_external |= cpuif_req_masked & (cpuif_addr == 8'h44 + (8)'(i0) * 8'h4) & !cpuif_req_is_wr;
        end
        for(int i0=0; i0<16; i0++) begin
            decoded_reg_strb.done_id[i0] = cpuif_req_masked & (cpuif_addr == 8'h84 + (8)'(i0) * 8'h4) & !cpuif_req_is_wr;
            is_external |= cpuif_req_masked & (cpuif_addr == 8'h84 + (8)'(i0) * 8'h4) & !cpuif_req_is_wr;
        end
        for(int i0=0; i0<1; i0++) begin
            decoded_reg_strb.dst_addr[i0] = cpuif_req_masked & (cpuif_addr == 8'hd0 + (8)'(i0) * 8'h4);
        end
        for(int i0=0; i0<1; i0++) begin
            decoded_reg_strb.src_addr[i0] = cpuif_req_masked & (cpuif_addr == 8'hd4 + (8)'(i0) * 8'h4);
        end
        for(int i0=0; i0<1; i0++) begin
            decoded_reg_strb.length[i0] = cpuif_req_masked & (cpuif_addr == 8'hd8 + (8)'(i0) * 8'h4);
        end
        for(int i0=0; i0<2; i0++) begin
            for(int i1=0; i1<1; i1++) begin
                decoded_reg_strb.dim[i0].dst_stride[i1] = cpuif_req_masked & (cpuif_addr == 8'he0 + (8)'(i0) * 8'hc + (8)'(i1) * 8'h4);
            end
            for(int i1=0; i1<1; i1++) begin
                decoded_reg_strb.dim[i0].src_stride[i1] = cpuif_req_masked & (cpuif_addr == 8'he4 + (8)'(i0) * 8'hc + (8)'(i1) * 8'h4);
            end
            for(int i1=0; i1<1; i1++) begin
                decoded_reg_strb.dim[i0].reps[i1] = cpuif_req_masked & (cpuif_addr == 8'he8 + (8)'(i0) * 8'hc + (8)'(i1) * 8'h4);
            end
        end
        decoded_err = '0;
        decoded_req_is_external = is_external;
    end
    logic external_wr_ack;
    logic external_rd_ack;
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            external_pending <= '0;
        end else begin
            if(decoded_req_is_external & ~external_wr_ack & ~external_rd_ack) external_pending <= '1;
            else if(external_wr_ack | external_rd_ack) external_pending <= '0;
            `ifndef SYNTHESIS
                assert_bad_ext_wr_ack: assert(!external_wr_ack || (external_pending | decoded_req_is_external))
                    else $error("An external wr_ack strobe was asserted when no external request was active");
                assert_bad_ext_rd_ack: assert(!external_rd_ack || (external_pending | decoded_req_is_external))
                    else $error("An external rd_ack strobe was asserted when no external request was active");
            `endif
        end
    end

    // Pass down signals to next stage
    assign decoded_addr = cpuif_addr;
    assign decoded_req = cpuif_req_masked;
    assign decoded_req_is_wr = cpuif_req_is_wr;
    assign decoded_wr_data = cpuif_wr_data;
    assign decoded_wr_biten = cpuif_wr_biten;

    //--------------------------------------------------------------------------
    // Field logic
    //--------------------------------------------------------------------------
    typedef struct {
        struct {
            struct {
                logic next;
                logic load_next;
            } decouple_aw;
            struct {
                logic next;
                logic load_next;
            } decouple_rw;
            struct {
                logic next;
                logic load_next;
            } src_reduce_len;
            struct {
                logic next;
                logic load_next;
            } dst_reduce_len;
            struct {
                logic [2:0] next;
                logic load_next;
            } src_max_llen;
            struct {
                logic [2:0] next;
                logic load_next;
            } dst_max_llen;
            struct {
                logic [2:0] next;
                logic load_next;
            } enable_nd;
            struct {
                logic [2:0] next;
                logic load_next;
            } src_protocol;
            struct {
                logic [2:0] next;
                logic load_next;
            } dst_protocol;
        } conf;
        struct {
            struct {
                logic [31:0] next;
                logic load_next;
            } dst_addr;
        } dst_addr[1];
        struct {
            struct {
                logic [31:0] next;
                logic load_next;
            } src_addr;
        } src_addr[1];
        struct {
            struct {
                logic [31:0] next;
                logic load_next;
            } length;
        } length[1];
        struct {
            struct {
                struct {
                    logic [31:0] next;
                    logic load_next;
                } dst_stride;
            } dst_stride[1];
            struct {
                struct {
                    logic [31:0] next;
                    logic load_next;
                } src_stride;
            } src_stride[1];
            struct {
                struct {
                    logic [31:0] next;
                    logic load_next;
                } reps;
            } reps[1];
        } dim[2];
    } field_combo_t;
    field_combo_t field_combo;

    typedef struct {
        struct {
            struct {
                logic value;
            } decouple_aw;
            struct {
                logic value;
            } decouple_rw;
            struct {
                logic value;
            } src_reduce_len;
            struct {
                logic value;
            } dst_reduce_len;
            struct {
                logic [2:0] value;
            } src_max_llen;
            struct {
                logic [2:0] value;
            } dst_max_llen;
            struct {
                logic [2:0] value;
            } enable_nd;
            struct {
                logic [2:0] value;
            } src_protocol;
            struct {
                logic [2:0] value;
            } dst_protocol;
        } conf;
        struct {
            struct {
                logic [31:0] value;
            } dst_addr;
        } dst_addr[1];
        struct {
            struct {
                logic [31:0] value;
            } src_addr;
        } src_addr[1];
        struct {
            struct {
                logic [31:0] value;
            } length;
        } length[1];
        struct {
            struct {
                struct {
                    logic [31:0] value;
                } dst_stride;
            } dst_stride[1];
            struct {
                struct {
                    logic [31:0] value;
                } src_stride;
            } src_stride[1];
            struct {
                struct {
                    logic [31:0] value;
                } reps;
            } reps[1];
        } dim[2];
    } field_storage_t;
    field_storage_t field_storage;

    // Field: idma_reg.conf.decouple_aw
    always_comb begin
        automatic logic [0:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.decouple_aw.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.decouple_aw.value & ~decoded_wr_biten[0:0]) | (decoded_wr_data[0:0] & decoded_wr_biten[0:0]);
            load_next_c = '1;
        end
        field_combo.conf.decouple_aw.next = next_c;
        field_combo.conf.decouple_aw.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.decouple_aw.value <= 1'h0;
        end else begin
            if(field_combo.conf.decouple_aw.load_next) begin
                field_storage.conf.decouple_aw.value <= field_combo.conf.decouple_aw.next;
            end
        end
    end
    assign hwif_out.conf.decouple_aw.value = field_storage.conf.decouple_aw.value;
    // Field: idma_reg.conf.decouple_rw
    always_comb begin
        automatic logic [0:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.decouple_rw.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.decouple_rw.value & ~decoded_wr_biten[1:1]) | (decoded_wr_data[1:1] & decoded_wr_biten[1:1]);
            load_next_c = '1;
        end
        field_combo.conf.decouple_rw.next = next_c;
        field_combo.conf.decouple_rw.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.decouple_rw.value <= 1'h0;
        end else begin
            if(field_combo.conf.decouple_rw.load_next) begin
                field_storage.conf.decouple_rw.value <= field_combo.conf.decouple_rw.next;
            end
        end
    end
    assign hwif_out.conf.decouple_rw.value = field_storage.conf.decouple_rw.value;
    // Field: idma_reg.conf.src_reduce_len
    always_comb begin
        automatic logic [0:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.src_reduce_len.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.src_reduce_len.value & ~decoded_wr_biten[2:2]) | (decoded_wr_data[2:2] & decoded_wr_biten[2:2]);
            load_next_c = '1;
        end
        field_combo.conf.src_reduce_len.next = next_c;
        field_combo.conf.src_reduce_len.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.src_reduce_len.value <= 1'h0;
        end else begin
            if(field_combo.conf.src_reduce_len.load_next) begin
                field_storage.conf.src_reduce_len.value <= field_combo.conf.src_reduce_len.next;
            end
        end
    end
    assign hwif_out.conf.src_reduce_len.value = field_storage.conf.src_reduce_len.value;
    // Field: idma_reg.conf.dst_reduce_len
    always_comb begin
        automatic logic [0:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.dst_reduce_len.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.dst_reduce_len.value & ~decoded_wr_biten[3:3]) | (decoded_wr_data[3:3] & decoded_wr_biten[3:3]);
            load_next_c = '1;
        end
        field_combo.conf.dst_reduce_len.next = next_c;
        field_combo.conf.dst_reduce_len.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.dst_reduce_len.value <= 1'h0;
        end else begin
            if(field_combo.conf.dst_reduce_len.load_next) begin
                field_storage.conf.dst_reduce_len.value <= field_combo.conf.dst_reduce_len.next;
            end
        end
    end
    assign hwif_out.conf.dst_reduce_len.value = field_storage.conf.dst_reduce_len.value;
    // Field: idma_reg.conf.src_max_llen
    always_comb begin
        automatic logic [2:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.src_max_llen.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.src_max_llen.value & ~decoded_wr_biten[6:4]) | (decoded_wr_data[6:4] & decoded_wr_biten[6:4]);
            load_next_c = '1;
        end
        field_combo.conf.src_max_llen.next = next_c;
        field_combo.conf.src_max_llen.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.src_max_llen.value <= 3'h0;
        end else begin
            if(field_combo.conf.src_max_llen.load_next) begin
                field_storage.conf.src_max_llen.value <= field_combo.conf.src_max_llen.next;
            end
        end
    end
    assign hwif_out.conf.src_max_llen.value = field_storage.conf.src_max_llen.value;
    // Field: idma_reg.conf.dst_max_llen
    always_comb begin
        automatic logic [2:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.dst_max_llen.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.dst_max_llen.value & ~decoded_wr_biten[9:7]) | (decoded_wr_data[9:7] & decoded_wr_biten[9:7]);
            load_next_c = '1;
        end
        field_combo.conf.dst_max_llen.next = next_c;
        field_combo.conf.dst_max_llen.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.dst_max_llen.value <= 3'h0;
        end else begin
            if(field_combo.conf.dst_max_llen.load_next) begin
                field_storage.conf.dst_max_llen.value <= field_combo.conf.dst_max_llen.next;
            end
        end
    end
    assign hwif_out.conf.dst_max_llen.value = field_storage.conf.dst_max_llen.value;
    // Field: idma_reg.conf.enable_nd
    always_comb begin
        automatic logic [2:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.enable_nd.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.enable_nd.value & ~decoded_wr_biten[12:10]) | (decoded_wr_data[12:10] & decoded_wr_biten[12:10]);
            load_next_c = '1;
        end
        field_combo.conf.enable_nd.next = next_c;
        field_combo.conf.enable_nd.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.enable_nd.value <= 3'h0;
        end else begin
            if(field_combo.conf.enable_nd.load_next) begin
                field_storage.conf.enable_nd.value <= field_combo.conf.enable_nd.next;
            end
        end
    end
    assign hwif_out.conf.enable_nd.value = field_storage.conf.enable_nd.value;
    // Field: idma_reg.conf.src_protocol
    always_comb begin
        automatic logic [2:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.src_protocol.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.src_protocol.value & ~decoded_wr_biten[15:13]) | (decoded_wr_data[15:13] & decoded_wr_biten[15:13]);
            load_next_c = '1;
        end
        field_combo.conf.src_protocol.next = next_c;
        field_combo.conf.src_protocol.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.src_protocol.value <= 3'h0;
        end else begin
            if(field_combo.conf.src_protocol.load_next) begin
                field_storage.conf.src_protocol.value <= field_combo.conf.src_protocol.next;
            end
        end
    end
    assign hwif_out.conf.src_protocol.value = field_storage.conf.src_protocol.value;
    // Field: idma_reg.conf.dst_protocol
    always_comb begin
        automatic logic [2:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.dst_protocol.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.dst_protocol.value & ~decoded_wr_biten[18:16]) | (decoded_wr_data[18:16] & decoded_wr_biten[18:16]);
            load_next_c = '1;
        end
        field_combo.conf.dst_protocol.next = next_c;
        field_combo.conf.dst_protocol.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.dst_protocol.value <= 3'h0;
        end else begin
            if(field_combo.conf.dst_protocol.load_next) begin
                field_storage.conf.dst_protocol.value <= field_combo.conf.dst_protocol.next;
            end
        end
    end
    assign hwif_out.conf.dst_protocol.value = field_storage.conf.dst_protocol.value;
    for(genvar i0=0; i0<16; i0++) begin
        // External register: idma_reg.status[]

        assign hwif_out.status[i0].req = !decoded_req_is_wr ? decoded_reg_strb.status[i0] : '0;
        assign hwif_out.status[i0].req_is_wr = decoded_req_is_wr;
    end
    for(genvar i0=0; i0<16; i0++) begin
        // External register: idma_reg.next_id[]

        assign hwif_out.next_id[i0].req = !decoded_req_is_wr ? decoded_reg_strb.next_id[i0] : '0;
        assign hwif_out.next_id[i0].req_is_wr = decoded_req_is_wr;
    end
    for(genvar i0=0; i0<16; i0++) begin
        // External register: idma_reg.done_id[]

        assign hwif_out.done_id[i0].req = !decoded_req_is_wr ? decoded_reg_strb.done_id[i0] : '0;
        assign hwif_out.done_id[i0].req_is_wr = decoded_req_is_wr;
    end
    for(genvar i0=0; i0<1; i0++) begin
        // Field: idma_reg.dst_addr[].dst_addr
        always_comb begin
            automatic logic [31:0] next_c;
            automatic logic load_next_c;
            next_c = field_storage.dst_addr[i0].dst_addr.value;
            load_next_c = '0;
            if(decoded_reg_strb.dst_addr[i0] && decoded_req_is_wr) begin // SW write
                next_c = (field_storage.dst_addr[i0].dst_addr.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                load_next_c = '1;
            end
            field_combo.dst_addr[i0].dst_addr.next = next_c;
            field_combo.dst_addr[i0].dst_addr.load_next = load_next_c;
        end
        always_ff @(posedge clk or negedge arst_n) begin
            if(~arst_n) begin
                field_storage.dst_addr[i0].dst_addr.value <= 32'h0;
            end else begin
                if(field_combo.dst_addr[i0].dst_addr.load_next) begin
                    field_storage.dst_addr[i0].dst_addr.value <= field_combo.dst_addr[i0].dst_addr.next;
                end
            end
        end
        assign hwif_out.dst_addr[i0].dst_addr.value = field_storage.dst_addr[i0].dst_addr.value;
    end
    for(genvar i0=0; i0<1; i0++) begin
        // Field: idma_reg.src_addr[].src_addr
        always_comb begin
            automatic logic [31:0] next_c;
            automatic logic load_next_c;
            next_c = field_storage.src_addr[i0].src_addr.value;
            load_next_c = '0;
            if(decoded_reg_strb.src_addr[i0] && decoded_req_is_wr) begin // SW write
                next_c = (field_storage.src_addr[i0].src_addr.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                load_next_c = '1;
            end
            field_combo.src_addr[i0].src_addr.next = next_c;
            field_combo.src_addr[i0].src_addr.load_next = load_next_c;
        end
        always_ff @(posedge clk or negedge arst_n) begin
            if(~arst_n) begin
                field_storage.src_addr[i0].src_addr.value <= 32'h0;
            end else begin
                if(field_combo.src_addr[i0].src_addr.load_next) begin
                    field_storage.src_addr[i0].src_addr.value <= field_combo.src_addr[i0].src_addr.next;
                end
            end
        end
        assign hwif_out.src_addr[i0].src_addr.value = field_storage.src_addr[i0].src_addr.value;
    end
    for(genvar i0=0; i0<1; i0++) begin
        // Field: idma_reg.length[].length
        always_comb begin
            automatic logic [31:0] next_c;
            automatic logic load_next_c;
            next_c = field_storage.length[i0].length.value;
            load_next_c = '0;
            if(decoded_reg_strb.length[i0] && decoded_req_is_wr) begin // SW write
                next_c = (field_storage.length[i0].length.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                load_next_c = '1;
            end
            field_combo.length[i0].length.next = next_c;
            field_combo.length[i0].length.load_next = load_next_c;
        end
        always_ff @(posedge clk or negedge arst_n) begin
            if(~arst_n) begin
                field_storage.length[i0].length.value <= 32'h0;
            end else begin
                if(field_combo.length[i0].length.load_next) begin
                    field_storage.length[i0].length.value <= field_combo.length[i0].length.next;
                end
            end
        end
        assign hwif_out.length[i0].length.value = field_storage.length[i0].length.value;
    end
    for(genvar i0=0; i0<2; i0++) begin
        for(genvar i1=0; i1<1; i1++) begin
            // Field: idma_reg.dim[].dst_stride[].dst_stride
            always_comb begin
                automatic logic [31:0] next_c;
                automatic logic load_next_c;
                next_c = field_storage.dim[i0].dst_stride[i1].dst_stride.value;
                load_next_c = '0;
                if(decoded_reg_strb.dim[i0].dst_stride[i1] && decoded_req_is_wr) begin // SW write
                    next_c = (field_storage.dim[i0].dst_stride[i1].dst_stride.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                    load_next_c = '1;
                end
                field_combo.dim[i0].dst_stride[i1].dst_stride.next = next_c;
                field_combo.dim[i0].dst_stride[i1].dst_stride.load_next = load_next_c;
            end
            always_ff @(posedge clk or negedge arst_n) begin
                if(~arst_n) begin
                    field_storage.dim[i0].dst_stride[i1].dst_stride.value <= 32'h0;
                end else begin
                    if(field_combo.dim[i0].dst_stride[i1].dst_stride.load_next) begin
                        field_storage.dim[i0].dst_stride[i1].dst_stride.value <= field_combo.dim[i0].dst_stride[i1].dst_stride.next;
                    end
                end
            end
            assign hwif_out.dim[i0].dst_stride[i1].dst_stride.value = field_storage.dim[i0].dst_stride[i1].dst_stride.value;
        end
        for(genvar i1=0; i1<1; i1++) begin
            // Field: idma_reg.dim[].src_stride[].src_stride
            always_comb begin
                automatic logic [31:0] next_c;
                automatic logic load_next_c;
                next_c = field_storage.dim[i0].src_stride[i1].src_stride.value;
                load_next_c = '0;
                if(decoded_reg_strb.dim[i0].src_stride[i1] && decoded_req_is_wr) begin // SW write
                    next_c = (field_storage.dim[i0].src_stride[i1].src_stride.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                    load_next_c = '1;
                end
                field_combo.dim[i0].src_stride[i1].src_stride.next = next_c;
                field_combo.dim[i0].src_stride[i1].src_stride.load_next = load_next_c;
            end
            always_ff @(posedge clk or negedge arst_n) begin
                if(~arst_n) begin
                    field_storage.dim[i0].src_stride[i1].src_stride.value <= 32'h0;
                end else begin
                    if(field_combo.dim[i0].src_stride[i1].src_stride.load_next) begin
                        field_storage.dim[i0].src_stride[i1].src_stride.value <= field_combo.dim[i0].src_stride[i1].src_stride.next;
                    end
                end
            end
            assign hwif_out.dim[i0].src_stride[i1].src_stride.value = field_storage.dim[i0].src_stride[i1].src_stride.value;
        end
        for(genvar i1=0; i1<1; i1++) begin
            // Field: idma_reg.dim[].reps[].reps
            always_comb begin
                automatic logic [31:0] next_c;
                automatic logic load_next_c;
                next_c = field_storage.dim[i0].reps[i1].reps.value;
                load_next_c = '0;
                if(decoded_reg_strb.dim[i0].reps[i1] && decoded_req_is_wr) begin // SW write
                    next_c = (field_storage.dim[i0].reps[i1].reps.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                    load_next_c = '1;
                end
                field_combo.dim[i0].reps[i1].reps.next = next_c;
                field_combo.dim[i0].reps[i1].reps.load_next = load_next_c;
            end
            always_ff @(posedge clk or negedge arst_n) begin
                if(~arst_n) begin
                    field_storage.dim[i0].reps[i1].reps.value <= 32'h0;
                end else begin
                    if(field_combo.dim[i0].reps[i1].reps.load_next) begin
                        field_storage.dim[i0].reps[i1].reps.value <= field_combo.dim[i0].reps[i1].reps.next;
                    end
                end
            end
            assign hwif_out.dim[i0].reps[i1].reps.value = field_storage.dim[i0].reps[i1].reps.value;
        end
    end

    //--------------------------------------------------------------------------
    // Write response
    //--------------------------------------------------------------------------
    always_comb begin
        automatic logic wr_ack;
        wr_ack = '0;
        
        external_wr_ack = wr_ack;
    end
    assign cpuif_wr_ack = external_wr_ack | (decoded_req & decoded_req_is_wr & ~decoded_req_is_external);
    // Writes are always granted with no error response
    assign cpuif_wr_err = '0;

    //--------------------------------------------------------------------------
    // Readback
    //--------------------------------------------------------------------------
    logic readback_external_rd_ack_c;
    always_comb begin
        automatic logic rd_ack;
        rd_ack = '0;
        for(int i0=0; i0<16; i0++) begin
            rd_ack |= hwif_in.status[i0].rd_ack;
        end
        for(int i0=0; i0<16; i0++) begin
            rd_ack |= hwif_in.next_id[i0].rd_ack;
        end
        for(int i0=0; i0<16; i0++) begin
            rd_ack |= hwif_in.done_id[i0].rd_ack;
        end
        readback_external_rd_ack_c = rd_ack;
    end

    logic readback_external_rd_ack;

    assign readback_external_rd_ack = readback_external_rd_ack_c;

    logic [7:0] rd_mux_addr;
    logic [7:0] pending_rd_addr;
    // Hold read mux address to guarantee it is stable throughout any external accesses
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            pending_rd_addr <= '0;
        end else begin
            if(decoded_req) pending_rd_addr <= decoded_addr;
        end
    end
    assign rd_mux_addr = decoded_req ? decoded_addr : pending_rd_addr;

    logic readback_err;
    logic readback_done;
    logic [31:0] readback_data;
    always_comb begin
        automatic logic [31:0] readback_data_var;
        readback_data_var = '0;
        if(rd_mux_addr == 8'h0) begin
            readback_data_var[0] = field_storage.conf.decouple_aw.value;
            readback_data_var[1] = field_storage.conf.decouple_rw.value;
            readback_data_var[2] = field_storage.conf.src_reduce_len.value;
            readback_data_var[3] = field_storage.conf.dst_reduce_len.value;
            readback_data_var[6:4] = field_storage.conf.src_max_llen.value;
            readback_data_var[9:7] = field_storage.conf.dst_max_llen.value;
            readback_data_var[12:10] = field_storage.conf.enable_nd.value;
            readback_data_var[15:13] = field_storage.conf.src_protocol.value;
            readback_data_var[18:16] = field_storage.conf.dst_protocol.value;
        end
        for(int i0=0; i0<16; i0++) begin
            if(rd_mux_addr == 8'h4 + (8)'(i0) * 8'h4) begin
                readback_data_var = hwif_in.status[i0].rd_data;
            end
        end
        for(int i0=0; i0<16; i0++) begin
            if(rd_mux_addr == 8'h44 + (8)'(i0) * 8'h4) begin
                readback_data_var = hwif_in.next_id[i0].rd_data;
            end
        end
        for(int i0=0; i0<16; i0++) begin
            if(rd_mux_addr == 8'h84 + (8)'(i0) * 8'h4) begin
                readback_data_var = hwif_in.done_id[i0].rd_data;
            end
        end
        for(int i0=0; i0<1; i0++) begin
            if(rd_mux_addr == 8'hd0 + (8)'(i0) * 8'h4) begin
                readback_data_var[31:0] = field_storage.dst_addr[i0].dst_addr.value;
            end
        end
        for(int i0=0; i0<1; i0++) begin
            if(rd_mux_addr == 8'hd4 + (8)'(i0) * 8'h4) begin
                readback_data_var[31:0] = field_storage.src_addr[i0].src_addr.value;
            end
        end
        for(int i0=0; i0<1; i0++) begin
            if(rd_mux_addr == 8'hd8 + (8)'(i0) * 8'h4) begin
                readback_data_var[31:0] = field_storage.length[i0].length.value;
            end
        end
        for(int i0=0; i0<2; i0++) begin
            for(int i1=0; i1<1; i1++) begin
                if(rd_mux_addr == 8'he0 + (8)'(i0) * 8'hc + (8)'(i1) * 8'h4) begin
                    readback_data_var[31:0] = field_storage.dim[i0].dst_stride[i1].dst_stride.value;
                end
            end
            for(int i1=0; i1<1; i1++) begin
                if(rd_mux_addr == 8'he4 + (8)'(i0) * 8'hc + (8)'(i1) * 8'h4) begin
                    readback_data_var[31:0] = field_storage.dim[i0].src_stride[i1].src_stride.value;
                end
            end
            for(int i1=0; i1<1; i1++) begin
                if(rd_mux_addr == 8'he8 + (8)'(i0) * 8'hc + (8)'(i1) * 8'h4) begin
                    readback_data_var[31:0] = field_storage.dim[i0].reps[i1].reps.value;
                end
            end
        end
        readback_data = readback_data_var;
        readback_done = decoded_req & ~decoded_req_is_wr & ~decoded_req_is_external;
        readback_err = '0;
    end

    assign external_rd_ack = readback_external_rd_ack;
    assign cpuif_rd_ack = readback_done | readback_external_rd_ack;
    assign cpuif_rd_data = readback_data;
    assign cpuif_rd_err = readback_err;
endmodule
// Generated by PeakRDL-regblock - A free and open-source SystemVerilog generator
//  https://github.com/SystemRDL/PeakRDL-regblock

module idma_reg64_2d_reg_top #(
        parameter ID_WIDTH = 1
    ) (
        input wire clk,
        input wire arst_n,

        input wire s_obi_req,
        output logic s_obi_gnt,
        input wire [8:0] s_obi_addr,
        input wire s_obi_we,
        input wire [3:0] s_obi_be,
        input wire [31:0] s_obi_wdata,
        input wire [ID_WIDTH-1:0] s_obi_aid,
        output logic s_obi_rvalid,
        input wire s_obi_rready,
        output logic [31:0] s_obi_rdata,
        output logic s_obi_err,
        output logic [ID_WIDTH-1:0] s_obi_rid,

        input idma_reg64_2d_reg_pkg::idma_reg__in_t hwif_in,
        output idma_reg64_2d_reg_pkg::idma_reg__out_t hwif_out
    );

    //--------------------------------------------------------------------------
    // CPU Bus interface logic
    //--------------------------------------------------------------------------
    logic cpuif_req;
    logic cpuif_req_is_wr;
    logic [8:0] cpuif_addr;
    logic [31:0] cpuif_wr_data;
    logic [31:0] cpuif_wr_biten;
    logic cpuif_req_stall_wr;
    logic cpuif_req_stall_rd;

    logic cpuif_rd_ack;
    logic cpuif_rd_err;
    logic [31:0] cpuif_rd_data;

    logic cpuif_wr_ack;
    logic cpuif_wr_err;

    // State & holding regs
    logic is_active; // A request is being served (not yet fully responded)
    logic gnt_q; // one-cycle grant for A-channel
    logic rsp_pending; // response ready but not yet accepted by manager
    logic [31:0] rsp_rdata_q;
    logic rsp_err_q;
    logic [$bits(s_obi_rid)-1:0] rid_q;

    // Latch AID on accept to echo back the response
    always_ff @(posedge clk or negedge arst_n) begin
        if (~arst_n) begin
            is_active <= 1'b0;
            gnt_q <= 1'b0;
            rsp_pending <= 1'b0;
            rsp_rdata_q <= '0;
            rsp_err_q <= 1'b0;
            rid_q <= '0;

            cpuif_req <= '0;
            cpuif_req_is_wr <= '0;
            cpuif_addr <= '0;
            cpuif_wr_data <= '0;
            cpuif_wr_biten <= '0;
        end else begin
            // defaults
            cpuif_req <= 1'b0;
            gnt_q <= s_obi_req & ~is_active;

            // Accept new request when idle
            if (~is_active) begin
                if (s_obi_req) begin
                    is_active <= 1'b1;
                    cpuif_req <= 1'b1;
                    cpuif_req_is_wr <= s_obi_we;
                    cpuif_addr <= {s_obi_addr[8:2], 2'b0};
                    cpuif_wr_data <= s_obi_wdata;
                    rid_q <= s_obi_aid;
                    for (int i = 0; i < 4; i++) begin
                        cpuif_wr_biten[i*8 +: 8] <= {8{ s_obi_be[i] }};
                    end
                end
            end

            // Capture response
            if (is_active && (cpuif_rd_ack || cpuif_wr_ack)) begin
                rsp_pending <= 1'b1;
                rsp_rdata_q <= cpuif_rd_data;
                rsp_err_q <= cpuif_rd_err | cpuif_wr_err;
                // NOTE: Keep 'is_active' asserted until the external R handshake completes
            end

            // Complete external R-channel handshake only if manager ready
            if (rsp_pending && s_obi_rvalid && s_obi_rready) begin
                rsp_pending <= 1'b0;
                is_active <= 1'b0; // free to accept the next request
            end
        end
    end

    // R-channel outputs (held stable while rsp_pending=1)
    assign s_obi_rvalid = rsp_pending;
    assign s_obi_rdata = rsp_rdata_q;
    assign s_obi_err = rsp_err_q;
    assign s_obi_rid = rid_q;

    // A-channel grant (registered one-cycle pulse when we accept a request)
    assign s_obi_gnt = gnt_q;

    logic cpuif_req_masked;
    logic external_pending;

    // Read & write latencies are balanced. Stalls not required
    // except if external
    assign cpuif_req_stall_rd = external_pending;
    assign cpuif_req_stall_wr = external_pending;
    assign cpuif_req_masked = cpuif_req
                            & !(!cpuif_req_is_wr & cpuif_req_stall_rd)
                            & !(cpuif_req_is_wr & cpuif_req_stall_wr);

    //--------------------------------------------------------------------------
    // Address Decode
    //--------------------------------------------------------------------------
    typedef struct {
        logic conf;
        logic status[16];
        logic next_id[16];
        logic done_id[16];
        logic dst_addr[2];
        logic src_addr[2];
        logic length[2];
        struct {
            logic dst_stride[2];
            logic src_stride[2];
            logic reps[2];
        } dim[1];
    } decoded_reg_strb_t;
    decoded_reg_strb_t decoded_reg_strb;
    logic decoded_err;
    logic decoded_req_is_external;

    logic [8:0] decoded_addr;
    logic decoded_req;
    logic decoded_req_is_wr;
    logic [31:0] decoded_wr_data;
    logic [31:0] decoded_wr_biten;

    always_comb begin
        automatic logic is_valid_addr;
        automatic logic is_valid_rw;
        automatic logic is_external;
        is_external = '0;
        is_valid_addr = '1; // No valid address check
        is_valid_rw = '1; // No valid RW check
        decoded_reg_strb.conf = cpuif_req_masked & (cpuif_addr == 9'h0);
        for(int i0=0; i0<16; i0++) begin
            decoded_reg_strb.status[i0] = cpuif_req_masked & (cpuif_addr == 9'h4 + (9)'(i0) * 9'h4) & !cpuif_req_is_wr;
            is_external |= cpuif_req_masked & (cpuif_addr == 9'h4 + (9)'(i0) * 9'h4) & !cpuif_req_is_wr;
        end
        for(int i0=0; i0<16; i0++) begin
            decoded_reg_strb.next_id[i0] = cpuif_req_masked & (cpuif_addr == 9'h44 + (9)'(i0) * 9'h4) & !cpuif_req_is_wr;
            is_external |= cpuif_req_masked & (cpuif_addr == 9'h44 + (9)'(i0) * 9'h4) & !cpuif_req_is_wr;
        end
        for(int i0=0; i0<16; i0++) begin
            decoded_reg_strb.done_id[i0] = cpuif_req_masked & (cpuif_addr == 9'h84 + (9)'(i0) * 9'h4) & !cpuif_req_is_wr;
            is_external |= cpuif_req_masked & (cpuif_addr == 9'h84 + (9)'(i0) * 9'h4) & !cpuif_req_is_wr;
        end
        for(int i0=0; i0<2; i0++) begin
            decoded_reg_strb.dst_addr[i0] = cpuif_req_masked & (cpuif_addr == 9'hd0 + (9)'(i0) * 9'h4);
        end
        for(int i0=0; i0<2; i0++) begin
            decoded_reg_strb.src_addr[i0] = cpuif_req_masked & (cpuif_addr == 9'hd8 + (9)'(i0) * 9'h4);
        end
        for(int i0=0; i0<2; i0++) begin
            decoded_reg_strb.length[i0] = cpuif_req_masked & (cpuif_addr == 9'he0 + (9)'(i0) * 9'h4);
        end
        for(int i0=0; i0<1; i0++) begin
            for(int i1=0; i1<2; i1++) begin
                decoded_reg_strb.dim[i0].dst_stride[i1] = cpuif_req_masked & (cpuif_addr == 9'h100 + (9)'(i0) * 9'h18 + (9)'(i1) * 9'h4);
            end
            for(int i1=0; i1<2; i1++) begin
                decoded_reg_strb.dim[i0].src_stride[i1] = cpuif_req_masked & (cpuif_addr == 9'h108 + (9)'(i0) * 9'h18 + (9)'(i1) * 9'h4);
            end
            for(int i1=0; i1<2; i1++) begin
                decoded_reg_strb.dim[i0].reps[i1] = cpuif_req_masked & (cpuif_addr == 9'h110 + (9)'(i0) * 9'h18 + (9)'(i1) * 9'h4);
            end
        end
        decoded_err = '0;
        decoded_req_is_external = is_external;
    end
    logic external_wr_ack;
    logic external_rd_ack;
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            external_pending <= '0;
        end else begin
            if(decoded_req_is_external & ~external_wr_ack & ~external_rd_ack) external_pending <= '1;
            else if(external_wr_ack | external_rd_ack) external_pending <= '0;
            `ifndef SYNTHESIS
                assert_bad_ext_wr_ack: assert(!external_wr_ack || (external_pending | decoded_req_is_external))
                    else $error("An external wr_ack strobe was asserted when no external request was active");
                assert_bad_ext_rd_ack: assert(!external_rd_ack || (external_pending | decoded_req_is_external))
                    else $error("An external rd_ack strobe was asserted when no external request was active");
            `endif
        end
    end

    // Pass down signals to next stage
    assign decoded_addr = cpuif_addr;
    assign decoded_req = cpuif_req_masked;
    assign decoded_req_is_wr = cpuif_req_is_wr;
    assign decoded_wr_data = cpuif_wr_data;
    assign decoded_wr_biten = cpuif_wr_biten;

    //--------------------------------------------------------------------------
    // Field logic
    //--------------------------------------------------------------------------
    typedef struct {
        struct {
            struct {
                logic next;
                logic load_next;
            } decouple_aw;
            struct {
                logic next;
                logic load_next;
            } decouple_rw;
            struct {
                logic next;
                logic load_next;
            } src_reduce_len;
            struct {
                logic next;
                logic load_next;
            } dst_reduce_len;
            struct {
                logic [2:0] next;
                logic load_next;
            } src_max_llen;
            struct {
                logic [2:0] next;
                logic load_next;
            } dst_max_llen;
            struct {
                logic [1:0] next;
                logic load_next;
            } enable_nd;
            struct {
                logic [2:0] next;
                logic load_next;
            } src_protocol;
            struct {
                logic [2:0] next;
                logic load_next;
            } dst_protocol;
        } conf;
        struct {
            struct {
                logic [31:0] next;
                logic load_next;
            } dst_addr;
        } dst_addr[2];
        struct {
            struct {
                logic [31:0] next;
                logic load_next;
            } src_addr;
        } src_addr[2];
        struct {
            struct {
                logic [31:0] next;
                logic load_next;
            } length;
        } length[2];
        struct {
            struct {
                struct {
                    logic [31:0] next;
                    logic load_next;
                } dst_stride;
            } dst_stride[2];
            struct {
                struct {
                    logic [31:0] next;
                    logic load_next;
                } src_stride;
            } src_stride[2];
            struct {
                struct {
                    logic [31:0] next;
                    logic load_next;
                } reps;
            } reps[2];
        } dim[1];
    } field_combo_t;
    field_combo_t field_combo;

    typedef struct {
        struct {
            struct {
                logic value;
            } decouple_aw;
            struct {
                logic value;
            } decouple_rw;
            struct {
                logic value;
            } src_reduce_len;
            struct {
                logic value;
            } dst_reduce_len;
            struct {
                logic [2:0] value;
            } src_max_llen;
            struct {
                logic [2:0] value;
            } dst_max_llen;
            struct {
                logic [1:0] value;
            } enable_nd;
            struct {
                logic [2:0] value;
            } src_protocol;
            struct {
                logic [2:0] value;
            } dst_protocol;
        } conf;
        struct {
            struct {
                logic [31:0] value;
            } dst_addr;
        } dst_addr[2];
        struct {
            struct {
                logic [31:0] value;
            } src_addr;
        } src_addr[2];
        struct {
            struct {
                logic [31:0] value;
            } length;
        } length[2];
        struct {
            struct {
                struct {
                    logic [31:0] value;
                } dst_stride;
            } dst_stride[2];
            struct {
                struct {
                    logic [31:0] value;
                } src_stride;
            } src_stride[2];
            struct {
                struct {
                    logic [31:0] value;
                } reps;
            } reps[2];
        } dim[1];
    } field_storage_t;
    field_storage_t field_storage;

    // Field: idma_reg.conf.decouple_aw
    always_comb begin
        automatic logic [0:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.decouple_aw.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.decouple_aw.value & ~decoded_wr_biten[0:0]) | (decoded_wr_data[0:0] & decoded_wr_biten[0:0]);
            load_next_c = '1;
        end
        field_combo.conf.decouple_aw.next = next_c;
        field_combo.conf.decouple_aw.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.decouple_aw.value <= 1'h0;
        end else begin
            if(field_combo.conf.decouple_aw.load_next) begin
                field_storage.conf.decouple_aw.value <= field_combo.conf.decouple_aw.next;
            end
        end
    end
    assign hwif_out.conf.decouple_aw.value = field_storage.conf.decouple_aw.value;
    // Field: idma_reg.conf.decouple_rw
    always_comb begin
        automatic logic [0:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.decouple_rw.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.decouple_rw.value & ~decoded_wr_biten[1:1]) | (decoded_wr_data[1:1] & decoded_wr_biten[1:1]);
            load_next_c = '1;
        end
        field_combo.conf.decouple_rw.next = next_c;
        field_combo.conf.decouple_rw.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.decouple_rw.value <= 1'h0;
        end else begin
            if(field_combo.conf.decouple_rw.load_next) begin
                field_storage.conf.decouple_rw.value <= field_combo.conf.decouple_rw.next;
            end
        end
    end
    assign hwif_out.conf.decouple_rw.value = field_storage.conf.decouple_rw.value;
    // Field: idma_reg.conf.src_reduce_len
    always_comb begin
        automatic logic [0:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.src_reduce_len.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.src_reduce_len.value & ~decoded_wr_biten[2:2]) | (decoded_wr_data[2:2] & decoded_wr_biten[2:2]);
            load_next_c = '1;
        end
        field_combo.conf.src_reduce_len.next = next_c;
        field_combo.conf.src_reduce_len.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.src_reduce_len.value <= 1'h0;
        end else begin
            if(field_combo.conf.src_reduce_len.load_next) begin
                field_storage.conf.src_reduce_len.value <= field_combo.conf.src_reduce_len.next;
            end
        end
    end
    assign hwif_out.conf.src_reduce_len.value = field_storage.conf.src_reduce_len.value;
    // Field: idma_reg.conf.dst_reduce_len
    always_comb begin
        automatic logic [0:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.dst_reduce_len.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.dst_reduce_len.value & ~decoded_wr_biten[3:3]) | (decoded_wr_data[3:3] & decoded_wr_biten[3:3]);
            load_next_c = '1;
        end
        field_combo.conf.dst_reduce_len.next = next_c;
        field_combo.conf.dst_reduce_len.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.dst_reduce_len.value <= 1'h0;
        end else begin
            if(field_combo.conf.dst_reduce_len.load_next) begin
                field_storage.conf.dst_reduce_len.value <= field_combo.conf.dst_reduce_len.next;
            end
        end
    end
    assign hwif_out.conf.dst_reduce_len.value = field_storage.conf.dst_reduce_len.value;
    // Field: idma_reg.conf.src_max_llen
    always_comb begin
        automatic logic [2:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.src_max_llen.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.src_max_llen.value & ~decoded_wr_biten[6:4]) | (decoded_wr_data[6:4] & decoded_wr_biten[6:4]);
            load_next_c = '1;
        end
        field_combo.conf.src_max_llen.next = next_c;
        field_combo.conf.src_max_llen.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.src_max_llen.value <= 3'h0;
        end else begin
            if(field_combo.conf.src_max_llen.load_next) begin
                field_storage.conf.src_max_llen.value <= field_combo.conf.src_max_llen.next;
            end
        end
    end
    assign hwif_out.conf.src_max_llen.value = field_storage.conf.src_max_llen.value;
    // Field: idma_reg.conf.dst_max_llen
    always_comb begin
        automatic logic [2:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.dst_max_llen.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.dst_max_llen.value & ~decoded_wr_biten[9:7]) | (decoded_wr_data[9:7] & decoded_wr_biten[9:7]);
            load_next_c = '1;
        end
        field_combo.conf.dst_max_llen.next = next_c;
        field_combo.conf.dst_max_llen.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.dst_max_llen.value <= 3'h0;
        end else begin
            if(field_combo.conf.dst_max_llen.load_next) begin
                field_storage.conf.dst_max_llen.value <= field_combo.conf.dst_max_llen.next;
            end
        end
    end
    assign hwif_out.conf.dst_max_llen.value = field_storage.conf.dst_max_llen.value;
    // Field: idma_reg.conf.enable_nd
    always_comb begin
        automatic logic [1:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.enable_nd.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.enable_nd.value & ~decoded_wr_biten[11:10]) | (decoded_wr_data[11:10] & decoded_wr_biten[11:10]);
            load_next_c = '1;
        end
        field_combo.conf.enable_nd.next = next_c;
        field_combo.conf.enable_nd.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.enable_nd.value <= 2'h0;
        end else begin
            if(field_combo.conf.enable_nd.load_next) begin
                field_storage.conf.enable_nd.value <= field_combo.conf.enable_nd.next;
            end
        end
    end
    assign hwif_out.conf.enable_nd.value = field_storage.conf.enable_nd.value;
    // Field: idma_reg.conf.src_protocol
    always_comb begin
        automatic logic [2:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.src_protocol.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.src_protocol.value & ~decoded_wr_biten[14:12]) | (decoded_wr_data[14:12] & decoded_wr_biten[14:12]);
            load_next_c = '1;
        end
        field_combo.conf.src_protocol.next = next_c;
        field_combo.conf.src_protocol.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.src_protocol.value <= 3'h0;
        end else begin
            if(field_combo.conf.src_protocol.load_next) begin
                field_storage.conf.src_protocol.value <= field_combo.conf.src_protocol.next;
            end
        end
    end
    assign hwif_out.conf.src_protocol.value = field_storage.conf.src_protocol.value;
    // Field: idma_reg.conf.dst_protocol
    always_comb begin
        automatic logic [2:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.dst_protocol.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.dst_protocol.value & ~decoded_wr_biten[17:15]) | (decoded_wr_data[17:15] & decoded_wr_biten[17:15]);
            load_next_c = '1;
        end
        field_combo.conf.dst_protocol.next = next_c;
        field_combo.conf.dst_protocol.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.dst_protocol.value <= 3'h0;
        end else begin
            if(field_combo.conf.dst_protocol.load_next) begin
                field_storage.conf.dst_protocol.value <= field_combo.conf.dst_protocol.next;
            end
        end
    end
    assign hwif_out.conf.dst_protocol.value = field_storage.conf.dst_protocol.value;
    for(genvar i0=0; i0<16; i0++) begin
        // External register: idma_reg.status[]

        assign hwif_out.status[i0].req = !decoded_req_is_wr ? decoded_reg_strb.status[i0] : '0;
        assign hwif_out.status[i0].req_is_wr = decoded_req_is_wr;
    end
    for(genvar i0=0; i0<16; i0++) begin
        // External register: idma_reg.next_id[]

        assign hwif_out.next_id[i0].req = !decoded_req_is_wr ? decoded_reg_strb.next_id[i0] : '0;
        assign hwif_out.next_id[i0].req_is_wr = decoded_req_is_wr;
    end
    for(genvar i0=0; i0<16; i0++) begin
        // External register: idma_reg.done_id[]

        assign hwif_out.done_id[i0].req = !decoded_req_is_wr ? decoded_reg_strb.done_id[i0] : '0;
        assign hwif_out.done_id[i0].req_is_wr = decoded_req_is_wr;
    end
    for(genvar i0=0; i0<2; i0++) begin
        // Field: idma_reg.dst_addr[].dst_addr
        always_comb begin
            automatic logic [31:0] next_c;
            automatic logic load_next_c;
            next_c = field_storage.dst_addr[i0].dst_addr.value;
            load_next_c = '0;
            if(decoded_reg_strb.dst_addr[i0] && decoded_req_is_wr) begin // SW write
                next_c = (field_storage.dst_addr[i0].dst_addr.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                load_next_c = '1;
            end
            field_combo.dst_addr[i0].dst_addr.next = next_c;
            field_combo.dst_addr[i0].dst_addr.load_next = load_next_c;
        end
        always_ff @(posedge clk or negedge arst_n) begin
            if(~arst_n) begin
                field_storage.dst_addr[i0].dst_addr.value <= 32'h0;
            end else begin
                if(field_combo.dst_addr[i0].dst_addr.load_next) begin
                    field_storage.dst_addr[i0].dst_addr.value <= field_combo.dst_addr[i0].dst_addr.next;
                end
            end
        end
        assign hwif_out.dst_addr[i0].dst_addr.value = field_storage.dst_addr[i0].dst_addr.value;
    end
    for(genvar i0=0; i0<2; i0++) begin
        // Field: idma_reg.src_addr[].src_addr
        always_comb begin
            automatic logic [31:0] next_c;
            automatic logic load_next_c;
            next_c = field_storage.src_addr[i0].src_addr.value;
            load_next_c = '0;
            if(decoded_reg_strb.src_addr[i0] && decoded_req_is_wr) begin // SW write
                next_c = (field_storage.src_addr[i0].src_addr.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                load_next_c = '1;
            end
            field_combo.src_addr[i0].src_addr.next = next_c;
            field_combo.src_addr[i0].src_addr.load_next = load_next_c;
        end
        always_ff @(posedge clk or negedge arst_n) begin
            if(~arst_n) begin
                field_storage.src_addr[i0].src_addr.value <= 32'h0;
            end else begin
                if(field_combo.src_addr[i0].src_addr.load_next) begin
                    field_storage.src_addr[i0].src_addr.value <= field_combo.src_addr[i0].src_addr.next;
                end
            end
        end
        assign hwif_out.src_addr[i0].src_addr.value = field_storage.src_addr[i0].src_addr.value;
    end
    for(genvar i0=0; i0<2; i0++) begin
        // Field: idma_reg.length[].length
        always_comb begin
            automatic logic [31:0] next_c;
            automatic logic load_next_c;
            next_c = field_storage.length[i0].length.value;
            load_next_c = '0;
            if(decoded_reg_strb.length[i0] && decoded_req_is_wr) begin // SW write
                next_c = (field_storage.length[i0].length.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                load_next_c = '1;
            end
            field_combo.length[i0].length.next = next_c;
            field_combo.length[i0].length.load_next = load_next_c;
        end
        always_ff @(posedge clk or negedge arst_n) begin
            if(~arst_n) begin
                field_storage.length[i0].length.value <= 32'h0;
            end else begin
                if(field_combo.length[i0].length.load_next) begin
                    field_storage.length[i0].length.value <= field_combo.length[i0].length.next;
                end
            end
        end
        assign hwif_out.length[i0].length.value = field_storage.length[i0].length.value;
    end
    for(genvar i0=0; i0<1; i0++) begin
        for(genvar i1=0; i1<2; i1++) begin
            // Field: idma_reg.dim[].dst_stride[].dst_stride
            always_comb begin
                automatic logic [31:0] next_c;
                automatic logic load_next_c;
                next_c = field_storage.dim[i0].dst_stride[i1].dst_stride.value;
                load_next_c = '0;
                if(decoded_reg_strb.dim[i0].dst_stride[i1] && decoded_req_is_wr) begin // SW write
                    next_c = (field_storage.dim[i0].dst_stride[i1].dst_stride.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                    load_next_c = '1;
                end
                field_combo.dim[i0].dst_stride[i1].dst_stride.next = next_c;
                field_combo.dim[i0].dst_stride[i1].dst_stride.load_next = load_next_c;
            end
            always_ff @(posedge clk or negedge arst_n) begin
                if(~arst_n) begin
                    field_storage.dim[i0].dst_stride[i1].dst_stride.value <= 32'h0;
                end else begin
                    if(field_combo.dim[i0].dst_stride[i1].dst_stride.load_next) begin
                        field_storage.dim[i0].dst_stride[i1].dst_stride.value <= field_combo.dim[i0].dst_stride[i1].dst_stride.next;
                    end
                end
            end
            assign hwif_out.dim[i0].dst_stride[i1].dst_stride.value = field_storage.dim[i0].dst_stride[i1].dst_stride.value;
        end
        for(genvar i1=0; i1<2; i1++) begin
            // Field: idma_reg.dim[].src_stride[].src_stride
            always_comb begin
                automatic logic [31:0] next_c;
                automatic logic load_next_c;
                next_c = field_storage.dim[i0].src_stride[i1].src_stride.value;
                load_next_c = '0;
                if(decoded_reg_strb.dim[i0].src_stride[i1] && decoded_req_is_wr) begin // SW write
                    next_c = (field_storage.dim[i0].src_stride[i1].src_stride.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                    load_next_c = '1;
                end
                field_combo.dim[i0].src_stride[i1].src_stride.next = next_c;
                field_combo.dim[i0].src_stride[i1].src_stride.load_next = load_next_c;
            end
            always_ff @(posedge clk or negedge arst_n) begin
                if(~arst_n) begin
                    field_storage.dim[i0].src_stride[i1].src_stride.value <= 32'h0;
                end else begin
                    if(field_combo.dim[i0].src_stride[i1].src_stride.load_next) begin
                        field_storage.dim[i0].src_stride[i1].src_stride.value <= field_combo.dim[i0].src_stride[i1].src_stride.next;
                    end
                end
            end
            assign hwif_out.dim[i0].src_stride[i1].src_stride.value = field_storage.dim[i0].src_stride[i1].src_stride.value;
        end
        for(genvar i1=0; i1<2; i1++) begin
            // Field: idma_reg.dim[].reps[].reps
            always_comb begin
                automatic logic [31:0] next_c;
                automatic logic load_next_c;
                next_c = field_storage.dim[i0].reps[i1].reps.value;
                load_next_c = '0;
                if(decoded_reg_strb.dim[i0].reps[i1] && decoded_req_is_wr) begin // SW write
                    next_c = (field_storage.dim[i0].reps[i1].reps.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                    load_next_c = '1;
                end
                field_combo.dim[i0].reps[i1].reps.next = next_c;
                field_combo.dim[i0].reps[i1].reps.load_next = load_next_c;
            end
            always_ff @(posedge clk or negedge arst_n) begin
                if(~arst_n) begin
                    field_storage.dim[i0].reps[i1].reps.value <= 32'h0;
                end else begin
                    if(field_combo.dim[i0].reps[i1].reps.load_next) begin
                        field_storage.dim[i0].reps[i1].reps.value <= field_combo.dim[i0].reps[i1].reps.next;
                    end
                end
            end
            assign hwif_out.dim[i0].reps[i1].reps.value = field_storage.dim[i0].reps[i1].reps.value;
        end
    end

    //--------------------------------------------------------------------------
    // Write response
    //--------------------------------------------------------------------------
    always_comb begin
        automatic logic wr_ack;
        wr_ack = '0;
        
        external_wr_ack = wr_ack;
    end
    assign cpuif_wr_ack = external_wr_ack | (decoded_req & decoded_req_is_wr & ~decoded_req_is_external);
    // Writes are always granted with no error response
    assign cpuif_wr_err = '0;

    //--------------------------------------------------------------------------
    // Readback
    //--------------------------------------------------------------------------
    logic readback_external_rd_ack_c;
    always_comb begin
        automatic logic rd_ack;
        rd_ack = '0;
        for(int i0=0; i0<16; i0++) begin
            rd_ack |= hwif_in.status[i0].rd_ack;
        end
        for(int i0=0; i0<16; i0++) begin
            rd_ack |= hwif_in.next_id[i0].rd_ack;
        end
        for(int i0=0; i0<16; i0++) begin
            rd_ack |= hwif_in.done_id[i0].rd_ack;
        end
        readback_external_rd_ack_c = rd_ack;
    end

    logic readback_external_rd_ack;

    assign readback_external_rd_ack = readback_external_rd_ack_c;

    logic [8:0] rd_mux_addr;
    logic [8:0] pending_rd_addr;
    // Hold read mux address to guarantee it is stable throughout any external accesses
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            pending_rd_addr <= '0;
        end else begin
            if(decoded_req) pending_rd_addr <= decoded_addr;
        end
    end
    assign rd_mux_addr = decoded_req ? decoded_addr : pending_rd_addr;

    logic readback_err;
    logic readback_done;
    logic [31:0] readback_data;
    always_comb begin
        automatic logic [31:0] readback_data_var;
        readback_data_var = '0;
        if(rd_mux_addr == 9'h0) begin
            readback_data_var[0] = field_storage.conf.decouple_aw.value;
            readback_data_var[1] = field_storage.conf.decouple_rw.value;
            readback_data_var[2] = field_storage.conf.src_reduce_len.value;
            readback_data_var[3] = field_storage.conf.dst_reduce_len.value;
            readback_data_var[6:4] = field_storage.conf.src_max_llen.value;
            readback_data_var[9:7] = field_storage.conf.dst_max_llen.value;
            readback_data_var[11:10] = field_storage.conf.enable_nd.value;
            readback_data_var[14:12] = field_storage.conf.src_protocol.value;
            readback_data_var[17:15] = field_storage.conf.dst_protocol.value;
        end
        for(int i0=0; i0<16; i0++) begin
            if(rd_mux_addr == 9'h4 + (9)'(i0) * 9'h4) begin
                readback_data_var = hwif_in.status[i0].rd_data;
            end
        end
        for(int i0=0; i0<16; i0++) begin
            if(rd_mux_addr == 9'h44 + (9)'(i0) * 9'h4) begin
                readback_data_var = hwif_in.next_id[i0].rd_data;
            end
        end
        for(int i0=0; i0<16; i0++) begin
            if(rd_mux_addr == 9'h84 + (9)'(i0) * 9'h4) begin
                readback_data_var = hwif_in.done_id[i0].rd_data;
            end
        end
        for(int i0=0; i0<2; i0++) begin
            if(rd_mux_addr == 9'hd0 + (9)'(i0) * 9'h4) begin
                readback_data_var[31:0] = field_storage.dst_addr[i0].dst_addr.value;
            end
        end
        for(int i0=0; i0<2; i0++) begin
            if(rd_mux_addr == 9'hd8 + (9)'(i0) * 9'h4) begin
                readback_data_var[31:0] = field_storage.src_addr[i0].src_addr.value;
            end
        end
        for(int i0=0; i0<2; i0++) begin
            if(rd_mux_addr == 9'he0 + (9)'(i0) * 9'h4) begin
                readback_data_var[31:0] = field_storage.length[i0].length.value;
            end
        end
        for(int i0=0; i0<1; i0++) begin
            for(int i1=0; i1<2; i1++) begin
                if(rd_mux_addr == 9'h100 + (9)'(i0) * 9'h18 + (9)'(i1) * 9'h4) begin
                    readback_data_var[31:0] = field_storage.dim[i0].dst_stride[i1].dst_stride.value;
                end
            end
            for(int i1=0; i1<2; i1++) begin
                if(rd_mux_addr == 9'h108 + (9)'(i0) * 9'h18 + (9)'(i1) * 9'h4) begin
                    readback_data_var[31:0] = field_storage.dim[i0].src_stride[i1].src_stride.value;
                end
            end
            for(int i1=0; i1<2; i1++) begin
                if(rd_mux_addr == 9'h110 + (9)'(i0) * 9'h18 + (9)'(i1) * 9'h4) begin
                    readback_data_var[31:0] = field_storage.dim[i0].reps[i1].reps.value;
                end
            end
        end
        readback_data = readback_data_var;
        readback_done = decoded_req & ~decoded_req_is_wr & ~decoded_req_is_external;
        readback_err = '0;
    end

    assign external_rd_ack = readback_external_rd_ack;
    assign cpuif_rd_ack = readback_done | readback_external_rd_ack;
    assign cpuif_rd_data = readback_data;
    assign cpuif_rd_err = readback_err;
endmodule
// Generated by PeakRDL-regblock - A free and open-source SystemVerilog generator
//  https://github.com/SystemRDL/PeakRDL-regblock

module idma_reg64_1d_reg_top #(
        parameter ID_WIDTH = 1
    ) (
        input wire clk,
        input wire arst_n,

        input wire s_obi_req,
        output logic s_obi_gnt,
        input wire [8:0] s_obi_addr,
        input wire s_obi_we,
        input wire [3:0] s_obi_be,
        input wire [31:0] s_obi_wdata,
        input wire [ID_WIDTH-1:0] s_obi_aid,
        output logic s_obi_rvalid,
        input wire s_obi_rready,
        output logic [31:0] s_obi_rdata,
        output logic s_obi_err,
        output logic [ID_WIDTH-1:0] s_obi_rid,

        input idma_reg64_1d_reg_pkg::idma_reg__in_t hwif_in,
        output idma_reg64_1d_reg_pkg::idma_reg__out_t hwif_out
    );

    //--------------------------------------------------------------------------
    // CPU Bus interface logic
    //--------------------------------------------------------------------------
    logic cpuif_req;
    logic cpuif_req_is_wr;
    logic [8:0] cpuif_addr;
    logic [31:0] cpuif_wr_data;
    logic [31:0] cpuif_wr_biten;
    logic cpuif_req_stall_wr;
    logic cpuif_req_stall_rd;

    logic cpuif_rd_ack;
    logic cpuif_rd_err;
    logic [31:0] cpuif_rd_data;

    logic cpuif_wr_ack;
    logic cpuif_wr_err;

    // State & holding regs
    logic is_active; // A request is being served (not yet fully responded)
    logic gnt_q; // one-cycle grant for A-channel
    logic rsp_pending; // response ready but not yet accepted by manager
    logic [31:0] rsp_rdata_q;
    logic rsp_err_q;
    logic [$bits(s_obi_rid)-1:0] rid_q;

    // Latch AID on accept to echo back the response
    always_ff @(posedge clk or negedge arst_n) begin
        if (~arst_n) begin
            is_active <= 1'b0;
            gnt_q <= 1'b0;
            rsp_pending <= 1'b0;
            rsp_rdata_q <= '0;
            rsp_err_q <= 1'b0;
            rid_q <= '0;

            cpuif_req <= '0;
            cpuif_req_is_wr <= '0;
            cpuif_addr <= '0;
            cpuif_wr_data <= '0;
            cpuif_wr_biten <= '0;
        end else begin
            // defaults
            cpuif_req <= 1'b0;
            gnt_q <= s_obi_req & ~is_active;

            // Accept new request when idle
            if (~is_active) begin
                if (s_obi_req) begin
                    is_active <= 1'b1;
                    cpuif_req <= 1'b1;
                    cpuif_req_is_wr <= s_obi_we;
                    cpuif_addr <= {s_obi_addr[8:2], 2'b0};
                    cpuif_wr_data <= s_obi_wdata;
                    rid_q <= s_obi_aid;
                    for (int i = 0; i < 4; i++) begin
                        cpuif_wr_biten[i*8 +: 8] <= {8{ s_obi_be[i] }};
                    end
                end
            end

            // Capture response
            if (is_active && (cpuif_rd_ack || cpuif_wr_ack)) begin
                rsp_pending <= 1'b1;
                rsp_rdata_q <= cpuif_rd_data;
                rsp_err_q <= cpuif_rd_err | cpuif_wr_err;
                // NOTE: Keep 'is_active' asserted until the external R handshake completes
            end

            // Complete external R-channel handshake only if manager ready
            if (rsp_pending && s_obi_rvalid && s_obi_rready) begin
                rsp_pending <= 1'b0;
                is_active <= 1'b0; // free to accept the next request
            end
        end
    end

    // R-channel outputs (held stable while rsp_pending=1)
    assign s_obi_rvalid = rsp_pending;
    assign s_obi_rdata = rsp_rdata_q;
    assign s_obi_err = rsp_err_q;
    assign s_obi_rid = rid_q;

    // A-channel grant (registered one-cycle pulse when we accept a request)
    assign s_obi_gnt = gnt_q;

    logic cpuif_req_masked;
    logic external_pending;

    // Read & write latencies are balanced. Stalls not required
    // except if external
    assign cpuif_req_stall_rd = external_pending;
    assign cpuif_req_stall_wr = external_pending;
    assign cpuif_req_masked = cpuif_req
                            & !(!cpuif_req_is_wr & cpuif_req_stall_rd)
                            & !(cpuif_req_is_wr & cpuif_req_stall_wr);

    //--------------------------------------------------------------------------
    // Address Decode
    //--------------------------------------------------------------------------
    typedef struct {
        logic conf;
        logic status[16];
        logic next_id[16];
        logic done_id[16];
        logic dst_addr[2];
        logic src_addr[2];
        logic length[2];
    } decoded_reg_strb_t;
    decoded_reg_strb_t decoded_reg_strb;
    logic decoded_err;
    logic decoded_req_is_external;

    logic [8:0] decoded_addr;
    logic decoded_req;
    logic decoded_req_is_wr;
    logic [31:0] decoded_wr_data;
    logic [31:0] decoded_wr_biten;

    always_comb begin
        automatic logic is_valid_addr;
        automatic logic is_valid_rw;
        automatic logic is_external;
        is_external = '0;
        is_valid_addr = '1; // No valid address check
        is_valid_rw = '1; // No valid RW check
        decoded_reg_strb.conf = cpuif_req_masked & (cpuif_addr == 9'h0);
        for(int i0=0; i0<16; i0++) begin
            decoded_reg_strb.status[i0] = cpuif_req_masked & (cpuif_addr == 9'h4 + (9)'(i0) * 9'h4) & !cpuif_req_is_wr;
            is_external |= cpuif_req_masked & (cpuif_addr == 9'h4 + (9)'(i0) * 9'h4) & !cpuif_req_is_wr;
        end
        for(int i0=0; i0<16; i0++) begin
            decoded_reg_strb.next_id[i0] = cpuif_req_masked & (cpuif_addr == 9'h44 + (9)'(i0) * 9'h4) & !cpuif_req_is_wr;
            is_external |= cpuif_req_masked & (cpuif_addr == 9'h44 + (9)'(i0) * 9'h4) & !cpuif_req_is_wr;
        end
        for(int i0=0; i0<16; i0++) begin
            decoded_reg_strb.done_id[i0] = cpuif_req_masked & (cpuif_addr == 9'h84 + (9)'(i0) * 9'h4) & !cpuif_req_is_wr;
            is_external |= cpuif_req_masked & (cpuif_addr == 9'h84 + (9)'(i0) * 9'h4) & !cpuif_req_is_wr;
        end
        for(int i0=0; i0<2; i0++) begin
            decoded_reg_strb.dst_addr[i0] = cpuif_req_masked & (cpuif_addr == 9'hd0 + (9)'(i0) * 9'h4);
        end
        for(int i0=0; i0<2; i0++) begin
            decoded_reg_strb.src_addr[i0] = cpuif_req_masked & (cpuif_addr == 9'hd8 + (9)'(i0) * 9'h4);
        end
        for(int i0=0; i0<2; i0++) begin
            decoded_reg_strb.length[i0] = cpuif_req_masked & (cpuif_addr == 9'he0 + (9)'(i0) * 9'h4);
        end
        decoded_err = '0;
        decoded_req_is_external = is_external;
    end
    logic external_wr_ack;
    logic external_rd_ack;
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            external_pending <= '0;
        end else begin
            if(decoded_req_is_external & ~external_wr_ack & ~external_rd_ack) external_pending <= '1;
            else if(external_wr_ack | external_rd_ack) external_pending <= '0;
            `ifndef SYNTHESIS
                assert_bad_ext_wr_ack: assert(!external_wr_ack || (external_pending | decoded_req_is_external))
                    else $error("An external wr_ack strobe was asserted when no external request was active");
                assert_bad_ext_rd_ack: assert(!external_rd_ack || (external_pending | decoded_req_is_external))
                    else $error("An external rd_ack strobe was asserted when no external request was active");
            `endif
        end
    end

    // Pass down signals to next stage
    assign decoded_addr = cpuif_addr;
    assign decoded_req = cpuif_req_masked;
    assign decoded_req_is_wr = cpuif_req_is_wr;
    assign decoded_wr_data = cpuif_wr_data;
    assign decoded_wr_biten = cpuif_wr_biten;

    //--------------------------------------------------------------------------
    // Field logic
    //--------------------------------------------------------------------------
    typedef struct {
        struct {
            struct {
                logic next;
                logic load_next;
            } decouple_aw;
            struct {
                logic next;
                logic load_next;
            } decouple_rw;
            struct {
                logic next;
                logic load_next;
            } src_reduce_len;
            struct {
                logic next;
                logic load_next;
            } dst_reduce_len;
            struct {
                logic [2:0] next;
                logic load_next;
            } src_max_llen;
            struct {
                logic [2:0] next;
                logic load_next;
            } dst_max_llen;
            struct {
                logic [1:0] next;
                logic load_next;
            } enable_nd;
            struct {
                logic [2:0] next;
                logic load_next;
            } src_protocol;
            struct {
                logic [2:0] next;
                logic load_next;
            } dst_protocol;
        } conf;
        struct {
            struct {
                logic [31:0] next;
                logic load_next;
            } dst_addr;
        } dst_addr[2];
        struct {
            struct {
                logic [31:0] next;
                logic load_next;
            } src_addr;
        } src_addr[2];
        struct {
            struct {
                logic [31:0] next;
                logic load_next;
            } length;
        } length[2];
    } field_combo_t;
    field_combo_t field_combo;

    typedef struct {
        struct {
            struct {
                logic value;
            } decouple_aw;
            struct {
                logic value;
            } decouple_rw;
            struct {
                logic value;
            } src_reduce_len;
            struct {
                logic value;
            } dst_reduce_len;
            struct {
                logic [2:0] value;
            } src_max_llen;
            struct {
                logic [2:0] value;
            } dst_max_llen;
            struct {
                logic [1:0] value;
            } enable_nd;
            struct {
                logic [2:0] value;
            } src_protocol;
            struct {
                logic [2:0] value;
            } dst_protocol;
        } conf;
        struct {
            struct {
                logic [31:0] value;
            } dst_addr;
        } dst_addr[2];
        struct {
            struct {
                logic [31:0] value;
            } src_addr;
        } src_addr[2];
        struct {
            struct {
                logic [31:0] value;
            } length;
        } length[2];
    } field_storage_t;
    field_storage_t field_storage;

    // Field: idma_reg.conf.decouple_aw
    always_comb begin
        automatic logic [0:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.decouple_aw.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.decouple_aw.value & ~decoded_wr_biten[0:0]) | (decoded_wr_data[0:0] & decoded_wr_biten[0:0]);
            load_next_c = '1;
        end
        field_combo.conf.decouple_aw.next = next_c;
        field_combo.conf.decouple_aw.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.decouple_aw.value <= 1'h0;
        end else begin
            if(field_combo.conf.decouple_aw.load_next) begin
                field_storage.conf.decouple_aw.value <= field_combo.conf.decouple_aw.next;
            end
        end
    end
    assign hwif_out.conf.decouple_aw.value = field_storage.conf.decouple_aw.value;
    // Field: idma_reg.conf.decouple_rw
    always_comb begin
        automatic logic [0:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.decouple_rw.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.decouple_rw.value & ~decoded_wr_biten[1:1]) | (decoded_wr_data[1:1] & decoded_wr_biten[1:1]);
            load_next_c = '1;
        end
        field_combo.conf.decouple_rw.next = next_c;
        field_combo.conf.decouple_rw.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.decouple_rw.value <= 1'h0;
        end else begin
            if(field_combo.conf.decouple_rw.load_next) begin
                field_storage.conf.decouple_rw.value <= field_combo.conf.decouple_rw.next;
            end
        end
    end
    assign hwif_out.conf.decouple_rw.value = field_storage.conf.decouple_rw.value;
    // Field: idma_reg.conf.src_reduce_len
    always_comb begin
        automatic logic [0:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.src_reduce_len.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.src_reduce_len.value & ~decoded_wr_biten[2:2]) | (decoded_wr_data[2:2] & decoded_wr_biten[2:2]);
            load_next_c = '1;
        end
        field_combo.conf.src_reduce_len.next = next_c;
        field_combo.conf.src_reduce_len.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.src_reduce_len.value <= 1'h0;
        end else begin
            if(field_combo.conf.src_reduce_len.load_next) begin
                field_storage.conf.src_reduce_len.value <= field_combo.conf.src_reduce_len.next;
            end
        end
    end
    assign hwif_out.conf.src_reduce_len.value = field_storage.conf.src_reduce_len.value;
    // Field: idma_reg.conf.dst_reduce_len
    always_comb begin
        automatic logic [0:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.dst_reduce_len.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.dst_reduce_len.value & ~decoded_wr_biten[3:3]) | (decoded_wr_data[3:3] & decoded_wr_biten[3:3]);
            load_next_c = '1;
        end
        field_combo.conf.dst_reduce_len.next = next_c;
        field_combo.conf.dst_reduce_len.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.dst_reduce_len.value <= 1'h0;
        end else begin
            if(field_combo.conf.dst_reduce_len.load_next) begin
                field_storage.conf.dst_reduce_len.value <= field_combo.conf.dst_reduce_len.next;
            end
        end
    end
    assign hwif_out.conf.dst_reduce_len.value = field_storage.conf.dst_reduce_len.value;
    // Field: idma_reg.conf.src_max_llen
    always_comb begin
        automatic logic [2:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.src_max_llen.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.src_max_llen.value & ~decoded_wr_biten[6:4]) | (decoded_wr_data[6:4] & decoded_wr_biten[6:4]);
            load_next_c = '1;
        end
        field_combo.conf.src_max_llen.next = next_c;
        field_combo.conf.src_max_llen.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.src_max_llen.value <= 3'h0;
        end else begin
            if(field_combo.conf.src_max_llen.load_next) begin
                field_storage.conf.src_max_llen.value <= field_combo.conf.src_max_llen.next;
            end
        end
    end
    assign hwif_out.conf.src_max_llen.value = field_storage.conf.src_max_llen.value;
    // Field: idma_reg.conf.dst_max_llen
    always_comb begin
        automatic logic [2:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.dst_max_llen.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.dst_max_llen.value & ~decoded_wr_biten[9:7]) | (decoded_wr_data[9:7] & decoded_wr_biten[9:7]);
            load_next_c = '1;
        end
        field_combo.conf.dst_max_llen.next = next_c;
        field_combo.conf.dst_max_llen.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.dst_max_llen.value <= 3'h0;
        end else begin
            if(field_combo.conf.dst_max_llen.load_next) begin
                field_storage.conf.dst_max_llen.value <= field_combo.conf.dst_max_llen.next;
            end
        end
    end
    assign hwif_out.conf.dst_max_llen.value = field_storage.conf.dst_max_llen.value;
    // Field: idma_reg.conf.enable_nd
    always_comb begin
        automatic logic [1:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.enable_nd.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.enable_nd.value & ~decoded_wr_biten[11:10]) | (decoded_wr_data[11:10] & decoded_wr_biten[11:10]);
            load_next_c = '1;
        end
        field_combo.conf.enable_nd.next = next_c;
        field_combo.conf.enable_nd.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.enable_nd.value <= 2'h0;
        end else begin
            if(field_combo.conf.enable_nd.load_next) begin
                field_storage.conf.enable_nd.value <= field_combo.conf.enable_nd.next;
            end
        end
    end
    assign hwif_out.conf.enable_nd.value = field_storage.conf.enable_nd.value;
    // Field: idma_reg.conf.src_protocol
    always_comb begin
        automatic logic [2:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.src_protocol.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.src_protocol.value & ~decoded_wr_biten[14:12]) | (decoded_wr_data[14:12] & decoded_wr_biten[14:12]);
            load_next_c = '1;
        end
        field_combo.conf.src_protocol.next = next_c;
        field_combo.conf.src_protocol.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.src_protocol.value <= 3'h0;
        end else begin
            if(field_combo.conf.src_protocol.load_next) begin
                field_storage.conf.src_protocol.value <= field_combo.conf.src_protocol.next;
            end
        end
    end
    assign hwif_out.conf.src_protocol.value = field_storage.conf.src_protocol.value;
    // Field: idma_reg.conf.dst_protocol
    always_comb begin
        automatic logic [2:0] next_c;
        automatic logic load_next_c;
        next_c = field_storage.conf.dst_protocol.value;
        load_next_c = '0;
        if(decoded_reg_strb.conf && decoded_req_is_wr) begin // SW write
            next_c = (field_storage.conf.dst_protocol.value & ~decoded_wr_biten[17:15]) | (decoded_wr_data[17:15] & decoded_wr_biten[17:15]);
            load_next_c = '1;
        end
        field_combo.conf.dst_protocol.next = next_c;
        field_combo.conf.dst_protocol.load_next = load_next_c;
    end
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            field_storage.conf.dst_protocol.value <= 3'h0;
        end else begin
            if(field_combo.conf.dst_protocol.load_next) begin
                field_storage.conf.dst_protocol.value <= field_combo.conf.dst_protocol.next;
            end
        end
    end
    assign hwif_out.conf.dst_protocol.value = field_storage.conf.dst_protocol.value;
    for(genvar i0=0; i0<16; i0++) begin
        // External register: idma_reg.status[]

        assign hwif_out.status[i0].req = !decoded_req_is_wr ? decoded_reg_strb.status[i0] : '0;
        assign hwif_out.status[i0].req_is_wr = decoded_req_is_wr;
    end
    for(genvar i0=0; i0<16; i0++) begin
        // External register: idma_reg.next_id[]

        assign hwif_out.next_id[i0].req = !decoded_req_is_wr ? decoded_reg_strb.next_id[i0] : '0;
        assign hwif_out.next_id[i0].req_is_wr = decoded_req_is_wr;
    end
    for(genvar i0=0; i0<16; i0++) begin
        // External register: idma_reg.done_id[]

        assign hwif_out.done_id[i0].req = !decoded_req_is_wr ? decoded_reg_strb.done_id[i0] : '0;
        assign hwif_out.done_id[i0].req_is_wr = decoded_req_is_wr;
    end
    for(genvar i0=0; i0<2; i0++) begin
        // Field: idma_reg.dst_addr[].dst_addr
        always_comb begin
            automatic logic [31:0] next_c;
            automatic logic load_next_c;
            next_c = field_storage.dst_addr[i0].dst_addr.value;
            load_next_c = '0;
            if(decoded_reg_strb.dst_addr[i0] && decoded_req_is_wr) begin // SW write
                next_c = (field_storage.dst_addr[i0].dst_addr.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                load_next_c = '1;
            end
            field_combo.dst_addr[i0].dst_addr.next = next_c;
            field_combo.dst_addr[i0].dst_addr.load_next = load_next_c;
        end
        always_ff @(posedge clk or negedge arst_n) begin
            if(~arst_n) begin
                field_storage.dst_addr[i0].dst_addr.value <= 32'h0;
            end else begin
                if(field_combo.dst_addr[i0].dst_addr.load_next) begin
                    field_storage.dst_addr[i0].dst_addr.value <= field_combo.dst_addr[i0].dst_addr.next;
                end
            end
        end
        assign hwif_out.dst_addr[i0].dst_addr.value = field_storage.dst_addr[i0].dst_addr.value;
    end
    for(genvar i0=0; i0<2; i0++) begin
        // Field: idma_reg.src_addr[].src_addr
        always_comb begin
            automatic logic [31:0] next_c;
            automatic logic load_next_c;
            next_c = field_storage.src_addr[i0].src_addr.value;
            load_next_c = '0;
            if(decoded_reg_strb.src_addr[i0] && decoded_req_is_wr) begin // SW write
                next_c = (field_storage.src_addr[i0].src_addr.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                load_next_c = '1;
            end
            field_combo.src_addr[i0].src_addr.next = next_c;
            field_combo.src_addr[i0].src_addr.load_next = load_next_c;
        end
        always_ff @(posedge clk or negedge arst_n) begin
            if(~arst_n) begin
                field_storage.src_addr[i0].src_addr.value <= 32'h0;
            end else begin
                if(field_combo.src_addr[i0].src_addr.load_next) begin
                    field_storage.src_addr[i0].src_addr.value <= field_combo.src_addr[i0].src_addr.next;
                end
            end
        end
        assign hwif_out.src_addr[i0].src_addr.value = field_storage.src_addr[i0].src_addr.value;
    end
    for(genvar i0=0; i0<2; i0++) begin
        // Field: idma_reg.length[].length
        always_comb begin
            automatic logic [31:0] next_c;
            automatic logic load_next_c;
            next_c = field_storage.length[i0].length.value;
            load_next_c = '0;
            if(decoded_reg_strb.length[i0] && decoded_req_is_wr) begin // SW write
                next_c = (field_storage.length[i0].length.value & ~decoded_wr_biten[31:0]) | (decoded_wr_data[31:0] & decoded_wr_biten[31:0]);
                load_next_c = '1;
            end
            field_combo.length[i0].length.next = next_c;
            field_combo.length[i0].length.load_next = load_next_c;
        end
        always_ff @(posedge clk or negedge arst_n) begin
            if(~arst_n) begin
                field_storage.length[i0].length.value <= 32'h0;
            end else begin
                if(field_combo.length[i0].length.load_next) begin
                    field_storage.length[i0].length.value <= field_combo.length[i0].length.next;
                end
            end
        end
        assign hwif_out.length[i0].length.value = field_storage.length[i0].length.value;
    end

    //--------------------------------------------------------------------------
    // Write response
    //--------------------------------------------------------------------------
    always_comb begin
        automatic logic wr_ack;
        wr_ack = '0;
        
        external_wr_ack = wr_ack;
    end
    assign cpuif_wr_ack = external_wr_ack | (decoded_req & decoded_req_is_wr & ~decoded_req_is_external);
    // Writes are always granted with no error response
    assign cpuif_wr_err = '0;

    //--------------------------------------------------------------------------
    // Readback
    //--------------------------------------------------------------------------
    logic readback_external_rd_ack_c;
    always_comb begin
        automatic logic rd_ack;
        rd_ack = '0;
        for(int i0=0; i0<16; i0++) begin
            rd_ack |= hwif_in.status[i0].rd_ack;
        end
        for(int i0=0; i0<16; i0++) begin
            rd_ack |= hwif_in.next_id[i0].rd_ack;
        end
        for(int i0=0; i0<16; i0++) begin
            rd_ack |= hwif_in.done_id[i0].rd_ack;
        end
        readback_external_rd_ack_c = rd_ack;
    end

    logic readback_external_rd_ack;

    assign readback_external_rd_ack = readback_external_rd_ack_c;

    logic [8:0] rd_mux_addr;
    logic [8:0] pending_rd_addr;
    // Hold read mux address to guarantee it is stable throughout any external accesses
    always_ff @(posedge clk or negedge arst_n) begin
        if(~arst_n) begin
            pending_rd_addr <= '0;
        end else begin
            if(decoded_req) pending_rd_addr <= decoded_addr;
        end
    end
    assign rd_mux_addr = decoded_req ? decoded_addr : pending_rd_addr;

    logic readback_err;
    logic readback_done;
    logic [31:0] readback_data;
    always_comb begin
        automatic logic [31:0] readback_data_var;
        readback_data_var = '0;
        if(rd_mux_addr == 9'h0) begin
            readback_data_var[0] = field_storage.conf.decouple_aw.value;
            readback_data_var[1] = field_storage.conf.decouple_rw.value;
            readback_data_var[2] = field_storage.conf.src_reduce_len.value;
            readback_data_var[3] = field_storage.conf.dst_reduce_len.value;
            readback_data_var[6:4] = field_storage.conf.src_max_llen.value;
            readback_data_var[9:7] = field_storage.conf.dst_max_llen.value;
            readback_data_var[11:10] = field_storage.conf.enable_nd.value;
            readback_data_var[14:12] = field_storage.conf.src_protocol.value;
            readback_data_var[17:15] = field_storage.conf.dst_protocol.value;
        end
        for(int i0=0; i0<16; i0++) begin
            if(rd_mux_addr == 9'h4 + (9)'(i0) * 9'h4) begin
                readback_data_var = hwif_in.status[i0].rd_data;
            end
        end
        for(int i0=0; i0<16; i0++) begin
            if(rd_mux_addr == 9'h44 + (9)'(i0) * 9'h4) begin
                readback_data_var = hwif_in.next_id[i0].rd_data;
            end
        end
        for(int i0=0; i0<16; i0++) begin
            if(rd_mux_addr == 9'h84 + (9)'(i0) * 9'h4) begin
                readback_data_var = hwif_in.done_id[i0].rd_data;
            end
        end
        for(int i0=0; i0<2; i0++) begin
            if(rd_mux_addr == 9'hd0 + (9)'(i0) * 9'h4) begin
                readback_data_var[31:0] = field_storage.dst_addr[i0].dst_addr.value;
            end
        end
        for(int i0=0; i0<2; i0++) begin
            if(rd_mux_addr == 9'hd8 + (9)'(i0) * 9'h4) begin
                readback_data_var[31:0] = field_storage.src_addr[i0].src_addr.value;
            end
        end
        for(int i0=0; i0<2; i0++) begin
            if(rd_mux_addr == 9'he0 + (9)'(i0) * 9'h4) begin
                readback_data_var[31:0] = field_storage.length[i0].length.value;
            end
        end
        readback_data = readback_data_var;
        readback_done = decoded_req & ~decoded_req_is_wr & ~decoded_req_is_external;
        readback_err = '0;
    end

    assign external_rd_ack = readback_external_rd_ack;
    assign cpuif_rd_ack = readback_done | readback_external_rd_ack;
    assign cpuif_rd_data = readback_data;
    assign cpuif_rd_err = readback_err;
endmodule
// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
// Generated by PeakRDL raw-header

package idma_desc64_addrmap_pkg;

localparam longint unsigned IDMA_DESC64_REG_BASE_ADDR = 64'h0;
localparam longint unsigned IDMA_DESC64_REG_SIZE = 64'h10;

localparam longint unsigned IDMA_DESC64_REG_DESC_ADDR_BASE_ADDR = 64'h0;
localparam longint unsigned IDMA_DESC64_REG_STATUS_BASE_ADDR = 64'h8;


endpackage;
// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
// Generated by PeakRDL raw-header

package idma_reg32_3d_addrmap_pkg;

localparam longint unsigned IDMA_REG_BASE_ADDR = 64'h0;
localparam longint unsigned IDMA_REG_SIZE = 64'hF8;

function automatic longint unsigned IDMA_REG_DIM_BASE_ADDR(input int unsigned dim_idx);
    return 64'hE0 + (dim_idx * 64'hC);
endfunction
localparam longint unsigned IDMA_REG_DIM_NUM = 64'h2;
localparam longint unsigned IDMA_REG_DIM_SIZE = 64'hC;
localparam longint unsigned IDMA_REG_DIM_STRIDE = 64'hC;
localparam longint unsigned IDMA_REG_DIM_TOTAL_SIZE = 64'h18;

localparam longint unsigned IDMA_REG_CONF_BASE_ADDR = 64'h0;
function automatic longint unsigned IDMA_REG_STATUS_BASE_ADDR(input int unsigned status_idx);
    return 64'h4 + (status_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_STATUS_NUM = 64'h10;
function automatic longint unsigned IDMA_REG_NEXT_ID_BASE_ADDR(input int unsigned next_id_idx);
    return 64'h44 + (next_id_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_NEXT_ID_NUM = 64'h10;
function automatic longint unsigned IDMA_REG_DONE_ID_BASE_ADDR(input int unsigned done_id_idx);
    return 64'h84 + (done_id_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_DONE_ID_NUM = 64'h10;
function automatic longint unsigned IDMA_REG_DST_ADDR_BASE_ADDR(input int unsigned dst_addr_idx);
    return 64'hD0 + (dst_addr_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_DST_ADDR_NUM = 64'h1;
function automatic longint unsigned IDMA_REG_SRC_ADDR_BASE_ADDR(input int unsigned src_addr_idx);
    return 64'hD4 + (src_addr_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_SRC_ADDR_NUM = 64'h1;
function automatic longint unsigned IDMA_REG_LENGTH_BASE_ADDR(input int unsigned length_idx);
    return 64'hD8 + (length_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_LENGTH_NUM = 64'h1;
function automatic longint unsigned IDMA_REG_DIM_DST_STRIDE_BASE_ADDR(input int unsigned dim_idx, input int unsigned dst_stride_idx);
    return 64'hE0 + (dim_idx * 64'hC) + (dst_stride_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_DIM_DST_STRIDE_NUM = 64'h1;
function automatic longint unsigned IDMA_REG_DIM_SRC_STRIDE_BASE_ADDR(input int unsigned dim_idx, input int unsigned src_stride_idx);
    return 64'hE4 + (dim_idx * 64'hC) + (src_stride_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_DIM_SRC_STRIDE_NUM = 64'h1;
function automatic longint unsigned IDMA_REG_DIM_REPS_BASE_ADDR(input int unsigned dim_idx, input int unsigned reps_idx);
    return 64'hE8 + (dim_idx * 64'hC) + (reps_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_DIM_REPS_NUM = 64'h1;


endpackage;
// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
// Generated by PeakRDL raw-header

package idma_reg64_2d_addrmap_pkg;

localparam longint unsigned IDMA_REG_BASE_ADDR = 64'h0;
localparam longint unsigned IDMA_REG_SIZE = 64'h118;

function automatic longint unsigned IDMA_REG_DIM_BASE_ADDR(input int unsigned dim_idx);
    return 64'h100 + (dim_idx * 64'h18);
endfunction
localparam longint unsigned IDMA_REG_DIM_NUM = 64'h1;
localparam longint unsigned IDMA_REG_DIM_SIZE = 64'h18;
localparam longint unsigned IDMA_REG_DIM_STRIDE = 64'h18;
localparam longint unsigned IDMA_REG_DIM_TOTAL_SIZE = 64'h18;

localparam longint unsigned IDMA_REG_CONF_BASE_ADDR = 64'h0;
function automatic longint unsigned IDMA_REG_STATUS_BASE_ADDR(input int unsigned status_idx);
    return 64'h4 + (status_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_STATUS_NUM = 64'h10;
function automatic longint unsigned IDMA_REG_NEXT_ID_BASE_ADDR(input int unsigned next_id_idx);
    return 64'h44 + (next_id_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_NEXT_ID_NUM = 64'h10;
function automatic longint unsigned IDMA_REG_DONE_ID_BASE_ADDR(input int unsigned done_id_idx);
    return 64'h84 + (done_id_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_DONE_ID_NUM = 64'h10;
function automatic longint unsigned IDMA_REG_DST_ADDR_BASE_ADDR(input int unsigned dst_addr_idx);
    return 64'hD0 + (dst_addr_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_DST_ADDR_NUM = 64'h2;
function automatic longint unsigned IDMA_REG_SRC_ADDR_BASE_ADDR(input int unsigned src_addr_idx);
    return 64'hD8 + (src_addr_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_SRC_ADDR_NUM = 64'h2;
function automatic longint unsigned IDMA_REG_LENGTH_BASE_ADDR(input int unsigned length_idx);
    return 64'hE0 + (length_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_LENGTH_NUM = 64'h2;
function automatic longint unsigned IDMA_REG_DIM_DST_STRIDE_BASE_ADDR(input int unsigned dim_idx, input int unsigned dst_stride_idx);
    return 64'h100 + (dim_idx * 64'h18) + (dst_stride_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_DIM_DST_STRIDE_NUM = 64'h2;
function automatic longint unsigned IDMA_REG_DIM_SRC_STRIDE_BASE_ADDR(input int unsigned dim_idx, input int unsigned src_stride_idx);
    return 64'h108 + (dim_idx * 64'h18) + (src_stride_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_DIM_SRC_STRIDE_NUM = 64'h2;
function automatic longint unsigned IDMA_REG_DIM_REPS_BASE_ADDR(input int unsigned dim_idx, input int unsigned reps_idx);
    return 64'h110 + (dim_idx * 64'h18) + (reps_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_DIM_REPS_NUM = 64'h2;


endpackage;
// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
// Generated by PeakRDL raw-header

package idma_reg64_1d_addrmap_pkg;

localparam longint unsigned IDMA_REG_BASE_ADDR = 64'h0;
localparam longint unsigned IDMA_REG_SIZE = 64'h118;

localparam longint unsigned IDMA_REG_CONF_BASE_ADDR = 64'h0;
function automatic longint unsigned IDMA_REG_STATUS_BASE_ADDR(input int unsigned status_idx);
    return 64'h4 + (status_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_STATUS_NUM = 64'h10;
function automatic longint unsigned IDMA_REG_NEXT_ID_BASE_ADDR(input int unsigned next_id_idx);
    return 64'h44 + (next_id_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_NEXT_ID_NUM = 64'h10;
function automatic longint unsigned IDMA_REG_DONE_ID_BASE_ADDR(input int unsigned done_id_idx);
    return 64'h84 + (done_id_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_DONE_ID_NUM = 64'h10;
function automatic longint unsigned IDMA_REG_DST_ADDR_BASE_ADDR(input int unsigned dst_addr_idx);
    return 64'hD0 + (dst_addr_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_DST_ADDR_NUM = 64'h2;
function automatic longint unsigned IDMA_REG_SRC_ADDR_BASE_ADDR(input int unsigned src_addr_idx);
    return 64'hD8 + (src_addr_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_SRC_ADDR_NUM = 64'h2;
function automatic longint unsigned IDMA_REG_LENGTH_BASE_ADDR(input int unsigned length_idx);
    return 64'hE0 + (length_idx * 64'h4);
endfunction
localparam longint unsigned IDMA_REG_LENGTH_NUM = 64'h2;


endpackage;

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

// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Michael Rogenmoser <michaero@iis.ee.ethz.ch>
// - Thomas Benz <tbenz@iis.ee.ethz.ch>

`include "obi/typedef.svh"

/// Description: Register-based front-end for iDMA
module idma_reg64_2d #(
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
  localparam int unsigned RegAddrWidth  = idma_reg64_2d_reg_pkg::IDMA_REG64_2D_REG_TOP_MIN_ADDR_WIDTH;

  // register connections
  idma_reg64_2d_reg_pkg::idma_reg__out_t dma_reg2hw [NumRegs-1:0];
  idma_reg64_2d_reg_pkg::idma_reg__in_t  dma_hw2reg [NumRegs-1:0];

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
    idma_reg64_2d_reg_top #(
      .ID_WIDTH ( $bits(dma_ctrl_req_i[i].a.aid) )
    ) i_idma_reg64_2d_reg_top (
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
      arb_payload[i].req.burst_req.length   = {dma_reg2hw[i].length[1].length.value,     dma_reg2hw[i].length[0].length.value};
      arb_payload[i].req.burst_req.src_addr = {dma_reg2hw[i].src_addr[1].src_addr.value, dma_reg2hw[i].src_addr[0].src_addr.value};
      arb_payload[i].req.burst_req.dst_addr = {dma_reg2hw[i].dst_addr[1].dst_addr.value, dma_reg2hw[i].dst_addr[0].dst_addr.value};

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
      arb_payload[i].req.d_req[0].reps = {dma_reg2hw[i].dim[0].reps[1].reps.value,
                                      dma_reg2hw[i].dim[0].reps[0].reps.value };
      arb_payload[i].req.d_req[0].src_strides = {dma_reg2hw[i].dim[0].src_stride[1].src_stride.value,
                                             dma_reg2hw[i].dim[0].src_stride[0].src_stride.value};
      arb_payload[i].req.d_req[0].dst_strides = {dma_reg2hw[i].dim[0].dst_stride[1].dst_stride.value,
                                             dma_reg2hw[i].dim[0].dst_stride[0].dst_stride.value};

      // Disable higher dimensions
      if ( dma_reg2hw[i].conf.enable_nd.value == 0) begin
        arb_payload[i].req.d_req[0].reps = 'd1;
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
      arb_payload[i].req.length   = {dma_reg2hw[i].length[1].length.value,     dma_reg2hw[i].length[0].length.value};
      arb_payload[i].req.src_addr = {dma_reg2hw[i].src_addr[1].src_addr.value, dma_reg2hw[i].src_addr[0].src_addr.value};
      arb_payload[i].req.dst_addr = {dma_reg2hw[i].dst_addr[1].dst_addr.value, dma_reg2hw[i].dst_addr[0].dst_addr.value};

      // Protocols
      arb_payload[i].req.opt.src_protocol = idma_pkg::protocol_e'(dma_reg2hw[i].conf.src_protocol.value);
      arb_payload[i].req.opt.dst_protocol = idma_pkg::protocol_e'(dma_reg2hw[i].conf.dst_protocol.value);

      // Current backend only supports incremental burst
      arb_payload[i].req.opt.src.burst = axi_pkg::BURST_INCR;
      arb_payload[i].req.opt.dst.burst = axi_pkg::BURST_INCR;
        // this frontend currently does not support cache variations
      arb_payload[i].req.opt.src.cache = axi_pkg::CACHE_MODIFIABLE;
      arb_payload[i].req.opt.dst.cache = axi_pkg::CACHE_MODIFIABLE;

      // Backend options
      arb_payload[i].req.opt.beo.decouple_aw    = dma_reg2hw[i].conf.decouple_aw.value;
      arb_payload[i].req.opt.beo.decouple_rw    = dma_reg2hw[i].conf.decouple_rw.value;
      arb_payload[i].req.opt.beo.src_max_llen   = dma_reg2hw[i].conf.src_max_llen.value;
      arb_payload[i].req.opt.beo.dst_max_llen   = dma_reg2hw[i].conf.dst_max_llen.value;
      arb_payload[i].req.opt.beo.src_reduce_len = dma_reg2hw[i].conf.src_reduce_len.value;
      arb_payload[i].req.opt.beo.dst_reduce_len = dma_reg2hw[i].conf.dst_reduce_len.value;

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

