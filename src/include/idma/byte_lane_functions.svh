// Copyright 2026 Mosaic SoC AG
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Include inside a protocol module after its port list. Requires StrbWidth
// (a power of two) and strb_t (one bit per byte lane). No include guard: each
// module needs its own functions, sized using that module's parameters.
typedef logic [$clog2(StrbWidth)-1:0] idma_lane_t;
typedef logic [$clog2(StrbWidth):0] idma_beat_bytes_t;

// Truncation wraps the rotation at the physical bus width, not the AXI size.
function automatic idma_lane_t idma_lane_shift(
    input idma_lane_t from_lane,
    input idma_lane_t to_lane
);
    return from_lane - to_lane;
endfunction

// Select consecutive physical lanes. Prefix and excess bytes are excluded;
// unlike a rotation, this mask must not wrap around the bus word.
function automatic strb_t idma_payload_mask(
    input idma_lane_t first_lane,
    input int unsigned num_bytes
);
    strb_t mask;
    for (int unsigned lane = 0; lane < StrbWidth; lane++) begin
        mask[lane] = lane >= int'(first_lane) &&
                     lane < int'(first_lane) + num_bytes;
    end
    return mask;
endfunction

// An unaligned first beat can use only the bytes up to its selected-size
// boundary. The extra count bit represents a complete physical bus word.
function automatic idma_beat_bytes_t idma_bytes_to_size_boundary(
    input idma_lane_t bus_lane,
    input logic [2:0] size
);
    idma_beat_bytes_t beat_bytes;
    beat_bytes = idma_beat_bytes_t'(1) << size;
    return beat_bytes - (idma_beat_bytes_t'(bus_lane) & (beat_bytes - 1'b1));
endfunction
