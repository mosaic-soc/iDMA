---
title: Backend
description: The backend executes 1D transfers over concrete transport protocols.
---

## Backend Role

The backend is the lowest layer of the iDMA pipeline. It takes 1D transfer requests from the midend and drives the actual bus transactions. Each backend variant targets a specific combination of read and write protocols. Generated modules follow the naming pattern `idma_backend_<variant>` (e.g., `idma_backend_rw_axi`, `idma_backend_r_obi_w_axi`).

![Backend Architecture](/iDMA/fig/backend.svg)

### Transfer Lifecycle

When a 1D transfer request arrives at the backend, it flows through three stages. First, the **legalizer** splits the request into protocol-legal bus bursts, respecting page boundaries, maximum burst lengths, and alignment constraints. Each burst carries its payload byte count, resolved size, burst mode, physical offset, and starting FIFO lane. Second, the **transport layer** executes each burst: the read channel fetches data from the source, barrel shifters realign byte lanes, the data buffer absorbs timing differences, and the write channel stores data at the destination. Third, the **error handler** (if enabled) monitors bus responses and reports faults to software. The backend signals completion through `idma_rsp_t` once all bursts of a transfer have finished, including write responses.

## Runtime AXI Sizes

`backend_options_t` provides independent source and destination size controls:

| Field | Meaning |
|-------|---------|
| `src_size_enable`, `dst_size_enable` | Apply the corresponding size field when set; otherwise use the physical bus width and ignore that field |
| `src_size`, `dst_size` | AXI log2(bytes per beat): 0 = 1 byte, 1 = 2 bytes, through 7 = 128 bytes; must not exceed `log2(DataWidth / 8)` when enabled |

Zero-initialized options preserve existing full-width accesses. Sizes and enables are
sampled with each request; queued transfers may use different settings. `length`
always counts **payload bytes**, not beats. Source and destination sizes may differ,
and lengths need not be multiples of either size.

Enabled size controls require `HardwareLegalizer=1`, `CombinedShifter=0`, and compute
disabled **for that request**. The backend may still elaborate compute hardware for
other requests. Endpoint capabilities are declared separately by
`supports_runtime_read_size` and `supports_runtime_write_size` in `src/db/idma_*.yml`;
currently only AXI advertises support. A different endpoint whose size control is
disabled retains its existing full-width behavior. Unsupported combinations are
asserted, not silently converted to another size.

### Alignment and Partial Beats

- INCR permits unaligned byte addresses. The legalizer aligns the bus address down to
  the selected size, discards a read prefix, and suppresses prefix write strobes.
- FIXED requires alignment to the **selected size**, not the physical bus width. Its
  bus address stays constant while the logical payload stream continues to advance.
- A partial final read still accesses the complete selected-size word. Excess bytes
  do not enter the dataflow buffer. A partial final write retains AWSIZE and selects
  only the remaining payload bytes with WSTRB.

Software must account for read-prefix/final excess accesses and peripheral side
effects. A six-byte transfer with four-byte FIXED reads performs two four-byte
accesses, even though only six bytes are forwarded. Partial write strobes must also
be supported by the destination.

### Byte-Stream Packing

For ordinary copies with separate shifters, logical payload byte `j` uses FIFO lane
`j mod W`, where `W = DataWidth / 8`. Read and write FIFO positions advance independently
by the payload bytes accepted on each R/W handshake. They continue across legalized
bursts, including FIXED bursts, and restart at lane zero for each new transfer.
Physical bus lanes instead follow each endpoint's address, size, and burst mode.

For example, on a 64-bit bus, read six payload bytes using four-byte FIXED accesses
at `0x1004`, and write them using two-byte INCR accesses at `0x2002`:

| Access | Physical byte lanes | Payload |
|--------|---------------------|---------|
| R at `0x1004` | 4-7 | A, B, C, D enter FIFO lanes 0-3 |
| R at `0x1004` again | 4-7 | E, F enter FIFO lanes 4-5; the other two read bytes are discarded |
| W at `0x2002` | 2-3, WSTRB `0x0C` | A, B |
| W at `0x2004` | 4-5, WSTRB `0x30` | C, D |
| W at `0x2006` | 6-7, WSTRB `0xC0` | E, F |

This is backend request behavior; the current [register frontend](../frontend/register/)
exposes runtime sizes but selects INCR bursts on both endpoints.

## Parameters

The most important parameters for a new integration are `DataWidth` (match your bus width), `BufferDepth` (use 3 unless area-constrained), and `HardwareLegalizer` (use 1 unless your software pre-splits bursts). The remaining parameters tune throughput and area - see the [parameter presets](../guides/system-integration/#4-set-parameters) in the System Integration guide for recommended combinations.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DataWidth` | 16 | Data bus width in bits. Must be a power of 2 in {16, 32, 64, 128, 256, 512, 1024} |
| `AddrWidth` | 24 | Address width in bits. Must be >= 12 |
| `UserWidth` | 1 | AXI user signal width. Must be > 0 |
| `AxiIdWidth` | 1 | AXI ID width. Must be > 0 |
| `NumAxInFlight` | 2 | Number of concurrent in-flight transactions. Must be > 1 |
| `BufferDepth` | 2 | Depth of the internal reorder buffer. 2 = minimal, **3 = recommended** because depth-2 buffers stall on misaligned transfers where read and write offsets differ, requiring an extra buffer slot for the alignment pipeline |
| `TFLenWidth` | 24 | Transfer length width. Max transfer size is `2^TFLenWidth` bytes. Must be >= 12 and <= AddrWidth |
| `MemSysDepth` | 0 | Depth of the attached memory system (additional pipeline stages) |
| `CombinedShifter` | 0 | Use a single barrel shifter instead of two (saves area, data no longer word-aligned in buffer) |
| `RAWCouplingAvail` | 1 | Enable R-AW coupling hardware. Should be 1 for pure AXI-to-AXI variants (`rw_axi`); set to 0 for mixed-protocol variants where the write protocol has no AW channel |
| `MaskInvalidData` | 1 | Zero out invalid bytes on the manager interface to reduce toggling |
| `HardwareLegalizer` | 1 | Include hardware burst legalization. If 0, software must ensure legal bursts |
| `RejectZeroTransfers` | 1 | Reject zero-length transfers with a `BACKEND` error response |
| `ErrorCap` | `NO_ERROR_HANDLING` | Error handling capability: `NO_ERROR_HANDLING` or `ERROR_HANDLING` |
| `PrintFifoInfo` | 0 | Print FIFO configuration during elaboration |
| `TimingCuts` | `'0` | Opt-in timing cuts (`idma_pkg::timing_cuts_t`), see [Timing cuts](#timing-cuts). `'0` is the stock datapath |

The maximum number of transfers in-flight at any point is:

```
MetaFifoDepth = BufferDepth + NumAxInFlight + MemSysDepth
```

This determines how many 1D bursts can be in-flight simultaneously - `BufferDepth` entries in the data buffer, `NumAxInFlight` transactions on the bus, and `MemSysDepth` stages in the external memory system.

### Timing cuts

`TimingCuts` shortens the paths from the write-side handshake to the read side, which end at the
dataflow element pointers, `r_ready` and the request FIFOs. Each field is independent:

| Field | Effect | Cost |
|-------|--------|------|
| `dfe_ready_cut` | A full dataflow lane takes a new beat only in a cycle without a write-side pop. `r_ready` and the dataflow push no longer depend on the write datapath. Implies `dfe_reg_flags` | One more dataflow entry per byte lane (`BufferDepth + 1`), one more MX input queue entry per engine |
| `mx_beat_push` | An MX beat enters its engine queue on the read beat of any read port (its response valid and the read response side ready) and the queue's ready, without the byte-lane masks and the masked buffer-ready reduction of the read port: MX beats are whole beats (asserted). The MX push no longer depends on the read datapath request offsets, the read shift or the last beat flag. Cycle-identical | None |
| `dfe_ready_ahead` | The dataflow element's ready comes from its registered lane flags and from the lanes the next write beat pops: each write port reports them from the request head and its burst counters (`buffer_out_offer_o`), without the W handshake; a transpose reports its pending pop; an MX burst none. A lane takes its one extra entry only in such a cycle, so a full element refills in the cycle the write side starts to drain it and `r_ready` no longer depends on the write-side handshake. The MX input queues keep their depth. Combines with `dfe_ready_cut` (one entry each) | One more dataflow entry per byte lane |
| `mx_in_reg` | MX beats enter their engine through a one-beat register stage whose data and tag load in every cycle it can take a beat, so the read handshake and data paths end at the stage instead of the engine input queues | One cycle more from R to W for MX beats; the stage takes one input queue entry of each engine |
| `dfe_reg_flags` | Dataflow lanes keep full/empty in flops, update their pointers without load enables and write the free slot every cycle the lane can accept. Cycle-identical | A few flops per lane |
| `wdp_head_spill` | A spill register on the head of the write datapath request FIFO, so the write datapath starts from flops instead of the FIFO read mux | +1 cycle from the legalizer to the write datapath; `MetaFifoDepth` +2 |
| `outst_cnt_reg` | The outstanding-transfer counter of `RejectZeroTransfers` counts an accepted request one cycle late, so it does not depend on `req_ready_o`. Cycle-identical | One flop |

`MetaFifoDepth` grows by each extra dataflow entry and by two with `wdp_head_spill`.

## Interface

### Port Groups

All backends have the Request, Response, Bus Read, Bus Write, and Busy port groups. The Error Handler ports are only present when `ErrorCap = ERROR_HANDLING`.

| Group | Signals | Direction | Description |
|-------|---------|-----------|-------------|
| **Request** | `idma_req_i`, `req_valid_i`, `req_ready_o` | in/in/out | 1D transfer request (valid/ready handshake) |
| **Response** | `idma_rsp_o`, `rsp_valid_o`, `rsp_ready_i` | out/out/in | Transfer completion response |
| **Error Handler** | `idma_eh_req_i`, `eh_req_valid_i`, `eh_req_ready_o` | in/in/out | Error handling action (CONTINUE/ABORT) |
| **Bus Read** | `<proto>_read_req_o`, `<proto>_read_rsp_i` | out/in | Read channel to memory system |
| **Bus Write** | `<proto>_write_req_o`, `<proto>_write_rsp_i` | out/in | Write channel to memory system |
| **Busy** | `busy_o` | out | Per-subunit busy flags (`idma_busy_t`) |

### Busy Signal (`idma_busy_t`)

Software can poll the busy signal to check if the backend is idle before clock-gating, resetting, or reconfiguring it. Each flag corresponds to a specific subunit - if a transfer stalls, the stuck flag identifies the bottleneck:

```verilog
typedef struct packed {
    logic buffer_busy;       // Data buffer contains valid data
    logic r_dp_busy;         // Read datapath active
    logic w_dp_busy;         // Write datapath active
    logic r_leg_busy;        // Read legalizer processing
    logic w_leg_busy;        // Write legalizer processing
    logic eh_fsm_busy;       // Error handler FSM not idle
    logic eh_cnt_busy;       // Outstanding transfer counter != 0
    logic raw_coupler_busy;  // R-AW coupler holds pending AWs
} idma_busy_t;
```

## Variant Matrix

Each backend variant combines a set of read and write protocols:

| Variant ID | Read Protocol | Write Protocol |
|------------|---------------|----------------|
| `rw_axi` | AXI4 | AXI4 |
| `r_obi_w_axi` | OBI | AXI4 |
| `r_axi_w_obi` | AXI4 | OBI |
| `rw_axi_rw_axis` | AXI4 | AXI4 + AXI Stream |
| `rw_obi` | OBI | OBI |
| `r_obi_rw_init_w_axi` | OBI | INIT + AXI4 |
| `r_axi_rw_init_rw_obi` | AXI4 | INIT + OBI |
| `rw_axi_rw_init_rw_obi` | AXI4 | AXI4 + INIT + OBI |

The tracked set is `IDMA_TREE_IDS` in `idma.mk`; extend it with `IDMA_ADD_IDS` for custom protocol combinations. Add-id variants are generated by `make idma_hw_all idma_add_all IDMA_ADD_IDS="..."` into `target/rtl/idma_generated_add.sv` and reach Bender through the `add_ids` target.

Each variant also gets its own tracer header, `idma/tracer_<id>.svh`, holding the `IDMA_TRACER_<ID>` macro; `idma/tracer.svh` carries only the id-independent helpers and is included by the per-id headers.

![Variant Matrix](/iDMA/fig/variant_matrix.svg)

## Legalizer

The legalizer decomposes a 1D transfer request into a sequence of protocol-legal bus bursts. Separate read and write state machines track each side's remaining bytes and current address. Read and write burst lengths are chosen independently. With compute, the write side keeps up to three accepted requests in a queue (`MxWqDepth`): while the write side still emits the bursts of an MX transfer, the read side takes the next decoupled request as soon as it has emitted its own bursts. The read side (dequant planes) and the write side (quant planes) keep separate MX plane state. While no MX transfer is on the write side, a request waits until both sides are done and the queue is empty, as without compute.

The legalizer is pure control path: it does not touch the data. It computes page/burst boundaries and emits the metadata needed to execute and realign each burst; the datapath derives masks and rotations from that metadata.

![Legalizer](/iDMA/fig/legalizer.svg)

### Splitting Rules

| Protocol | Burst Mode | Page Size | Max Beats |
|----------|-----------|-----------|-----------|
| AXI INCR | `split_at_page_boundary` | 4096 B | 256 |
| AXI FIXED | Repeated selected-size access | No page split from payload progress | 16 |
| TileLink | `only_pow2` | 2048 B | Power-of-2 sized |
| OBI | `not_supported` (single-beat) | StrbWidth | 1 |
| INIT | `not_supported` (single-beat) | StrbWidth | 1 |
| AXI Stream | `not_supported` (single-beat) | StrbWidth | 1 |

AXI INCR bursts respect the 4 KiB boundary and 256-beat limit; FIXED bursts repeat one size-aligned access and are limited to 16 beats. User limits apply independently: when `src_reduce_len` or `dst_reduce_len` is set, that endpoint is limited to `2^max_llen` beats, further restricted by its protocol limit. At size `s`, the byte capacity is `max_beats << s`, less the first-beat prefix for an unaligned INCR request. Narrower sizes therefore reduce bytes per burst for the same LLEN setting. Read and write splits need not coincide.

TileLink uses power-of-2 aligned bursts with a 2048 B page size (limited by the TLToAXI4 bridge for AXI compliance); in TLToAXI4 compatibility mode, write bursts are further limited to 32 beats and never cross page boundaries. For non-bursting protocols (OBI, INIT, AXI Stream), each burst is a single bus-width beat.

If `HardwareLegalizer=1` and software submits a transfer crossing a page boundary, the legalizer splits it automatically. With `HardwareLegalizer=0`, such a transfer would violate the protocol and cause undefined bus behavior.

**Example**: A 5000-byte AXI INCR endpoint using 16-byte beats, starting at `0xFF8`, without user LLEN reduction or safe-mode clamping. The legalizer emits bursts carrying 8 bytes at logical address `0xFF8`, 4096 bytes at `0x1000`, and 896 bytes at `0x2000`. The first AXI address is aligned down to `0xFF0`; its eight prefix bytes are discarded or excluded by write strobes. Each burst stays within one 4 KiB page and respects the 256-beat limit.

### FIXED Bursts

A FIXED burst accesses the **same address on every beat**. It is useful for a
peripheral that supplies or accepts a stream of data through one address.
Source and destination modes can be selected separately:

| Source | Destination | Result |
|--------|-------------|--------|
| FIXED | INCR | Read repeatedly from one address and write to consecutive addresses |
| INCR | FIXED | Read from consecutive addresses and write repeatedly to one address |
| FIXED | FIXED | Read and write repeatedly at the two given addresses |

Set `request.opt.src.burst` and `request.opt.dst.burst` to
`axi_pkg::BURST_FIXED` or `axi_pkg::BURST_INCR`. Only AXI currently supports FIXED
in the backend. Use INCR for other protocols. WRAP is not supported; the hardware
legalizer asserts on an unsupported burst mode.

**Set both burst fields explicitly.** Their zero value is FIXED, not INCR. A
zero-initialized request therefore does not describe a normal sequential copy.
The register frontend defaults to INCR and exposes FIXED through `conf.src_fixed`
and `conf.dst_fixed`. The instruction frontend and RT midend explicitly select
INCR. The descriptor frontend exposes both burst fields to software.

For FIXED accesses, align the address to the selected beat size, which defaults
to `DataWidth / 8` bytes. `length` is the number of bytes to transfer, not
the number of times to access the address. A final partial write uses byte strobes
to leave the other bytes untouched. A final partial read still reads a whole selected-size
beat and discards the unused bytes. Check that this is suitable for the peripheral.

For example, on a 64-bit bus with size controls disabled, a 24-byte FIXED-to-INCR transfer reads three times
from `0x1000` and writes eight bytes each at `0x2000`, `0x2008`, and `0x2010`.
Ordinary memory returns the same eight bytes on each read. A peripheral may return
new data each time. With a FIXED destination in ordinary memory, later writes
overwrite earlier ones; only the last value of each written byte remains.

A transfer longer than 16 beats is split into several FIXED bursts, all starting
at the original address. With no smaller user or safe-mode limit, 40 beats become
bursts of 16, 16, and 8 beats.
The address does not advance when a burst finishes. Payload length alone does not
cause a page split, since every beat accesses the same word.

User burst limits still apply. If `src_reduce_len` or `dst_reduce_len` is set,
that side uses at most `2^max_llen` beats per burst, capped at 16 for FIXED.
For example, `src_max_llen=2` limits FIXED read bursts to four beats. Safe mode
may further limit read bursts and waits for enough free buffer space before
issuing them. FIXED does not by itself prevent deadlocks; use `deadlock_free`
when the memory system needs it.

The generator reads `supports_fixed_bursts` and `max_beats_per_fixed_burst` from
each protocol's YAML file in `src/db/`. These settings control the generated
protocol checks and FIXED burst limit.

### Deadlock-Free Reservations

With `deadlock_free=1`, read admission reserves space for payload bytes in each byte-lane
FIFO. For a burst of `N` bytes starting at FIFO lane `P`, lane `i` reserves:

```text
floor(N / W) + (((i - P) mod W) < (N mod W))
```

Discarded read prefix and final excess bytes consume no entries. Reservations include
outstanding read responses and buffered data; credits return when bytes actually leave
the dataflow buffer, not merely when R responses arrive. Requests with safe mode off
are still accounted for, so switching modes between queued requests does not lose usage.
Admission uses registered usage; a current-cycle pop makes its credit available next
cycle, avoiding a combinational WREADY-to-ARVALID dependency.

The legalizer conservatively caps each safe read burst at
`(BufferDepth - 1) * W` bytes **before subtracting the first-beat prefix**. The spare
entry per lane accommodates residual bytes of a partially assembled write beat.
This is a byte limit, not a fixed beat count: smaller ARSIZE can use more beats while
still respecting protocol and user LLEN limits. The reservation gate also checks
existing per-lane usage before admitting a burst. Safe mode does not require R/W
burst coupling and cannot guarantee progress from an external slave that never responds.

### Datapath Control Signals

Datapath requests carry `num_bytes` (logical payload length), `size` (resolved log2
bytes per beat), `burst`, `offset` (initial position within the physical bus word),
and `fifo_start_lane` (FIFO position of the first payload byte). `is_single` remains
for protocol consumers that need it. These fields travel through decoupling FIFOs;
active bursts never depend on a later frontend request. AXI derives each beat's
mask, rotation, and completion from its current positions and remaining payload.
The former request fields `tailer`, `shift`, and `num_beats` are no longer needed.

### Software Legalization

When `HardwareLegalizer=0`, the legalizer is bypassed and replaced with a simple `stream_fork` that synchronizes the read and write paths. In this mode, software is responsible for ensuring all transfers are already legal for the target protocol (e.g., no AXI page-boundary crossings). Use this only when software pre-splits all transfers into protocol-legal bursts (e.g., an RTOS DMA driver that already handles AXI page boundaries). This saves ~1-2K gates but moves burst-splitting responsibility to the driver.

`deadlock_free` has no effect in this configuration. Software burst splitting
alone does not provide the read-buffer checks described below.

For FIXED, software must also keep each burst within the 16-beat limit and keep
the address unchanged when splitting a longer transfer.

Runtime-size enables are unsupported in this mode and must remain zero. The
hardware legalizer's safe-mode burst clamp is absent as well.

## Transport Layer

### Architecture

The transport layer is responsible for moving data from source to destination, handling the byte-lane realignment that arises when source and destination addresses have different bus-word offsets. It contains the read channel, byte-granular data buffer, and write channel. Data flows as: **read port** -> **read barrel shifter** -> **dataflow element (buffer)** -> **write barrel shifter** -> **write port**. With compute, MX beats go from the read barrel shifter to the input queue of their engine and do not enter the dataflow element (see [Compute](../compute/)).

The buffer (`idma_dataflow_element`) is an array of independent FIFOs, one per byte lane (`StrbWidth` = `DataWidth / 8`, i.e., the number of byte lanes; `StrbWidth` FIFOs of depth `BufferDepth`). This byte-granular design allows data to enter and leave the buffer at arbitrary byte-lane positions, enabling misaligned transfers without additional alignment stages.

### Deadlock-Free Mode

A read can get stuck if its data cannot fit in the DMA buffer while writes are
waiting on the same memory system. Limiting burst length is not enough: several
small read bursts can still fill the buffer together.

Set `request.opt.beo.deadlock_free = 1` to make the backend check buffer space
before issuing each read burst. The option is sampled with each transfer and is
off by default. It requires `HardwareLegalizer=1`.

The mode has two parts:

1. **Shorten read bursts.** The legalizer caps each burst at
   `(BufferDepth - 1) * StrbWidth` bytes before subtracting any first-beat prefix.
   The spare entry per lane allows partially assembled write beats to make progress.
   Smaller user limits, protocol limits, and page boundaries still apply. Safe mode
   changes the read limit only; the write limit and R-AW coupling stay unchanged.
2. **Reserve space before reading.** A gate between the read-address FIFO and the
   bus read module counts space already promised to earlier reads. A new read
   is allowed only if every byte-lane FIFO has enough room for its complete
   payload. Outstanding responses count as used space, even before they arrive.

The gate counts payload bytes, including the starting lane of a misaligned read.
Read bytes discarded at the beginning or end of a transfer do not use buffer
entries. Space becomes available when bytes leave the dataflow buffer, including
when a compute engine takes them, not merely when read data arrives.

For example, with eight byte lanes and depth 3, a 16-byte aligned read reserves
two entries in every lane. A second 16-byte read must wait until each lane has
at most one entry still reserved. Space freed in a cycle can be used by the gate
on the next cycle. This avoids a combinational path from write readiness back
to the read-address handshake.

Requests with the mode off skip the space check but are still counted. This lets
queued requests use different mode settings without losing track of earlier
reads. It does **not** make those earlier requests safe or recover a transfer
that is already stuck. Enable the mode on all transfers that may need it.

R-AW coupling does not replace this space check. Safe mode
addresses this DMA read-buffer problem; it is not a timeout or a general cure
for deadlocks elsewhere in the system. Memory and response consumers must still
make progress. Smaller bursts and waiting for space can reduce throughput, so
leave the mode off only when the surrounding system already guarantees progress.

#### Choosing a Read Burst Limit

The largest allowed burst is not always the fastest choice. Another equally sized
burst must wait until enough space is free in every lane, which can leave a gap
while new read data arrives. The spare entry prevents a completely full-buffer
reservation, but does not necessarily hide read latency.

As a starting point, try a read limit of about **half the buffer depth**, rounded
down to a power of two and at least one beat:

| `BufferDepth` | Default maximum full-width beats | Suggested starting limit | `src_max_llen` |
|---------------|-----------------------|--------------------------|----------------|
| 2 | 1 | 1 | 0 |
| 3 | 2 | 1 | 0 |
| 8 | 7 | 4 | 2 |
| 16 | 15 | 8 | 3 |

The table assumes aligned full-width beats and no smaller protocol limit. Narrower
AXI sizes can fit more beats in the same byte capacity.

Set `src_reduce_len=1` to apply the smaller limit; safe mode remains enabled.
For example, with depth 8:

```systemverilog
request.opt.beo.deadlock_free = 1'b1;
request.opt.beo.src_reduce_len = 1'b1;
request.opt.beo.src_max_llen = 3'd2; // At most four beats per read burst.
```

With four-beat reads, another read can start once four entries per lane are free,
while writes can still use the remaining data. Multiple smaller reads may be
outstanding, up to the configured request limit; the gate still checks their
combined space use. This can hide some read latency instead of waiting for the
entire buffer to drain.

Smaller bursts also mean more address transactions and may reduce throughput.
Compare the default and smaller limits on the target memory system; half depth
is a tuning starting point, not a guaranteed optimum. Changing the read limit
does not change the write limit. No hardware default is changed by this recommendation.

### Data Realignment

:::note[Figure placeholder]
Diagram: data realignment through read shifter, buffer, and write shifter.
Show an example with misaligned source/destination offsets and byte lane rotation.
:::

Two barrel shifters handle the address offset difference between source and destination. The **read shifter** aligns incoming data based on the source address offset; the **write shifter** rotates data to match the destination address offset.

When `CombinedShifter=1`, both shifts are folded into a single operation before the buffer. This halves the shifter area but means data inside the buffer is no longer word-aligned. The tradeoff is area (single shifter) vs. timing (data alignment happens earlier in the pipeline).

### On-the-Fly Compute

When `EnableCompute=1`, an optional compute stage (`idma_otf_compute`) is inserted in the write datapath, between the data buffer and the write port. It transforms the beat stream in flight - tiled transpose or OCP microscaling (MX) quantize/dequantize - substituting the write-side data, per-lane valid, and byte strobe when a transfer arms an op. Compute is only elaboratable on backends whose read and write sides both use AXI or OBI, and requires `ErrorCap=NO_ERROR_HANDLING` and `CombinedShifter=0`. See the [Compute](../compute/) page for the operations, parameters, and legalizer constraints.

## Channel Coupler

The R-AW channel coupler (`idma_channel_coupler`) holds back AW requests until the first corresponding R beat arrives. Without coupling, the DMA could issue a write address before the read data arrives, which wastes write-side resources and can increase interconnect pressure - particularly problematic in shared-bus fabrics. With coupling enabled, the write address is only sent once data is available, preventing write-before-read ordering hazards. Controlled by `RAWCouplingAvail` (enables the hardware) and `decouple_aw` (per-transfer opt-in via `backend_options_t`). Only available for AXI-to-AXI variants.

Despite the name, `decouple_aw` actually *enables* R-AW coupling (holding AW until R data arrives). The name refers to the backend option struct field (`beo.decouple_aw`), where setting it to 1 activates the coupling logic.

**When coupling helps**: On a shared AXI bus, an uncoupled DMA issues AW immediately, occupying a write-side slot before data is available. With coupling, AW waits for the first R beat, ensuring the write port is only claimed when data is ready to flow. **When to disable**: If the read and write ports go to different memory controllers (no shared resources), coupling adds unnecessary latency.

## Error Handler

The error handler monitors R and W datapath responses for non-OKAY AXI responses. When an error is detected, it reports the faulting burst address and error type to software and waits for a CONTINUE or ABORT action. See the [Error Handling guide](../guides/error-handling/) for the full FSM description and software handling patterns.

## Source Files

- **Backend template**: `src/backend/tpl/idma_backend.sv.tpl`
- **Legalizer template**: `src/backend/tpl/idma_legalizer.sv.tpl`
- **Transport layer template**: `src/backend/tpl/idma_transport_layer.sv.tpl`
- **Error handler**: `src/backend/idma_error_handler.sv`
- **Channel coupler**: `src/backend/idma_channel_coupler.sv`
- **Dataflow element**: `src/backend/idma_dataflow_element.sv`
- **Read reservation gate**: `src/backend/idma_read_reservation_gate.sv`
- **Generated output**: `target/rtl/idma_backend_*.sv`
