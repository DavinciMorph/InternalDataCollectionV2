"""
check_corruption.py — Fast corruption analysis for multi-port ADS1299 CSV files.

Usage:
    python check_corruption.py                          # analyzes all_channels_data.csv
    python check_corruption.py my_recording.csv         # analyzes a specific file
    python check_corruption.py recording1.csv recording2.csv  # multiple files

Expects CSV columns: timestamp, sample_number, Port{1-7}_dev{1-4}_ch{1-8}
(224 data channels from 7 ports × 4 daisy-chained ADS1299 × 8 channels each)

Detects:
    - Single-sample spikes (value jumps > threshold from neighbors)
    - Rail/saturation values (0, +/-8388608)
    - Bit-shift patterns (values that are bit-shifted versions of expected)
    - Startup artifacts (sample 1 anomalies)
    - Out-of-range values (outside expected test signal bounds)
    - Timestamp irregularities (gaps > 2x expected interval)
    - Missing/duplicate sample numbers
"""

import csv
import sys
import os
import time

# --- Configuration ---
SPIKE_THRESHOLD = 500       # min jump from both neighbors to flag as spike
TIMESTAMP_JITTER_MULT = 2.0 # flag intervals > this * expected_dt
RAIL_VALUES = {0, 8388607, -8388608}
BIT_SHIFTS_TO_CHECK = [1, 2, 3, 4]  # check if corrupted = expected >> N
RANGE_LOW = -5500               # test signal expected minimum
RANGE_HIGH = 3000               # test signal expected maximum
MAX_DETAIL_LINES = 10           # max detailed events to print per category


def parse_channel_name(ch_name):
    """Parse 'Port1_dev2_ch3' into (port, device, channel) strings."""
    parts = ch_name.split('_')
    if len(parts) == 3:
        return parts[0], f"{parts[0]}_{parts[1]}", ch_name
    # Fallback for unexpected formats
    return parts[0], ch_name, ch_name


def load_csv(filepath):
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = []
        for row in reader:
            rows.append(row)
    return header, rows


def analyze_file(filepath):
    print(f"\n{'='*70}")
    print(f"  FILE: {os.path.basename(filepath)}")
    print(f"{'='*70}")

    t_start = time.time()
    header, rows = load_csv(filepath)

    n_samples = len(rows)
    if n_samples < 3:
        print("  ERROR: File has fewer than 3 samples, cannot analyze.")
        return

    # Identify columns
    ts_col = header.index('timestamp')
    sn_col = header.index('sample_number')
    data_cols = [i for i in range(len(header)) if i not in (ts_col, sn_col)]
    data_names = [header[i] for i in data_cols]
    n_channels = len(data_cols)

    # Parse data, skipping truncated rows
    expected_cols = len(header)
    good_rows = [i for i in range(n_samples) if len(rows[i]) == expected_cols]
    n_skipped = n_samples - len(good_rows)
    if n_skipped:
        print(f"  WARNING: Skipped {n_skipped} truncated row(s)")
    rows = [rows[i] for i in good_rows]
    n_samples = len(rows)

    timestamps = [float(rows[i][ts_col]) for i in range(n_samples)]
    sample_nums = [int(rows[i][sn_col]) for i in range(n_samples)]
    data = []
    for i in range(n_samples):
        data.append([int(rows[i][c]) for c in data_cols])

    duration = timestamps[-1] - timestamps[0]
    expected_dt = duration / (n_samples - 1) if n_samples > 1 else 0.004

    print(f"\n  --- Overview ---")
    print(f"  Samples:      {n_samples:,}")
    print(f"  Channels:     {n_channels}")
    print(f"  Duration:     {duration:.2f}s ({duration/60:.1f} min)")
    print(f"  Sample rate:  {1/expected_dt:.1f} Hz" if expected_dt > 0 else "  Sample rate:  N/A")
    print(f"  Data points:  {n_samples * n_channels:,}")

    # --- Sample number continuity ---
    sn_gaps = []
    for i in range(1, n_samples):
        diff = sample_nums[i] - sample_nums[i-1]
        if diff != 1:
            sn_gaps.append((i, sample_nums[i-1], sample_nums[i], diff))

    # --- Timestamp analysis ---
    ts_events = []
    dt_list = []
    for i in range(1, n_samples):
        dt = timestamps[i] - timestamps[i-1]
        dt_list.append(dt)
        if dt > expected_dt * TIMESTAMP_JITTER_MULT:
            ts_events.append((i, dt, dt / expected_dt))

    # --- Corruption detection ---
    spike_events = []     # (sample_idx, channel_name, value, expected_approx)
    rail_events = []      # (sample_idx, channel_name, value)
    bitshift_events = []  # (sample_idx, channel_name, value, expected, shift)
    startup_events = []   # (sample_idx, channel_name, value)
    oor_events = []       # (sample_idx, channel_name, value) — out of range

    for ch_idx in range(n_channels):
        ch_name = data_names[ch_idx]

        # Check sample 0 for startup artifacts
        if n_samples > 1:
            v0 = data[0][ch_idx]
            v1 = data[1][ch_idx]
            if abs(v0 - v1) > SPIKE_THRESHOLD:
                startup_events.append((0, ch_name, v0))

        # Scan all samples for spikes and rail values
        for i in range(n_samples):
            val = data[i][ch_idx]

            # Rail check
            if val in RAIL_VALUES:
                rail_events.append((i, ch_name, val))

            # Out-of-range check
            if val < RANGE_LOW or val > RANGE_HIGH:
                oor_events.append((i, ch_name, val))

            # Spike check (need neighbors)
            if 1 <= i < n_samples - 1:
                prev_val = data[i-1][ch_idx]
                next_val = data[i+1][ch_idx]
                expected = (prev_val + next_val) / 2.0

                jump_from_prev = abs(val - prev_val)
                jump_from_next = abs(val - next_val)
                neighbor_diff = abs(next_val - prev_val)

                # Flag if value jumps from BOTH neighbors by more than threshold,
                # and the neighbors themselves are close (ruling out transitions)
                if (jump_from_prev > SPIKE_THRESHOLD and
                    jump_from_next > SPIKE_THRESHOLD and
                    neighbor_diff < SPIKE_THRESHOLD):

                    spike_events.append((i, ch_name, val, expected))

                    # Check if it's a bit-shift of the expected value
                    for shift in BIT_SHIFTS_TO_CHECK:
                        shifted = int(expected) >> shift
                        if abs(val - shifted) < 50:
                            bitshift_events.append((i, ch_name, val, int(expected), shift))
                            break

    # --- Deduplicate spike events by sample ---
    spike_by_sample = {}
    for (idx, ch, val, exp) in spike_events:
        if idx not in spike_by_sample:
            spike_by_sample[idx] = []
        spike_by_sample[idx].append((ch, val, exp))

    bitshift_by_sample = {}
    for (idx, ch, val, exp, shift) in bitshift_events:
        if idx not in bitshift_by_sample:
            bitshift_by_sample[idx] = []
        bitshift_by_sample[idx].append((ch, val, exp, shift))

    oor_by_sample = {}
    for (idx, ch, val) in oor_events:
        if idx not in oor_by_sample:
            oor_by_sample[idx] = []
        oor_by_sample[idx].append((ch, val))

    # --- Timestamp stats ---
    if dt_list:
        dt_mean = sum(dt_list) / len(dt_list)
        dt_sorted = sorted(dt_list)
        dt_min = dt_sorted[0]
        dt_max = dt_sorted[-1]
        dt_median = dt_sorted[len(dt_sorted)//2]
        pct_normal = sum(1 for d in dt_list if abs(d - expected_dt) < expected_dt * 0.5) / len(dt_list) * 100
    else:
        dt_mean = dt_min = dt_max = dt_median = 0
        pct_normal = 0

    # --- Print results ---
    print(f"\n  --- Timing ---")
    print(f"  Expected dt:  {expected_dt*1000:.3f} ms")
    print(f"  Mean dt:      {dt_mean*1000:.3f} ms")
    print(f"  Min dt:       {dt_min*1000:.3f} ms")
    print(f"  Max dt:       {dt_max*1000:.3f} ms")
    print(f"  Normal (+-50%): {pct_normal:.2f}%")
    print(f"  Jitter events (>{TIMESTAMP_JITTER_MULT}x): {len(ts_events)}")
    if ts_events:
        for idx, dt, mult in ts_events[:5]:
            print(f"    Sample {sample_nums[idx]:>7}: {dt*1000:.1f} ms ({mult:.1f}x)")
        if len(ts_events) > 5:
            print(f"    ... and {len(ts_events)-5} more")

    print(f"\n  --- Sample Numbers ---")
    print(f"  Range:  {sample_nums[0]} to {sample_nums[-1]}")
    print(f"  Gaps:   {len(sn_gaps)}")
    if sn_gaps:
        for idx, prev, curr, diff in sn_gaps[:5]:
            print(f"    Row {idx}: {prev} -> {curr} (jump={diff})")
        if len(sn_gaps) > 5:
            print(f"    ... and {len(sn_gaps)-5} more")

    print(f"\n  --- Corruption ---")

    if startup_events:
        ports_affected = set(parse_channel_name(ch)[0] for (_, ch, _) in startup_events)
        print(f"  Startup artifacts:  {len(startup_events)} channels ({', '.join(sorted(ports_affected))})")
    else:
        print(f"  Startup artifacts:  0")

    print(f"  Rail values:        {len(rail_events)}")
    if rail_events:
        # Group rail events by port
        rail_by_port = {}
        for idx, ch, val in rail_events:
            port = parse_channel_name(ch)[0]
            rail_by_port[port] = rail_by_port.get(port, 0) + 1
        for port in sorted(rail_by_port.keys()):
            print(f"    {port}: {rail_by_port[port]}")
        for idx, ch, val in rail_events[:MAX_DETAIL_LINES]:
            print(f"    Sample {sample_nums[idx]:>7}: {ch} = {val}")
        if len(rail_events) > MAX_DETAIL_LINES:
            print(f"    ... and {len(rail_events)-MAX_DETAIL_LINES} more")

    n_spikes = len(spike_by_sample)
    total_spike_channels = len(spike_events)
    print(f"  Spike events:       {n_spikes} samples, {total_spike_channels} channel-events")
    if spike_by_sample:
        for idx in sorted(spike_by_sample.keys())[:MAX_DETAIL_LINES]:
            channels = spike_by_sample[idx]
            ts = timestamps[idx]
            sn = sample_nums[idx]
            # Summarize by port for readability with 224 channels
            port_summary = {}
            for ch, val, exp in channels:
                port = parse_channel_name(ch)[0]
                port_summary[port] = port_summary.get(port, 0) + 1
            port_str = ', '.join(f"{p}({n}ch)" for p, n in sorted(port_summary.items()))
            print(f"    Sample {sn:>7} (t={ts:.3f}s): {len(channels)} channels [{port_str}]")

            # Show bit-shift info if applicable
            if idx in bitshift_by_sample:
                bs_summary = {}
                for ch, val, exp, shift in bitshift_by_sample[idx]:
                    port = parse_channel_name(ch)[0]
                    bs_summary[port] = bs_summary.get(port, 0) + 1
                bs_str = ', '.join(f"{p}({n})" for p, n in sorted(bs_summary.items()))
                print(f"      ^ bit-shift matches: [{bs_str}]")

        if n_spikes > MAX_DETAIL_LINES:
            print(f"    ... and {n_spikes-MAX_DETAIL_LINES} more events")

    n_bitshift = len(bitshift_by_sample)
    print(f"  Bit-shift matches:  {n_bitshift}")

    n_oor_samples = len(oor_by_sample)
    print(f"  Out-of-range:       {len(oor_events)} values in {n_oor_samples} samples (range [{RANGE_LOW}, {RANGE_HIGH}])")
    if oor_by_sample:
        for idx in sorted(oor_by_sample.keys())[:MAX_DETAIL_LINES]:
            channels = oor_by_sample[idx]
            ts = timestamps[idx]
            sn = sample_nums[idx]
            port_summary = {}
            for ch, val in channels:
                port = parse_channel_name(ch)[0]
                port_summary[port] = port_summary.get(port, 0) + 1
            port_str = ', '.join(f"{p}({n}ch)" for p, n in sorted(port_summary.items()))
            print(f"    Sample {sn:>7} (t={ts:.3f}s): {len(channels)} channels [{port_str}]")
        if n_oor_samples > MAX_DETAIL_LINES:
            print(f"    ... and {n_oor_samples-MAX_DETAIL_LINES} more events")

    # --- Summary ---
    total_corrupted = total_spike_channels + len(rail_events) + len(oor_events)
    total_points = n_samples * n_channels
    corruption_rate = (total_corrupted / total_points * 100) if total_points > 0 else 0

    print(f"\n  --- Summary ---")
    if total_corrupted == 0 and len(sn_gaps) == 0:
        print(f"  CLEAN - Zero corruption events detected")
    else:
        print(f"  Corrupted values:   {total_corrupted:,} / {total_points:,} ({corruption_rate:.6f}%)")
        corrupt_samples = set(spike_by_sample.keys()) | set(oor_by_sample.keys())
        print(f"  Corrupt samples:    {len(corrupt_samples)} / {n_samples:,}")

        # Per-port breakdown
        port_counts = {}
        dev_counts = {}
        for (idx, ch, val, exp) in spike_events:
            port, dev, _ = parse_channel_name(ch)
            port_counts[port] = port_counts.get(port, 0) + 1
            dev_counts[dev] = dev_counts.get(dev, 0) + 1
        for (idx, ch, val) in oor_events:
            port, dev, _ = parse_channel_name(ch)
            port_counts[port] = port_counts.get(port, 0) + 1
            dev_counts[dev] = dev_counts.get(dev, 0) + 1
        if port_counts:
            print(f"  By port:")
            for port in sorted(port_counts.keys()):
                print(f"    {port}: {port_counts[port]} corrupted values")
                # Show device breakdown within this port
                port_devs = {d: c for d, c in sorted(dev_counts.items()) if d.startswith(port)}
                if len(port_devs) > 1 or any(c != port_counts[port] for c in port_devs.values()):
                    for dev in sorted(port_devs.keys()):
                        print(f"      {dev}: {port_devs[dev]}")

    elapsed = time.time() - t_start
    print(f"\n  Analysis time: {elapsed:.1f}s")


def main():
    if len(sys.argv) > 1:
        files = sys.argv[1:]
    else:
        default = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'all_channels_data.csv')
        if os.path.exists(default):
            files = [default]
        else:
            print("Usage: python check_corruption.py <csv_file> [csv_file2 ...]")
            print("  Or place all_channels_data.csv in the data/ directory.")
            sys.exit(1)

    for f in files:
        if not os.path.exists(f):
            print(f"ERROR: File not found: {f}")
            continue
        analyze_file(f)

    print()


if __name__ == '__main__':
    main()
