import csv
import sys

def analyze(filepath, label):
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    timestamps = [float(r['timestamp']) for r in rows]
    sample_nums = [int(r['sample_number']) for r in rows]
    values = [int(r['ch1_value']) for r in rows]

    print(f"\n=== {label} ===")
    print(f"  Total rows: {len(rows)}")
    print(f"  Sample number range: {sample_nums[0]} to {sample_nums[-1]}")
    print(f"  Timestamp range: {timestamps[0]:.4f} to {timestamps[-1]:.4f} seconds")
    print(f"  Duration: {timestamps[-1] - timestamps[0]:.2f} seconds")

    # Timestamp intervals
    dts = [timestamps[i] - timestamps[i-1] for i in range(1, len(timestamps))]
    nonzero_dts = [d for d in dts if d > 0]
    zero_dts = [d for d in dts if d == 0]

    print(f"\n  Timestamp deltas:")
    print(f"    Total intervals: {len(dts)}")
    print(f"    Zero-dt intervals: {len(zero_dts)} ({100*len(zero_dts)/len(dts):.1f}%)")
    print(f"    Non-zero-dt intervals: {len(nonzero_dts)}")
    if nonzero_dts:
        print(f"    Avg non-zero dt: {sum(nonzero_dts)/len(nonzero_dts):.6f} s")
        print(f"    Min non-zero dt: {min(nonzero_dts):.6f} s")
        print(f"    Max non-zero dt: {max(nonzero_dts):.6f} s")
    if dts:
        avg_dt = sum(dts)/len(dts)
        print(f"    Avg dt (overall): {avg_dt:.6f} s")
        if avg_dt > 0:
            print(f"    Effective sample rate: {1/avg_dt:.2f} Hz")

    # Sample number gaps
    sample_diffs = [sample_nums[i] - sample_nums[i-1] for i in range(1, len(sample_nums))]
    gaps = [d for d in sample_diffs if d != 1]
    print(f"\n  Sample number continuity:")
    print(f"    Consecutive (diff=1): {sum(1 for d in sample_diffs if d==1)}")
    print(f"    Gaps (diff!=1): {len(gaps)}")
    if gaps:
        print(f"    Gap values (first 10): {gaps[:10]}")

    # Value distribution
    unique_vals = set(values)
    val_at_min = values.count(-8388608)
    val_at_max = values.count(8388607)
    other_vals = len(values) - val_at_min - val_at_max
    print(f"\n  Value distribution:")
    print(f"    Unique values: {len(unique_vals)}")
    print(f"    Count at -8388608 (min 24-bit): {val_at_min} ({100*val_at_min/len(values):.1f}%)")
    print(f"    Count at  8388607 (max 24-bit): {val_at_max} ({100*val_at_max/len(values):.1f}%)")
    print(f"    Other values: {other_vals} ({100*other_vals/len(values):.1f}%)")

    # Check if samples per timestamp batch is consistent (for CLIENT)
    if len(zero_dts) > len(dts) * 0.1:  # If >10% are zero
        batch_sizes = []
        current_batch = 1
        for d in dts:
            if d == 0:
                current_batch += 1
            else:
                batch_sizes.append(current_batch)
                current_batch = 1
        batch_sizes.append(current_batch)
        avg_batch = sum(batch_sizes)/len(batch_sizes)
        print(f"\n  Batching pattern (timestamps shared across samples):")
        print(f"    Number of batches: {len(batch_sizes)}")
        print(f"    Avg batch size: {avg_batch:.1f}")
        print(f"    Min batch size: {min(batch_sizes)}")
        print(f"    Max batch size: {max(batch_sizes)}")

base = r"C:\Users\cadav\Desktop\InternalDataCollection\CodeTesting"
analyze(f"{base}\\ch1_data.csv", "ch1_data.csv (server/direct)")
analyze(f"{base}\\CLIENTCh1.csv", "CLIENTCh1.csv (client)")
