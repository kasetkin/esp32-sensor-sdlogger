#!/usr/bin/env python3
"""Anonymize GPS coordinates in NMEA capture logs.

Replaces the real latitude/longitude in GNRMC, GNGGA and Unicore #PPPNAVA
messages with anonymized values and recomputes each sentence's checksum so the
firmware parser (modules/TinyGPSPlus) still accepts them.

Two-layer transform (see docs / plan):
  1. A big *fixed* per-file offset translates the whole track to open ocean
     (OCEAN_TARGET, Northern/Eastern hemisphere so the N/E indicator letters
     never flip). The offset is derived at runtime from the file's own first
     coordinate and is NOT persisted anywhere.
  2. A per-epoch *random-walk* offset is added on top: each epoch's offset =
     previous epoch's offset + a small Gaussian step (default 0.0002 deg ~ 22 m,
     reflected to stay within +/-MAX_DRIFT of the base). Because the walk
     integrates the steps, the cumulative drift (~ step * sqrt(N)) bends the
     course progressively and warps the global track shape so it can no longer
     be map-matched against a road graph. The same offset is applied to every
     position message within one epoch, so each epoch stays internally
     consistent (GNRMC == GNGGA, and #PPPNAVA agrees to its natural precision).

Checksum algorithms mirror the parser byte-for-byte:
  * $-sentences: 2-hex uppercase XOR of bytes between '$' and '*'.
  * #-sentences: 8-hex lowercase CRC-32 (poly 0xEDB88320, init=0, no final XOR)
    over bytes between '#' and '*' (commas and ';' included).

Usage:
    python3 anonymize_nmea.py FILE [FILE ...] [--step 0.0002] [--max-drift 2.0] [--seed N]
    python3 anonymize_nmea.py --validate-only FILE [FILE ...]
"""

from __future__ import annotations

import argparse
import random
import re
import sys
from typing import Optional

# Open North Pacific, far from any land; stays in the N/E hemisphere.
OCEAN_TARGET_LAT = 30.0
OCEAN_TARGET_LON = 150.0
DEFAULT_STEP_DEG = 0.0002  # per-epoch random-walk RMS step (~22 m)
DEFAULT_MAX_DRIFT_DEG = 2.0  # walk stays within +/-2 deg of base (open ocean)


# ── checksum primitives ──────────────────────────────────────────────────────

def _build_crc_table() -> list[int]:
    table = []
    for n in range(256):
        c = n
        for _ in range(8):
            c = (0xEDB88320 ^ (c >> 1)) if (c & 1) else (c >> 1)
        table.append(c & 0xFFFFFFFF)
    return table


_CRC_TABLE = _build_crc_table()
assert _CRC_TABLE[1] == 0x77073096 and _CRC_TABLE[2] == 0xEE0E612C, "bad CRC table"


def nmea_xor(payload: str) -> str:
    p = 0
    for b in payload.encode("ascii"):
        p ^= b
    return f"{p:02X}"


def unicore_crc32(payload: str) -> str:
    c = 0
    for b in payload.encode("ascii"):
        c = _CRC_TABLE[(c ^ b) & 0xFF] ^ (c >> 8)
    return f"{c:08x}"


def compute_checksum(start_char: str, payload: str) -> str:
    return unicore_crc32(payload) if start_char == "#" else nmea_xor(payload)


# ── line / sentence decomposition ────────────────────────────────────────────

def split_eol(line: str) -> tuple[str, str]:
    for eol in ("\r\n", "\n", "\r"):
        if line.endswith(eol):
            return line[: -len(eol)], eol
    return line, ""


class Sentence:
    """A parsed NMEA line, preserving everything needed to round-trip it."""

    __slots__ = ("prefix", "start", "payload", "csum", "rest", "eol")

    def __init__(self, line: str) -> "Sentence | None":
        content, self.eol = split_eol(line)
        si = next((i for i, ch in enumerate(content) if ch in "$#"), None)
        if si is None:
            raise ValueError("no sentence start")
        self.start = content[si]
        star = content.rfind("*")
        n = 8 if self.start == "#" else 2
        if star <= si or len(content) < star + 1 + n:
            raise ValueError("no/short checksum")
        self.prefix = content[:si]
        self.payload = content[si + 1 : star]
        self.csum = content[star + 1 : star + 1 + n]
        self.rest = content[star + 1 + n :]

    def render(self) -> str:
        c = compute_checksum(self.start, self.payload)
        return f"{self.prefix}{self.start}{self.payload}*{c}{self.rest}{self.eol}"


def parse(line: str) -> Optional[Sentence]:
    try:
        return Sentence(line)
    except ValueError:
        return None


# ── coordinate conversion ────────────────────────────────────────────────────

def dm_to_deg(field: str, deg_digits: int) -> tuple[float, int]:
    """'1230.00000000' -> (12.5, frac_digit_count). deg_digits unused but
    documents lat(2)/lon(3); minutes are always the last 2 integer digits."""
    intpart, _, frac = field.partition(".")
    deg = int(intpart[:-2]) if len(intpart) > 2 else 0
    minutes = float(intpart[-2:] + "." + frac) if frac else float(intpart[-2:])
    return deg + minutes / 60.0, len(frac)


def deg_to_dm(value: float, deg_digits: int, frac_digits: int) -> str:
    deg = int(value)
    rem = round((value - deg) * 60.0, frac_digits)
    if rem >= 60.0:
        deg += 1
        rem -= 60.0
    min_str = f"{rem:0{3 + frac_digits}.{frac_digits}f}"  # 2 int digits + '.' + frac
    return f"{deg:0{deg_digits}d}{min_str}"


def fmt_decimal(value: float, frac_digits: int) -> str:
    return f"{value:.{frac_digits}f}"


# ── source-location redaction (for corrupted / merged frames) ─────────────────
# Corrupted serial frames can splice a real lat/lon fragment into a sentence the
# structured parser does not recognise (e.g. a #PPPNAVA tail fused onto a $..GSV
# header). To catch such fragments without hardcoding any real location, the
# integer lat/lon degrees a capture actually used are derived from its own valid
# sentences (analyze_source) and turned into the patterns below. The leading
# (?<![\d.]) guard prevents matching a degree embedded inside a longer (e.g.
# already-anonymised) number.

def source_coord_regex(lat_degs: set[int], lon_degs: set[int]) -> "re.Pattern[str]":
    """Build the redaction regex for the given source lat/lon integer degrees,
    matching both NMEA DDMM(.mmmm) and decimal-degree coordinate fragments."""
    parts: list[str] = []
    for d in sorted(lat_degs):
        parts.append(rf"{d:02d}\d\d\.\d{{4,}}")  # DDMM latitude
        parts.append(rf"{d}\.\d{{5,}}")          # decimal latitude
    for n in sorted(lon_degs):
        parts.append(rf"0?{n}\d\d\.\d{{4,}}")    # DDDMM longitude (optional 0 pad)
        parts.append(rf"{n}\.\d{{5,}}")          # decimal longitude
    return re.compile(r"(?<![\d.])(?:" + "|".join(parts) + r")")


def make_redactor(lat_degs: set[int], lon_degs: set[int], dlat: float, dlon: float):
    """Return a re.sub callback that shifts any matched source-location fragment
    by the file's base offset (no jitter — these live in already-broken lines).
    A fragment is latitude iff its integer degree is one the capture used for
    latitude; otherwise it is longitude."""

    def redact(m: "re.Match[str]") -> str:
        tok = m.group(0)
        intpart, _, frac = tok.partition(".")
        deg = int(intpart[:-2]) if len(intpart) >= 4 else int(intpart)  # DDMM vs decimal
        is_lat = deg in lat_degs
        if len(intpart) >= 4:  # DDMM / DDDMM
            deg_digits = 2 if is_lat else 3
            val, fd = dm_to_deg(tok, deg_digits)
            return deg_to_dm(val + (dlat if is_lat else dlon), deg_digits, fd)
        return fmt_decimal(float(tok) + (dlat if is_lat else dlon), len(frac))

    return redact


# ── per-message coordinate access ────────────────────────────────────────────

def message_type(start: str, payload: str) -> str:
    if start == "#" and payload.startswith("PPPNAVA"):
        return "PPPNAVA"
    head = payload.split(",", 1)[0]
    if start == "$" and len(head) >= 3 and head[-3:] in ("RMC", "GGA"):
        return head[-3:]
    return ""


def _gxx_indices(mtype: str) -> tuple[int, int, int, int]:
    # lat, N/S, lon, E/W   (field indices within the comma-split payload)
    return (3, 4, 5, 6) if mtype == "RMC" else (2, 3, 4, 5)


def read_position(start: str, payload: str) -> Optional[tuple[float, float]]:
    """Decimal (lat, lon) for a position-bearing sentence, or None when the
    message carries no valid fix (empty or all-zero coordinates)."""
    mtype = message_type(start, payload)
    if mtype in ("RMC", "GGA"):
        f = payload.split(",")
        lat_i, _, lon_i, _ = _gxx_indices(mtype)
        if max(lat_i, lon_i) >= len(f) or not f[lat_i] or not f[lon_i]:
            return None
        lat = dm_to_deg(f[lat_i], 2)[0]
        lon = dm_to_deg(f[lon_i], 3)[0]
        return (lat, lon) if (lat or lon) else None
    if mtype == "PPPNAVA":
        head, sep, body = payload.partition(";")
        if not sep:
            return None
        b = body.split(",")
        if len(b) < 4 or not b[2] or not b[3]:
            return None
        lat, lon = float(b[2]), float(b[3])
        return (lat, lon) if (lat or lon) else None
    return None


def timestamp(start: str, payload: str) -> Optional[str]:
    mtype = message_type(start, payload)
    if mtype in ("RMC", "GGA"):
        f = payload.split(",")
        return f[1] if len(f) > 1 else None
    return None


def shift_payload(start: str, payload: str, dlat: float, dlon: float) -> str:
    """Return payload with lat/lon shifted by (dlat, dlon) decimal degrees."""
    mtype = message_type(start, payload)
    if mtype in ("RMC", "GGA"):
        f = payload.split(",")
        lat_i, ns_i, lon_i, ew_i = _gxx_indices(mtype)
        assert f[ns_i] == "N" and f[ew_i] == "E", f"unexpected hemisphere in {mtype}"
        lat, lat_fd = dm_to_deg(f[lat_i], 2)
        lon, lon_fd = dm_to_deg(f[lon_i], 3)
        nlat, nlon = lat + dlat, lon + dlon
        assert 0.0 <= nlat < 90.0 and 0.0 <= nlon < 180.0, "shift left N/E hemisphere"
        f[lat_i] = deg_to_dm(nlat, 2, lat_fd)
        f[lon_i] = deg_to_dm(nlon, 3, lon_fd)
        return ",".join(f)
    if mtype == "PPPNAVA":
        head, _, body = payload.partition(";")
        b = body.split(",")
        nlat = float(b[2]) + dlat
        nlon = float(b[3]) + dlon
        assert 0.0 <= nlat < 90.0 and 0.0 <= nlon < 180.0, "shift left N/E hemisphere"
        b[2] = fmt_decimal(nlat, len(b[2].partition(".")[2]))
        b[3] = fmt_decimal(nlon, len(b[3].partition(".")[2]))
        return head + ";" + ",".join(b)
    return payload


# ── file passes ──────────────────────────────────────────────────────────────

def validate_file(path: str) -> list[tuple[int, str, str, str]]:
    """Return list of (line_no, start, expected, found) checksum mismatches."""
    bad = []
    with open(path, "r", newline="", encoding="ascii", errors="surrogateescape") as fh:
        for n, line in enumerate(fh, 1):
            s = parse(line)
            if s is None:
                continue
            expected = compute_checksum(s.start, s.payload)
            if expected.lower() != s.csum.lower():
                bad.append((n, s.start, expected, s.csum))
    return bad


def privacy_scan(path: str, coord_re: "re.Pattern[str]") -> list[tuple[int, str]]:
    """Return (line_no, fragment) for every residual source-location signature.
    `coord_re` must be the SOURCE regex (from the originals) — the output no longer
    contains the source coordinates to derive it from."""
    hits = []
    with open(path, "r", newline="", encoding="ascii", errors="surrogateescape") as fh:
        for n, line in enumerate(fh, 1):
            for m in coord_re.finditer(line):
                hits.append((n, m.group(0)))
    return hits


def audit_positions(path: str) -> tuple[int, list[int], int]:
    """Return (#position sentences, [line_nos of position sentences whose checksum
    is invalid], #corrupt sentences overall). Every anonymized fix must keep a
    valid checksum so the firmware parser still accepts it."""
    n_pos = 0
    bad_pos: list[int] = []
    n_corrupt = 0
    with open(path, "r", newline="", encoding="ascii", errors="surrogateescape") as fh:
        for i, line in enumerate(fh, 1):
            s = parse(line)
            if s is None:
                continue
            ok = compute_checksum(s.start, s.payload).lower() == s.csum.lower()
            n_corrupt += not ok
            if read_position(s.start, s.payload) is not None:
                n_pos += 1
                if not ok:
                    bad_pos.append(i)
    return n_pos, bad_pos, n_corrupt


def rejoin_frames(raw_lines: list[str]) -> list[tuple[str, str]]:
    """Rejoin frames split by an errant mid-sentence newline.

    The UM980 logs occasionally insert a newline inside a sentence, e.g.
        $GNGGA,...,N,07  <newline>  012.84503391,E,...,144*78
    A physical line that does not start with '$'/'#' is a continuation, so it is
    appended back onto the previous logical line (restoring the original frame,
    whose trailing checksum then validates again). Clean streams are unaffected
    because every well-formed sentence already starts with '$'/'#'."""
    logical: list[list[str]] = []  # [content, eol]
    for phys in raw_lines:
        content, eol = split_eol(phys)
        if content[:1] in ("$", "#") or not logical:
            logical.append([content, eol])
        else:
            logical[-1][0] += content
            logical[-1][1] = eol
    return [(c, e) for c, e in logical]


def analyze_source(contents: list[str]) -> tuple[float, float, set[int], set[int]]:
    """One pass over the valid sentences to learn the capture's own geography:
    the base offset (from the first fix, anchoring it to OCEAN_TARGET) and the
    sets of integer lat/lon degrees it used (to target the corrupt-frame redactor
    — nothing about the real location is hardcoded)."""
    base: Optional[tuple[float, float]] = None
    lat_degs: set[int] = set()
    lon_degs: set[int] = set()
    for content in contents:
        s = parse(content)
        if s is None:
            continue
        pos = read_position(s.start, s.payload)
        if pos is None:
            continue
        lat, lon = pos
        lat_degs.add(int(lat))
        lon_degs.add(int(lon))
        if base is None:
            base = (OCEAN_TARGET_LAT - lat, OCEAN_TARGET_LON - lon)
    if base is None:
        raise SystemExit("no valid coordinate found to anchor the offset")
    return base[0], base[1], lat_degs, lon_degs


def _reflect(x: float, bound: float) -> float:
    """Fold x into [-bound, bound] by reflecting at the boundaries (triangle
    wave, period 4*bound). Keeps the random walk inside the safe ocean region."""
    if bound <= 0.0:
        return x
    period = 4.0 * bound
    y = (x + bound) % period
    if y > 2.0 * bound:
        y = period - y
    return y - bound


def transform_file(path: str, step: float, max_drift: float, rng: random.Random) -> dict:
    with open(path, "r", newline="", encoding="ascii", errors="surrogateescape") as fh:
        logical = rejoin_frames(fh.readlines())

    base_dlat, base_dlon, lat_degs, lon_degs = analyze_source([c for c, _ in logical])
    coord_re = source_coord_regex(lat_degs, lon_degs)
    redact = make_redactor(lat_degs, lon_degs, base_dlat, base_dlon)

    edited = epochs = redacted = 0
    cur_ts: Optional[str] = None
    jlat = jlon = 0.0  # random-walk offset, accumulated across epochs

    out: list[str] = []
    for content, eol in logical:
        s = parse(content)
        if s is None:
            # not a sentence (blank / leading partial): scrub any stray fragment.
            new_content = coord_re.sub(redact, content)
            redacted += new_content != content
            out.append(new_content + eol)
            continue

        s.eol = eol  # render() re-attaches the logical line's terminator
        ts = timestamp(s.start, s.payload)
        if ts is not None and ts != cur_ts:
            cur_ts = ts
            # advance the random walk by one small Gaussian step per epoch
            jlat = _reflect(jlat + rng.gauss(0.0, step), max_drift)
            jlon = _reflect(jlon + rng.gauss(0.0, step), max_drift)
            epochs += 1

        orig_valid = compute_checksum(s.start, s.payload).lower() == s.csum.lower()

        if orig_valid and read_position(s.start, s.payload) is not None:
            # well-formed position fix: structured shift + recomputed checksum
            s.payload = shift_payload(s.start, s.payload, base_dlat + jlat, base_dlon + jlon)
            out.append(s.render())
            edited += 1
        elif orig_valid:
            # well-formed non-position sentence (GSV/GSA/...): keep valid, but
            # scrub any stray coordinate fragment (defensive; normally a no-op).
            new_payload = coord_re.sub(redact, s.payload)
            redacted += new_payload != s.payload
            s.payload = new_payload
            out.append(s.render())
        else:
            # pre-existing corruption (bad checksum already → parser rejects it):
            # leave it rejected, but redact any spliced-in real coordinate.
            new_content = coord_re.sub(redact, content)
            redacted += new_content != content
            out.append(new_content + eol)

    with open(path, "w", newline="", encoding="ascii", errors="surrogateescape") as fh:
        fh.writelines(out)

    return {"edited": edited, "epochs": epochs, "redacted": redacted,
            "coord_re": coord_re, "lat_degs": lat_degs, "lon_degs": lon_degs}


# ── cli ──────────────────────────────────────────────────────────────────────

def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("files", nargs="+")
    ap.add_argument("--step", type=float, default=DEFAULT_STEP_DEG,
                    help="per-epoch random-walk RMS step in degrees (default 0.0002)")
    ap.add_argument("--max-drift", type=float, default=DEFAULT_MAX_DRIFT_DEG,
                    help="walk stays within +/- this many degrees of base (default 2.0)")
    ap.add_argument("--seed", type=int, default=None, help="RNG seed (default: random)")
    ap.add_argument("--validate-only", action="store_true")
    args = ap.parse_args(argv)

    # Step 1: prove the checksum implementation reproduces every original
    # checksum. The curated example files match 100%; the raw CSV logs carry a
    # few pre-existing corrupt frames (merged GSV frames, $devicename echoes).
    print("== self-validation (original files) ==")
    for path in args.files:
        bad = validate_file(path)
        print(f"  {path}: {len(bad)} pre-existing corrupt line(s)")
        for n, start, exp, found in bad[:3]:
            print(f"      line {n} [{start}] expected {exp} found {found}")
    if args.validate_only:
        return 0

    rng = random.Random(args.seed)

    print("\n== transform (in place) ==")
    source_re: dict[str, "re.Pattern[str]"] = {}
    for path in args.files:
        stats = transform_file(path, args.step, args.max_drift, rng)
        source_re[path] = stats["coord_re"]  # source signature, for the leak scan
        print(f"  {path}: {stats['edited']} coords shifted / {stats['redacted']} "
              f"fragment(s) redacted across {stats['epochs']} epoch(s) "
              f"[lat°{sorted(stats['lat_degs'])} lon°{sorted(stats['lon_degs'])}]")

    print("\n== post-validation (rewritten files) ==")
    failed = False
    for path in args.files:
        leaks = privacy_scan(path, source_re[path])
        n_pos, bad_pos, n_corrupt = audit_positions(path)
        if bad_pos:
            failed = True
            print(f"  {path}: INVALID FIX CHECKSUM at line(s) {bad_pos[:5]}")
        elif leaks:
            failed = True
            print(f"  {path}: PRIVACY LEAK — {len(leaks)} source fragment(s) remain, "
                  f"e.g. line {leaks[0][0]}: {leaks[0][1]}")
        else:
            print(f"  {path}: OK — {n_pos} position fixes all valid, "
                  f"{n_corrupt} corrupt frame(s) preserved, 0 source fragments remain")
    if failed:
        print("\nERROR: post-validation failed.")
        return 1

    print("\nOK: coordinates anonymized, every fix checksum valid, no leaks.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
