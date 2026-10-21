/**
 * Splits a stream of Annex-B H264 bytes (ffmpeg's `-f h264` output) into
 * individual NAL units, across arbitrarily-chunked `feed()` calls. Pure
 * byte-buffer logic, no ffmpeg/werift dependency, so it's directly
 * unit-testable without spawning anything.
 */
export class AnnexBParser {
  private buffer = Buffer.alloc(0);

  /**
   * Feeds a chunk of bytes, returning every COMPLETE NAL unit found so
   * far. A trailing incomplete NAL (after the last start code seen) is
   * kept internally and completed by a later feed() call.
   *
   * Start codes are detected by their minimal 3-byte suffix (00 00 01),
   * which matches both the 3-byte and 4-byte (00 00 00 01) forms at the
   * same trailing position, so both resolve to the same NAL boundary.
   * Trailing zero bytes immediately before the next start code are
   * trimmed as alignment padding, not NAL payload: H264's own emulation-
   * prevention scheme guarantees a real NAL's RBSP never contains an
   * unescaped run of 2+ zero bytes outside an actual start code, so any
   * such run found here can only be padding.
   */
  feed(chunk: Buffer): Buffer[] {
    this.buffer = Buffer.concat([this.buffer, chunk]);

    const starts: number[] = [];
    for (let i = 0; i + 2 < this.buffer.length; i += 1) {
      if (this.buffer[i] === 0 && this.buffer[i + 1] === 0 && this.buffer[i + 2] === 1) {
        starts.push(i);
      }
    }
    if (starts.length < 2) {
      return [];
    }

    const nalUnits: Buffer[] = [];
    for (let i = 0; i < starts.length - 1; i += 1) {
      const nalStart = starts[i] + 3;
      let nalEnd = starts[i + 1];
      while (nalEnd > nalStart && this.buffer[nalEnd - 1] === 0) {
        nalEnd -= 1;
      }
      if (nalEnd > nalStart) {
        nalUnits.push(this.buffer.subarray(nalStart, nalEnd));
      }
    }

    this.buffer = this.buffer.subarray(starts[starts.length - 1]);
    return nalUnits;
  }
}
