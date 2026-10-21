import { AnnexBParser } from "../../src/modules/video/annexBParser";

function startCode3(): Buffer {
  return Buffer.from([0x00, 0x00, 0x01]);
}

function startCode4(): Buffer {
  return Buffer.from([0x00, 0x00, 0x00, 0x01]);
}

describe("AnnexBParser", () => {
  it("extracts nothing from a single NAL with no following start code yet", () => {
    const parser = new AnnexBParser();
    const result = parser.feed(Buffer.concat([startCode3(), Buffer.from([0x67, 0x01, 0x02])]));

    expect(result).toEqual([]);
  });

  it("extracts one complete NAL once a second start code arrives", () => {
    const parser = new AnnexBParser();
    const nalA = Buffer.from([0x67, 0x01, 0x02]);
    const nalB = Buffer.from([0x68, 0x03]);

    const result = parser.feed(Buffer.concat([startCode3(), nalA, startCode3(), nalB]));

    expect(result).toEqual([nalA]);
  });

  it("handles 4-byte start codes the same as 3-byte ones", () => {
    const parser = new AnnexBParser();
    const nalA = Buffer.from([0x67, 0x01, 0x02]);
    const nalB = Buffer.from([0x68, 0x03]);

    const result = parser.feed(Buffer.concat([startCode4(), nalA, startCode4(), nalB]));

    expect(result).toEqual([nalA]);
  });

  it("splits one chunk containing three NAL units (e.g. SPS, PPS, slice)", () => {
    const parser = new AnnexBParser();
    const sps = Buffer.from([0x67, 0xaa]);
    const pps = Buffer.from([0x68, 0xbb]);
    const slice = Buffer.from([0x65, 0xcc, 0xdd]);

    const result = parser.feed(
      Buffer.concat([startCode4(), sps, startCode4(), pps, startCode4(), slice, startCode4()]),
    );

    expect(result).toEqual([sps, pps, slice]);
  });

  it("completes a NAL unit split across two feed() calls", () => {
    const parser = new AnnexBParser();
    const nalA = Buffer.from([0x67, 0x01, 0x02, 0x03]);
    const nalB = Buffer.from([0x68, 0x04]);

    expect(parser.feed(Buffer.concat([startCode3(), nalA.subarray(0, 2)]))).toEqual([]);
    const result = parser.feed(Buffer.concat([nalA.subarray(2), startCode3(), nalB]));

    expect(result).toEqual([nalA]);
  });

  it("trims trailing zero padding bytes that precede the next start code", () => {
    const parser = new AnnexBParser();
    // nalA's real payload ends at 0x02; the two zero bytes after it are
    // padding that happens to belong to the following 4-byte start code's
    // leading zeros once combined with the minimal 3-byte "00 00 01" match.
    const paddedNalA = Buffer.from([0x67, 0x01, 0x02, 0x00, 0x00]);
    const nalB = Buffer.from([0x68, 0x03]);

    const result = parser.feed(Buffer.concat([startCode3(), paddedNalA, startCode4(), nalB]));

    expect(result).toEqual([Buffer.from([0x67, 0x01, 0x02])]);
  });
});
