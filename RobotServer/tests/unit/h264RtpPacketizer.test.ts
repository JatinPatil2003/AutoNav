import { isVclNalUnit, packetizeNalUnit } from "../../src/modules/video/h264RtpPacketizer";

const NAL_TYPE_SPS = 7;
const NAL_TYPE_PPS = 8;
const NAL_TYPE_IDR_SLICE = 5;
const NAL_TYPE_NON_IDR_SLICE = 1;
const NRI_HIGH = 0x60; // nal_ref_idc bits set, forbidden_zero_bit clear

function makeNal(nalType: number, payloadLength: number, nri = NRI_HIGH): Buffer {
  const nal = Buffer.alloc(1 + payloadLength);
  nal[0] = nri | nalType;
  for (let i = 0; i < payloadLength; i += 1) {
    nal[1 + i] = (i + 1) & 0xff; // deterministic, non-zero-heavy filler
  }
  return nal;
}

describe("isVclNalUnit", () => {
  it("is false for an empty buffer", () => {
    expect(isVclNalUnit(Buffer.alloc(0))).toBe(false);
  });

  it("is true for slice NAL types 1 through 5", () => {
    for (let type = 1; type <= 5; type += 1) {
      expect(isVclNalUnit(makeNal(type, 4))).toBe(true);
    }
  });

  it("is false for SPS/PPS (non-VCL types)", () => {
    expect(isVclNalUnit(makeNal(NAL_TYPE_SPS, 4))).toBe(false);
    expect(isVclNalUnit(makeNal(NAL_TYPE_PPS, 4))).toBe(false);
  });
});

describe("packetizeNalUnit", () => {
  it("returns a single packet for a NAL under the fragmentation threshold, unchanged", () => {
    const nal = makeNal(NAL_TYPE_IDR_SLICE, 100);

    const packets = packetizeNalUnit(nal);

    expect(packets).toHaveLength(1);
    expect(packets[0].payload).toEqual(nal);
  });

  it("sets marker=true for a single-packet VCL NAL, false for non-VCL", () => {
    expect(packetizeNalUnit(makeNal(NAL_TYPE_IDR_SLICE, 10))[0].marker).toBe(true);
    expect(packetizeNalUnit(makeNal(NAL_TYPE_SPS, 10))[0].marker).toBe(false);
  });

  it("returns nothing for an empty NAL", () => {
    expect(packetizeNalUnit(Buffer.alloc(0))).toEqual([]);
  });

  it("fragments a large VCL NAL into FU-A packets with correct S/E bits and preserves every payload byte", () => {
    const nalType = NAL_TYPE_NON_IDR_SLICE;
    const nri = NRI_HIGH;
    const nal = makeNal(nalType, 3000); // comfortably over the 1200-byte threshold

    const packets = packetizeNalUnit(nal);

    expect(packets.length).toBeGreaterThan(1);

    // Every fragment: FU indicator byte (nri | type=28), FU header byte (S/E/R + original type).
    const FU_A_TYPE = 28;
    packets.forEach((packet, index) => {
      const fuIndicator = packet.payload[0];
      const fuHeader = packet.payload[1];
      expect(fuIndicator & 0xe0).toBe(nri);
      expect(fuIndicator & 0x1f).toBe(FU_A_TYPE);
      expect(fuHeader & 0x1f).toBe(nalType);

      const isFirst = index === 0;
      const isLast = index === packets.length - 1;
      expect(!!(fuHeader & 0x80)).toBe(isFirst); // S bit
      expect(!!(fuHeader & 0x40)).toBe(isLast); // E bit
      expect(packet.marker).toBe(isLast); // VCL NAL -> marker only on the final fragment
    });

    // Reassemble: original NAL header byte + every fragment's payload (stripping the 2 FU bytes) must equal the source NAL exactly.
    const reassembledPayload = Buffer.concat(packets.map((p) => p.payload.subarray(2)));
    const reassembled = Buffer.concat([Buffer.from([nal[0]]), reassembledPayload]);
    expect(reassembled).toEqual(nal);
  });

  it("never sets marker on FU-A fragments of a non-VCL NAL, even the last one", () => {
    const nal = makeNal(NAL_TYPE_SPS, 3000);

    const packets = packetizeNalUnit(nal);

    expect(packets.length).toBeGreaterThan(1);
    expect(packets.every((p) => !p.marker)).toBe(true);
  });
});
