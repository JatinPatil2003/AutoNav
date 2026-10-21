// Safely under typical MTU once RTP/UDP/IP headers are added — NAL units
// at or below this go out as a single RTP packet; larger ones are FU-A
// fragmented (RFC 6184 §5.8).
const MAX_PACKET_PAYLOAD_SIZE = 1200;
const FU_A_NAL_TYPE = 28;

export interface PacketizedPayload {
  payload: Buffer;
  marker: boolean;
}

/** VCL (slice) NAL unit types — 1 (non-IDR slice) through 5 (IDR slice). */
export function isVclNalUnit(nal: Buffer): boolean {
  if (nal.length === 0) return false;
  const nalType = nal[0] & 0x1f;
  return nalType >= 1 && nalType <= 5;
}

/**
 * Packetizes one NAL unit into one or more RTP payloads per RFC 6184: a
 * single-NAL-unit packet if it fits under MAX_PACKET_PAYLOAD_SIZE,
 * otherwise FU-A fragments. The marker bit (end of access unit) is only
 * ever set on the LAST fragment of a VCL NAL unit — callers that assume
 * one slice NAL per access unit (true for libx264's baseline/ultrafast
 * output in practice) can treat marker=true as "frame complete, flush
 * now." Pure function, no I/O, so it's directly unit-testable.
 */
export function packetizeNalUnit(nal: Buffer): PacketizedPayload[] {
  if (nal.length === 0) {
    return [];
  }

  const isVcl = isVclNalUnit(nal);

  if (nal.length <= MAX_PACKET_PAYLOAD_SIZE) {
    return [{ payload: nal, marker: isVcl }];
  }

  const nalHeaderByte = nal[0];
  const forbiddenAndNri = nalHeaderByte & 0xe0;
  const nalType = nalHeaderByte & 0x1f;
  const payload = nal.subarray(1);
  const fuIndicator = forbiddenAndNri | FU_A_NAL_TYPE;
  const maxChunkSize = MAX_PACKET_PAYLOAD_SIZE - 2; // room for the FU indicator + FU header bytes

  const fragments: PacketizedPayload[] = [];
  let offset = 0;
  while (offset < payload.length) {
    const isFirst = offset === 0;
    const chunkSize = Math.min(maxChunkSize, payload.length - offset);
    const isLast = offset + chunkSize >= payload.length;
    const fuHeader = (isFirst ? 0x80 : 0) | (isLast ? 0x40 : 0) | nalType;
    const rtpPayload = Buffer.concat([Buffer.from([fuIndicator, fuHeader]), payload.subarray(offset, offset + chunkSize)]);
    fragments.push({ payload: rtpPayload, marker: isVcl && isLast });
    offset += chunkSize;
  }
  return fragments;
}
