import { MagCalibrationData, Point } from "./math"

function makePacketView(id: number, size: number) {
  console.log(`sending packet with id 0x${id.toString(16)}`)

  const buffer = new ArrayBuffer(1 + size)
  const view = new DataView(buffer)

  view.setUint8(0, id)

  const packetView = new DataView(buffer, 1)

  return [buffer, packetView] as const
}

export function buildStartMagCalibrationPacket() {
  const [buffer, _] = makePacketView(0x11, 0)
  return buffer
}

export function buildMagCalibrationStopPacket() {
  const [buffer, _] = makePacketView(0x10, 0)
  return buffer
}

export function getPacketData(packet: ArrayBuffer) {
  const view = new DataView(packet)
  const id = view.getUint8(0)
  const packetView = new DataView(packet, 1)

  return [id, packetView] as const
}
