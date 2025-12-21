import { createEffect, createSignal, For, on, Show } from "solid-js"
import PointSpace from "./PointSpace"
import { calibrateMagnetometer, Point, MagCalibrationData } from "./math"
import { buildMagCalibrationStopPacket, buildStartMagCalibrationPacket, getPacketData } from "./packets"
import { createSessionSignal } from "./signal"

const MIN_MAG_POINTS = 100

export default function CalibrationPage(props: { ws: WebSocket; message: () => ArrayBuffer }) {
  const [collectedPoints, setCollectedPoints] = createSessionSignal<Point[]>("points", -1, [], localStorage)
  const [calibratingMag, setCalibratingMag] = createSignal(false)
  const [magCalData, setMagCalData] = createSignal<MagCalibrationData | undefined>()
  const [displayFixed, setDisplayFixed] = createSignal(false)

  createEffect(
    on(props.message, buffer => {
      if (!buffer) return
      const [id, view] = getPacketData(buffer)

      if (id === 0x01) {
        const newPoints = [
          ...collectedPoints(),
          [view.getFloat32(0, true), view.getFloat32(4, true), view.getFloat32(8, true)] as Point,
        ]
        setCollectedPoints(newPoints)

        if (calibratingMag() && newPoints.length >= MIN_MAG_POINTS) {
          if (newPoints.length % 10 != 0) return

          const calData = calibrateMagnetometer(newPoints)
          setMagCalData(calData)
        } else {
          setMagCalData()
        }
      }
    })
  )

  return (
    <>
      <h2 class="text-lg">Calibration</h2>
      <h3 class="text-md mt-1">Magnetometer</h3>

      <PointSpace
        points={collectedPoints()}
        calData={displayFixed() ? magCalData() : undefined}
        onClick={() => setDisplayFixed(v => !v)}
        class="mt-1 aspect-square w-full overflow-hidden rounded-lg"
      />

      <div class="mt-4 flex gap-4">
        <button
          onClick={() => {
            if (calibratingMag()) {
              setCalibratingMag(false)
              props.ws.send(buildMagCalibrationStopPacket())
              return
            }

            setCollectedPoints([])
          }}
          class="rounded-lg bg-red-300 px-4 py-2">
          {calibratingMag() ? "Stop" : "Clear"}
        </button>
        <button
          onClick={() => {
            if (!calibratingMag()) {
              props.ws.send(buildStartMagCalibrationPacket())
            } else {
              props.ws.send(buildMagCalibrationStopPacket())
              console.log(magCalData())
            }

            setCalibratingMag(v => !v)
          }}
          class={"rounded-lg px-4 py-2 " + (calibratingMag() ? "bg-blue-300" : "bg-cyan-300")}>
          {calibratingMag() ? "Log Calibration" : "Start Calibration"}
        </button>
      </div>

      <p class="mt-0.5 text-gray-600">
        Points collected: {collectedPoints().length} / {MIN_MAG_POINTS}
      </p>
      <div class="mt-1 text-sm text-gray-700">
        <p>Fit Error: {magCalData()?.fitError.toFixed(2) ?? "N/A"}%</p>
        <p>Field Strength: {magCalData()?.fieldStrength.toFixed(2) ?? "N/A"} µT</p>
      </div>
    </>
  )
}
