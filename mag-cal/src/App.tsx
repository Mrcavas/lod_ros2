import { createEventSignal } from "@solid-primitives/event-listener"
import { createWS, createWSState } from "@solid-primitives/websocket"
import CalibrationPage from "./CalibrationPage"

export default function App() {
  const ws = createWS(`ws://localhost:8080/ws`)
  ws.binaryType = "arraybuffer"
  const stateIndex = createWSState(ws)
  const state = () => ["Connecting", "Connected", "Disconnecting", "Disconnected"][stateIndex()]

  const messageEvent = createEventSignal(ws, "message")
  const message = () => (messageEvent() as MessageEvent)?.data as ArrayBuffer | undefined

  return (
    <main class="relative mx-auto flex h-full min-h-fit w-full max-w-[70vh] flex-col items-center px-8 py-8">
      <h1 class="text-2xl">{state()}</h1>

      <CalibrationPage ws={ws} message={message} />
    </main>
  )
}
