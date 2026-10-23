// Application layer.
//
// The non-time-critical main-loop work, decoupled from the Control layer's
// ISR via lock-free buffers or volatile shared state:
//
//   - Maze solving (explore + solve phases)
//   - Run/state management (competition run lifecycle)
//   - Logging to FRAM and telemetry framing out to the ESP32 radio link
//
// Empty for now -- this is a scaffold. See firmware_architecture_adr.md for
// the intended layering.

#pragma once
