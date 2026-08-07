
#define RADIOLIB_STATIC_ONLY 1
#include "RadioLibWrappers.h"

#define STATE_IDLE       0
#define STATE_RX         1
#define STATE_TX_WAIT    3
#define STATE_TX_DONE    4
#define STATE_INT_READY 16

#define NUM_NOISE_FLOOR_SAMPLES  64
#define SAMPLING_THRESHOLD  14

static volatile uint8_t state = STATE_IDLE;

// this function is called when a complete packet
// is transmitted by the module
static 
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  // we sent a packet, set the flag
  state |= STATE_INT_READY;
}

void RadioLibWrapper::begin() {
  _radio->setPacketReceivedAction(setFlag);  // this is also SentComplete interrupt
  state = STATE_IDLE;

  // HopNet fork: setFlag() is ONE interrupt for two events (see the comment on the
  // line above -- it is upstream's own). The chip does keep them apart, in its IRQ
  // status register, so resolve the chip-specific bits for RX_DONE and TX_DONE once
  // here rather than inferring the event from `state`, which cannot tell them apart.
  _irq_rx_done = _radio->getIrqMapped(1UL << RADIOLIB_IRQ_RX_DONE);
  _irq_tx_done = _radio->getIrqMapped(1UL << RADIOLIB_IRQ_TX_DONE);
  _irq_split_ok = (_irq_rx_done != 0 && _irq_tx_done != 0);
  _rx_pending = false;
  if (!_irq_split_ok) {
    // Fail OPEN, not closed: a radio that cannot report the two apart keeps
    // upstream's flag-alone behaviour. Gating receives on a signal this chip never
    // produces would stop it receiving altogether.
    MESH_DEBUG_PRINTLN("RadioLibWrapper: WARNING: radio cannot separate RX_DONE from TX_DONE; anomaly-B protection disabled");
  }

  if (_board->getStartupReason() == BD_STARTUP_RX_PACKET) {  // received a LoRa packet (while in deep sleep)
    setFlag(); // LoRa packet is already received
  }

  _noise_floor = 0;
  _threshold = 0;

  // start average out some samples
  _num_floor_samples = 0;
  _floor_sample_sum = 0;
}

void RadioLibWrapper::idle() {
  _radio->standby();
  state = STATE_IDLE;   // need another startReceive()

  // Only reached from startSendRaw()'s failure path. startTransmit() stages the frame
  // into the chip buffer BEFORE it can report an error, so a reception that was
  // waiting there may already be overwritten and there is no way to ask which. Drop
  // the pending marker: losing one frame in a rare error path is the cheaper mistake
  // than reading our own staged bytes back as an inbound frame.
  _rx_pending = false;
}

void RadioLibWrapper::triggerNoiseFloorCalibrate(int threshold) {
  _threshold = threshold;
  if (_num_floor_samples >= NUM_NOISE_FLOOR_SAMPLES) {  // ignore trigger if currently sampling
    _num_floor_samples = 0;
    _floor_sample_sum = 0;
  }
}

void RadioLibWrapper::resetAGC() {
  // make sure we're not mid-receive of packet!
  if ((state & STATE_INT_READY) != 0 || isReceivingPacket()) return;

  // NOTE: according to higher powers, just issuing RadioLib's startReceive() will reset the AGC.
  //      revisit this if a better impl is discovered.
  state = STATE_IDLE;   // trigger a startReceive()
}

void RadioLibWrapper::loop() {
  if (state == STATE_RX && _num_floor_samples < NUM_NOISE_FLOOR_SAMPLES) {
    if (!isReceivingPacket()) {
      int rssi = getCurrentRSSI();
      if (rssi < _noise_floor + SAMPLING_THRESHOLD) {  // only consider samples below current floor + sampling THRESHOLD
        _num_floor_samples++;
        _floor_sample_sum += rssi;
      }
    }
  } else if (_num_floor_samples >= NUM_NOISE_FLOOR_SAMPLES && _floor_sample_sum != 0) {
    _noise_floor = _floor_sample_sum / NUM_NOISE_FLOOR_SAMPLES;
    if (_noise_floor < -120) {
      _noise_floor = -120;    // clamp to lower bound of -120dBi
    }
    _floor_sample_sum = 0;

    MESH_DEBUG_PRINTLN("RadioLibWrapper: noise_floor = %d", (int)_noise_floor);
  }
}

void RadioLibWrapper::startRecv() {
  int err = _radio->startReceive();
  if (err == RADIOLIB_ERR_NONE) {
    state = STATE_RX;
  } else {
    MESH_DEBUG_PRINTLN("RadioLibWrapper: error: startReceive(%d)", err);
  }
}

bool RadioLibWrapper::isInRecvMode() const {
  return (state & ~STATE_INT_READY) == STATE_RX;
}

/**
 * Decide what the shared completion interrupt actually meant.
 *
 * The ISR cannot do this itself: telling RX_DONE from TX_DONE means reading the
 * chip's IRQ status over SPI, which an interrupt handler must not do. So setFlag()
 * records only "something completed" and the answer is resolved here, in the polling
 * context, from ONE SPI read that both callers share.
 *
 * A resolved reception is recorded in _rx_pending, which is sticky: it survives every
 * `state = ...` assignment, so the arrival signal is no longer destroyed by our own
 * transmit path. It is cleared only when the frame is actually read out, or when the
 * transmit that overwrote the chip buffer finishes.
 *
 * \returns the raw IRQ status word (0 when no interrupt is outstanding).
 */
uint32_t RadioLibWrapper::resolvePendingIrq() {
  if ((state & STATE_INT_READY) == 0) return 0;
  if (!_irq_split_ok) return 0;

  uint32_t irq = _radio->getIrqFlags();
  if (irq & _irq_rx_done) _rx_pending = true;
  return irq;
}

int RadioLibWrapper::recvRaw(uint8_t* bytes, int sz) {
  int len = 0;
  resolvePendingIrq();

  if (state & STATE_INT_READY) {
    // setFlag() is the completion interrupt for a transmission as well as for a
    // reception, and THIS test cannot tell the two apart -- the IRQ status read in
    // resolvePendingIrq() can. Kept on its original predicate so the metric stays
    // comparable across this fix: it still counts completion flags consumed outside
    // receive mode, which is the anomaly-B discriminator it was minted for.
    if ((state & ~STATE_INT_READY) != STATE_RX) n_read_not_rx++;

    // Read the chip buffer ONLY on evidence that a frame actually arrived. Without
    // this test, a transmit's completion is read back as a reception and hands our
    // own outgoing frame to the mesh as if a peer had sent it.
    if (_rx_pending || !_irq_split_ok) {
      len = _radio->getPacketLength();
      if (len > 0) {
        if (len > sz) { len = sz; }
        int err = _radio->readData(bytes, len);
        if (err != RADIOLIB_ERR_NONE) {
          MESH_DEBUG_PRINTLN("RadioLibWrapper: error: readData(%d)", err);
          n_recv_errors++;   // HopNet fork: this loss used to be invisible past the log line
          len = 0;
        } else {
        //  Serial.print("  readData() -> "); Serial.println(len);
          n_recv++;
        }
      }
      _rx_pending = false;   // read out (or empty) -- either way nothing is waiting
    }
    state = STATE_IDLE;   // need another startReceive()
  }

  if (state != STATE_RX) {
    int err = _radio->startReceive();
    if (err == RADIOLIB_ERR_NONE) {
      state = STATE_RX;
    } else {
      MESH_DEBUG_PRINTLN("RadioLibWrapper: error: startReceive(%d)", err);
    }
  }
  return len;
}

uint32_t RadioLibWrapper::getEstAirtimeFor(int len_bytes) {
  return _radio->getTimeOnAir(len_bytes) / 1000;
}

bool RadioLibWrapper::startSendRaw(const uint8_t* bytes, int len) {
  _board->onBeforeTransmit();
  int err = _radio->startTransmit((uint8_t *) bytes, len);
  if (err == RADIOLIB_ERR_NONE) {
    state = STATE_TX_WAIT;
    return true;
  }
  MESH_DEBUG_PRINTLN("RadioLibWrapper: error: startTransmit(%d)", err);
  idle();   // trigger another startRecv()
  return false;
}

bool RadioLibWrapper::isSendComplete() {
  // Deliberately NOT gated on TX_DONE, and the reason is measured, not assumed.
  //
  // An earlier revision required `irq & _irq_tx_done` here, on the theory that a
  // reception arriving just before the transmit was being consumed as a
  // send-completion and retiring the packet early. On the bench that gate turned
  // sends into TIMEOUTS: the same board, in the same spot, over the same 180s
  // window, logged +30 logTxFail drops with the strict check and +0 without it,
  // while the four boards beside it showed +7..+16. logTxFail DROPS the packet, so
  // the cost was real transmissions lost -- traded against a defect that is only
  // theorised. Waiting for a TX_DONE that does not arrive is worse than acting on
  // a completion flag that is occasionally the wrong one.
  //
  // The anomaly-B fix does not depend on this. What stops our own frame coming
  // back as an inbound one is the RX_DONE gate in recvRaw(); this function only
  // decides when to retire an outbound packet.
  resolvePendingIrq();   // still resolve, so a reception pending across the transmit
                         // is recorded in _rx_pending rather than lost
  if (state & STATE_INT_READY) {
    state = STATE_IDLE;
    n_sent++;
    return true;
  }
  return false;
}

void RadioLibWrapper::onSendFinished() {
  _radio->finishTransmit();
  _board->onAfterTransmit();
  state = STATE_IDLE;

  // The transmit rewrote the chip's buffer base and wrote our frame there, and
  // finishTransmit() has just cleared the chip's IRQ status. Any reception that was
  // still unread is therefore gone -- both its bytes and its arrival signal. Drop the
  // pending marker rather than let recvRaw() read the buffer and hand our own
  // outgoing frame back as an inbound one.
  _rx_pending = false;
}

bool RadioLibWrapper::isChannelActive() {
  return _threshold == 0 
          ? false    // interference check is disabled
          : getCurrentRSSI() > _noise_floor + _threshold;
}

float RadioLibWrapper::getLastRSSI() const {
  return _radio->getRSSI();
}
float RadioLibWrapper::getLastSNR() const {
  return _radio->getSNR();
}

// Approximate SNR threshold per SF for successful reception (based on Semtech datasheets)
static float snr_threshold[] = {
    -7.5,  // SF7 needs at least -7.5 dB SNR
    -10,   // SF8 needs at least -10 dB SNR
    -12.5, // SF9 needs at least -12.5 dB SNR
    -15,  // SF10 needs at least -15 dB SNR
    -17.5,// SF11 needs at least -17.5 dB SNR
    -20   // SF12 needs at least -20 dB SNR
};
  
float RadioLibWrapper::packetScoreInt(float snr, int sf, int packet_len) {
  if (sf < 7) return 0.0f;
  
  if (snr < snr_threshold[sf - 7]) return 0.0f;    // Below threshold, no chance of success

  auto success_rate_based_on_snr = (snr - snr_threshold[sf - 7]) / 10.0;
  auto collision_penalty = 1 - (packet_len / 256.0);   // Assuming max packet of 256 bytes

  return max(0.0, min(1.0, success_rate_based_on_snr * collision_penalty));
}
