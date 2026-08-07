#pragma once

#include <Mesh.h>
#include <RadioLib.h>

class RadioLibWrapper : public mesh::Radio {
protected:
  PhysicalLayer* _radio;
  mesh::MainBoard* _board;
  uint32_t n_recv, n_sent;
  uint32_t n_read_not_rx;   // reads that consumed a completion flag while not in Rx
  uint32_t n_recv_errors;   // readData() failures -- a frame arrived and was lost on the way out
  int16_t _noise_floor, _threshold;
  uint16_t _num_floor_samples;
  int32_t _floor_sample_sum;

  /**
   * A reception has COMPLETED and its bytes are still in the chip buffer, unread.
   *
   * Deliberately NOT a bit inside `state`. `state` is a mode word that four call
   * sites assign wholesale (startSendRaw, isSendComplete, onSendFinished, idle), and
   * an arrival signal stored in it is destroyed by every one of those assignments --
   * the defect this member exists to end. Two independent facts, two variables.
   */
  bool _rx_pending;

  /** Chip-specific IRQ bits for RX_DONE / TX_DONE, resolved once in begin(). */
  uint32_t _irq_rx_done, _irq_tx_done;

  /**
   * False when this radio cannot report RX_DONE and TX_DONE separately. In that case
   * every path below falls back to upstream's flag-alone behaviour rather than
   * silently never receiving.
   */
  bool _irq_split_ok;

  uint32_t resolvePendingIrq();
  void idle();
  void startRecv();
  float packetScoreInt(float snr, int sf, int packet_len);
  virtual bool isReceivingPacket() =0;

public:
  RadioLibWrapper(PhysicalLayer& radio, mesh::MainBoard& board) : _radio(&radio), _board(&board) {
    n_recv = n_sent = n_read_not_rx = n_recv_errors = 0;
    _rx_pending = false;
    _irq_rx_done = _irq_tx_done = 0;
    _irq_split_ok = false;
  }

  void begin() override;
  virtual void powerOff() { _radio->sleep(); }
  int recvRaw(uint8_t* bytes, int sz) override;
  uint32_t getEstAirtimeFor(int len_bytes) override;
  bool startSendRaw(const uint8_t* bytes, int len) override;
  bool isSendComplete() override;
  void onSendFinished() override;
  bool isInRecvMode() const override;
  bool isChannelActive();

  bool isReceiving() override {
    if (isReceivingPacket()) return true;

    return isChannelActive();
  }

  bool hasUnreadRx() override { return _rx_pending; }

  virtual float getCurrentRSSI() =0;

  int getNoiseFloor() const override { return _noise_floor; }
  void triggerNoiseFloorCalibrate(int threshold) override;
  void resetAGC() override;

  void loop() override;

  uint32_t getPacketsRecv() const { return n_recv; }
  uint32_t getPacketsSent() const { return n_sent; }

  /**
   * Reads that consumed a completion flag while the radio was not in receive
   * mode. Non-zero means a transmission's completion interrupt was taken for a
   * reception and the chip buffer was read back as if a frame had arrived.
   */
  uint32_t getReadsNotInRx() const { return n_read_not_rx; }

  /**
   * Receptions the radio driver reported and then threw away: readData() returned
   * something other than RADIOLIB_ERR_NONE, so the frame is indistinguishable from
   * "nothing arrived" everywhere downstream. Upstream counts the same thing
   * (`c16bcd2f`).
   */
  uint32_t getRecvErrors() const { return n_recv_errors; }

  void resetStats() { n_recv = n_sent = n_read_not_rx = n_recv_errors = 0; }

  virtual float getLastRSSI() const override;
  virtual float getLastSNR() const override;

  float packetScore(float snr, int packet_len) override { return packetScoreInt(snr, 10, packet_len); }  // assume sf=10
};

/**
 * \brief  an RNG impl using the noise from the LoRa radio as entropy.
 *         NOTE: this is VERY SLOW!  Use only for things like creating new LocalIdentity
*/
class RadioNoiseListener : public mesh::RNG {
  PhysicalLayer* _radio;
public:
  RadioNoiseListener(PhysicalLayer& radio): _radio(&radio) { }

  void random(uint8_t* dest, size_t sz) override {
    for (int i = 0; i < sz; i++) {
      dest[i] = _radio->randomByte() ^ (::random(0, 256) & 0xFF);
    }
  }
};
