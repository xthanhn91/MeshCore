#pragma once

#include <Mesh.h>

#ifdef ESP32
  #include <FS.h>
#endif

/* HopNet fork patch -- raised from 128 (docs/flood-control-dedup-architecture.md
 * §7.2, Proposition 4).
 *
 * A hash survives in this ring exactly as long as it takes the next
 * MAX_PACKET_HASHES unique frames to arrive; there is no timer. A late copy of
 * a flood that arrives after that point is not recognised as a duplicate -- it
 * is processed and relayed a second time, and no other guard catches it
 * (echo-detection only sees frames the node sent itself).
 *
 * At the design point -- 100 nodes, one position beacon each per 30 s -- new
 * frames arrive at ~3.3/s, so 128 slots are overwritten in ~38 s while the
 * longest flood lifetime is ~26 s. A safety margin of 1.0-1.4x, and the
 * measured TTL of 5 (the model assumed 4) makes it narrower still. 256 slots
 * push the overwrite horizon past 77 s, a margin near 3x.
 *
 * Costs 1 KB of SRAM. Both boards have room: the T114, the tighter of the two,
 * sits near a quarter of its RAM.
 *
 * Whether this actually removes the ghosts is not assumed: obs-catalog 0xA8
 * counts them from a witness that remembers 512 frames -- deliberately longer
 * than this table, so that a zero reading means "did not happen" rather than
 * "cannot be seen". That is field test T6. */
#define MAX_PACKET_HASHES  256
#define MAX_PACKET_ACKS     64

class SimpleMeshTables : public mesh::MeshTables {
  uint8_t _hashes[MAX_PACKET_HASHES*MAX_HASH_SIZE];
  int _next_idx;
  uint32_t _acks[MAX_PACKET_ACKS];
  int _next_ack_idx;
  uint32_t _direct_dups, _flood_dups;

public:
  SimpleMeshTables() { 
    memset(_hashes, 0, sizeof(_hashes));
    _next_idx = 0;
    memset(_acks, 0, sizeof(_acks));
    _next_ack_idx = 0;
    _direct_dups = _flood_dups = 0;
  }

#ifdef ESP32
  void restoreFrom(File f) {
    f.read(_hashes, sizeof(_hashes));
    f.read((uint8_t *) &_next_idx, sizeof(_next_idx));
    f.read((uint8_t *) &_acks[0], sizeof(_acks));
    f.read((uint8_t *) &_next_ack_idx, sizeof(_next_ack_idx));
  }
  void saveTo(File f) {
    f.write(_hashes, sizeof(_hashes));
    f.write((const uint8_t *) &_next_idx, sizeof(_next_idx));
    f.write((const uint8_t *) &_acks[0], sizeof(_acks));
    f.write((const uint8_t *) &_next_ack_idx, sizeof(_next_ack_idx));
  }
#endif

  bool hasSeen(const mesh::Packet* packet) override {
    if (packet->getPayloadType() == PAYLOAD_TYPE_ACK) {
      uint32_t ack;
      memcpy(&ack, packet->payload, 4);
      for (int i = 0; i < MAX_PACKET_ACKS; i++) {
        if (ack == _acks[i]) { 
          if (packet->isRouteDirect()) {
            _direct_dups++;   // keep some stats
          } else {
            _flood_dups++;
          }
          return true;
        }
      }
  
      _acks[_next_ack_idx] = ack;
      _next_ack_idx = (_next_ack_idx + 1) % MAX_PACKET_ACKS;  // cyclic table  
      return false;
    }

    uint8_t hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(hash);

    const uint8_t* sp = _hashes;
    for (int i = 0; i < MAX_PACKET_HASHES; i++, sp += MAX_HASH_SIZE) {
      if (memcmp(hash, sp, MAX_HASH_SIZE) == 0) { 
        if (packet->isRouteDirect()) {
          _direct_dups++;   // keep some stats
        } else {
          _flood_dups++;
        }
        return true;
      }
    }

    memcpy(&_hashes[_next_idx*MAX_HASH_SIZE], hash, MAX_HASH_SIZE);
    _next_idx = (_next_idx + 1) % MAX_PACKET_HASHES;  // cyclic table
    return false;
  }

  void clear(const mesh::Packet* packet) override {
    if (packet->getPayloadType() == PAYLOAD_TYPE_ACK) {
      uint32_t ack;
      memcpy(&ack, packet->payload, 4);
      for (int i = 0; i < MAX_PACKET_ACKS; i++) {
        if (ack == _acks[i]) { 
          _acks[i] = 0;
          break;
        }
      }
    } else {
      uint8_t hash[MAX_HASH_SIZE];
      packet->calculatePacketHash(hash);

      uint8_t* sp = _hashes;
      for (int i = 0; i < MAX_PACKET_HASHES; i++, sp += MAX_HASH_SIZE) {
        if (memcmp(hash, sp, MAX_HASH_SIZE) == 0) { 
          memset(sp, 0, MAX_HASH_SIZE);
          break;
        }
      }
    }
  }

  uint32_t getNumDirectDups() const { return _direct_dups; }
  uint32_t getNumFloodDups() const { return _flood_dups; }

  void resetStats() { _direct_dups = _flood_dups = 0; }
};
