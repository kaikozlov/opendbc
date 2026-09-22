#!/usr/bin/env python3
from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.structs import RadarData
from opendbc.car.toyota.values import DBC, ToyotaFlags
from opendbc.car.interfaces import RadarInterfaceBase


def _create_radar_can_parser(CP):
  if CP.flags & ToyotaFlags.TSS3:
    # TSS3 publishes three banks of eight objects. 0x180..0x182 carry
    # range/lateral geometry and 0x183..0x185 carry the matching relative
    # velocity record. The remaining synchronized companions contain object
    # metadata that is not needed for RadarPoint.
    messages = [(msg, 20) for msg in range(0x180, 0x186)]
  else:
    if CP.flags & ToyotaFlags.TSS2:
      RADAR_A_MSGS = list(range(0x180, 0x190))
      RADAR_B_MSGS = list(range(0x190, 0x1a0))
    else:
      RADAR_A_MSGS = list(range(0x210, 0x220))
      RADAR_B_MSGS = list(range(0x220, 0x230))

    msg_a_n = len(RADAR_A_MSGS)
    msg_b_n = len(RADAR_B_MSGS)
    messages = list(zip(RADAR_A_MSGS + RADAR_B_MSGS, [20] * (msg_a_n + msg_b_n), strict=True))
    messages.append(('STATUS_MSG', 10))

  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    if CP.flags & ToyotaFlags.TSS3:
      self.RADAR_A_MSGS = list(range(0x180, 0x183))
      self.RADAR_B_MSGS = list(range(0x183, 0x186))
      self.trigger_msg = 0x185
    elif CP.flags & ToyotaFlags.TSS2:
      self.RADAR_A_MSGS = list(range(0x180, 0x190))
      self.RADAR_B_MSGS = list(range(0x190, 0x1a0))
      self.trigger_msg = self.RADAR_B_MSGS[-1]
    else:
      self.RADAR_A_MSGS = list(range(0x210, 0x220))
      self.RADAR_B_MSGS = list(range(0x220, 0x230))
      self.trigger_msg = self.RADAR_B_MSGS[-1]

    self.valid_cnt = {key: 0 for key in self.RADAR_A_MSGS}

    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP)
    self.updated_messages = set()
    self.tss3_cycle = None
    self.tss3_cycle_time = None

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    if self.CP.flags & ToyotaFlags.TSS3:
      if can_strings and not isinstance(can_strings[0], list | tuple):
        can_strings = [can_strings]
      return self._update_tss3(can_strings)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()
    return rr

  def _update_tss3(self, can_packets):
    # Consume every source cycle, including intermediate lifecycle events when
    # multiple cycles arrive in one publication. Reading only the last vl would
    # lose a deletion/replacement followed by an ordinary tracking update.
    self.frame += 1
    result = None
    for nanos, packets in can_packets:
      for address, data, bus in packets:
        if bus != self.rcp.bus or address not in self.rcp.addresses or len(data) != 64:
          continue
        updated = self.rcp.update([(nanos, [(address, data, bus)])])
        self.updated_messages.update(updated)
        if len(self.updated_messages) != len(self.rcp.addresses):
          continue
        cycles = {(int(self.rcp.vl[a]["COUNTER"]), int(self.rcp.vl[a]["CYCLE_BYTE"])) for a in self.rcp.addresses}
        if len(cycles) != 1:
          continue
        cycle = next(iter(cycles))
        self.updated_messages.clear()
        if cycle == self.tss3_cycle:
          continue
        # Both source bytes increment independently modulo 256; they are not
        # one 16-bit counter. A missing cycle can hide a one-frame lifecycle
        # flag, so do not carry object identity across it.
        gap = self.tss3_cycle is not None and any((new - old) % 256 != 1 for new, old in zip(cycle, self.tss3_cycle, strict=True))
        timeout = self.tss3_cycle_time is not None and any(
          nanos - self.tss3_cycle_time > state.timeout_threshold for state in self.rcp.message_states.values())
        if gap or timeout:
          self.pts.clear()
        self.tss3_cycle = cycle
        self.tss3_cycle_time = nanos
        result = self._update_tss3_points()
      # Advance normal CAN-parser liveness even when no radar frame arrives.
      self.rcp.update([(nanos, [])])

    if not self.rcp.can_valid:
      self.pts.clear()
      if self.tss3_cycle_time is not None or self.rcp.bus_timeout:
        self.updated_messages.clear()
      self.tss3_cycle = None
      self.tss3_cycle_time = None
      if result is not None or self.frame % 5 == 0:
        result = RadarData()
        result.errors.canError = True
    return result

  def _update_tss3_points(self):
    for bank, address in enumerate(self.RADAR_A_MSGS):
      geometry, motion = self.rcp.vl[address], self.rcp.vl[address + 3]
      for slot in range(8):
        key = bank * 8 + slot
        # Ending the previous track and starting its replacement can happen in
        # the same occupied slot, without an intervening empty range sentinel.
        if motion[f"TRACK_ENDED_{slot}"] or motion[f"NEW_TRACK_{slot}"]:
          self.pts.pop(key, None)
        if motion[f"TRACK_ENDED_{slot}"] and not motion[f"NEW_TRACK_{slot}"]:
          continue
        distance = geometry[f"DIST_{slot}"]
        if motion[f"TRACK_STATE_{slot}"] == 0 or not 0 < distance < 0xFFF8 * 0.005:
          self.pts.pop(key, None)
          continue
        if key not in self.pts:
          self.pts[key] = RadarData.RadarPoint()
          self.pts[key].trackId = self.track_id
          self.track_id += 1
        self.pts[key].dRel = distance
        self.pts[key].yRel = geometry[f"LAT_{slot}"]
        self.pts[key].vRel = motion[f"VREL_{slot}"]
    result = RadarData()
    result.points = list(self.pts.values())
    return result

  def _update(self, updated_messages):
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.rcp.vl['STATUS_MSG']['RADAR_STATUS'] != 1 or self.rcp.vl['STATUS_MSG']['RADAR_PRE_FAULT'] != 0:
      ret.errors.radarUnavailableTemporary = True

    for ii in sorted(updated_messages):
      if ii in self.RADAR_A_MSGS:
        cpt = self.rcp.vl[ii]

        if cpt['LONG_DIST'] >= 255 or cpt['NEW_TRACK']:
          self.valid_cnt[ii] = 0    # reset counter
        if cpt['VALID'] and cpt['LONG_DIST'] < 255:
          self.valid_cnt[ii] += 1
        else:
          self.valid_cnt[ii] = max(self.valid_cnt[ii] - 1, 0)

        score = self.rcp.vl[ii+16]['SCORE']
        # print ii, self.valid_cnt[ii], score, cpt['VALID'], cpt['LONG_DIST'], cpt['LAT_DIST']

        # radar point only valid if it's a valid measurement and score is above 50
        if cpt['VALID'] or (score > 50 and cpt['LONG_DIST'] < 255 and self.valid_cnt[ii] > 0):
          if ii not in self.pts or cpt['NEW_TRACK']:
            self.pts[ii] = RadarData.RadarPoint()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1
          self.pts[ii].dRel = cpt['LONG_DIST']  # from front of car
          self.pts[ii].yRel = -cpt['LAT_DIST']  # in car frame's y axis, left is positive
          self.pts[ii].vRel = cpt['REL_SPEED']
        else:
          if ii in self.pts:
            del self.pts[ii]

    ret.points = list(self.pts.values())
    return ret
