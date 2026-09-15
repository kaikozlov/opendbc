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

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.CP.flags & ToyotaFlags.TSS3:
      # Empty geometry slots use the exact 0xFFF8 longitudinal sentinel
      # (655.28 m). The three core RadarPoint quantities are source-real:
      # u16*0.01 m range, s12*0.05 m lateral, and s10*0.1 m/s relative speed.
      # Toyota's existing radar convention is right-positive on the wire, while
      # openpilot's car frame is left-positive, hence the yRel sign inversion.
      for bank, geometry_msg in enumerate(self.RADAR_A_MSGS):
        geometry = self.rcp.vl[geometry_msg]
        motion = self.rcp.vl[geometry_msg + 3]
        for slot in range(8):
          point_id = bank * 8 + slot
          distance = geometry[f'DIST_{slot}']
          if 0. < distance <= 500.:
            if point_id not in self.pts:
              self.pts[point_id] = RadarData.RadarPoint()
              self.pts[point_id].trackId = point_id
            self.pts[point_id].dRel = distance
            self.pts[point_id].yRel = -geometry[f'LAT_{slot}']
            self.pts[point_id].vRel = motion[f'VREL_{slot}']
          elif point_id in self.pts:
            del self.pts[point_id]

      ret.points = list(self.pts.values())
      return ret

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
