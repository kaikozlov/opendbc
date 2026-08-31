import math
import numpy as np
from opendbc.car import Bus, make_tester_present_msg, rate_limit, structs, ACCELERATION_DUE_TO_GRAVITY, DT_CTRL
from opendbc.car.lateral import apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance
from opendbc.car.carlog import carlog
from opendbc.car.common.filter_simple import FirstOrderFilter, HighPassFilter
from opendbc.car.common.pid import PIDController
from opendbc.car.secoc import add_mac, add_mac28_zero_marker, build_sync_mac
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.toyota import toyotacan
from opendbc.car.toyota.tss3 import (TSS3B6CompanionFields, TSS3B6Template, TSS3Freshness, TSS3PandaSafetyCandidate,
                                     TSS3_B6_TARGET_ANGLE_MAX_RAW, TSS3_B6_TARGET_ANGLE_SCALE_DEG,
                                     TSS3_B6_TARGET_DELTA_MAX_PER_GAP_RAW,
                                     TSS3_B6_TARGET_LATERAL_ID_INACTIVE, TSS3_B6_TARGET_LATERAL_ID_LTA_LCA,
                                     build_b6_application, build_b6_zero_marker_frame, target_angle_deg_to_raw)
from opendbc.car.toyota.values import CAR, CarControllerParams, ToyotaFlags
from opendbc.can import CANPacker

Ecu = structs.CarParams.Ecu
LongCtrlState = structs.CarControl.Actuators.LongControlState
SteerControlType = structs.CarParams.SteerControlType
VisualAlert = structs.CarControl.HUDControl.VisualAlert

# The up limit allows the brakes/gas to unwind quickly leaving a stop,
# the down limit roughly matches the rate of ACCEL_NET, reducing PCM compensation windup
ACCEL_WINDUP_LIMIT = 4.0 * DT_CTRL * 3  # m/s^2 / frame
ACCEL_WINDDOWN_LIMIT = -4.0 * DT_CTRL * 3  # m/s^2 / frame
ACCEL_PID_UNWIND = 0.03 * DT_CTRL * 3  # m/s^2 / frame

MAX_PITCH_COMPENSATION = 1.5  # m/s^2

# LKA limits
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 17  # tx control frames needed before torque can be cut

# EPS allows user torque above threshold for 50 frames before permanently faulting
MAX_USER_TORQUE = 500


def get_long_tune(CP, params):
  if CP.flags & ToyotaFlags.TSS2:
    kiBP = [2., 5.]
    kiV = [0.5, 0.25]
  else:
    kiBP = [0., 5., 35.]
    kiV = [3.6, 2.4, 1.5]

  return PIDController(0.0, (kiBP, kiV), k_f=1.0,
                       pos_limit=params.ACCEL_MAX, neg_limit=params.ACCEL_MIN,
                       rate=1 / (DT_CTRL * 3))


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.last_torque = 0
    self.last_angle = 0
    self.alert_active = False
    self.standstill_req = False
    self.permit_braking = True
    self.steer_rate_counter = 0
    self.distance_button = 0

    # *** start long control state ***
    self.long_pid = get_long_tune(self.CP, self.params)
    self.aego = FirstOrderFilter(0.0, 0.25, DT_CTRL * 3)
    self.pitch = FirstOrderFilter(0, 0.5, DT_CTRL)
    self.pitch_hp = HighPassFilter(0.0, 0.25, 1.5, DT_CTRL)

    self.accel = 0
    self.prev_accel = 0
    # *** end long control state ***

    self.packer = CANPacker(dbc_names[Bus.pt])

    self.secoc_lka_message_counter = 0
    self.secoc_lta_message_counter = 0
    self.secoc_acc_message_counter = 0
    self.secoc_prev_reset_counter = 0
    # Set by openpilot's car card only after an external, evidence-gated EPS runtime
    # deployment. Default behavior remains ordinary key-backed SecOC.
    self.ephemeral_secoc_bridge = False

    # Exact-F33 shadow state remains analysis-only. The former bus-0 B6
    # development sender was removed after retained factory LTA/LCA proved zero
    # stock B6; exact F33 has a separate B6-independent internal assist path.
    # 0x08A producer/SecOC ownership is a separate network question, not proof
    # of an 0x08A-to-B6 stock-LTA transform.
    self.tss3_template = TSS3B6Template()
    # Exact F33 consumes B8/B9 as /100 controller contributions. The old
    # conservative development candidate left both at zero, which explicitly
    # removes both branches. GTS+ request records expose the adjacent steering
    # assist and damping gains with the same 0.01 scaling, and F33 dataflow maps
    # B8 to the angle/error-assist branch and B9 to a speed/return contribution.
    # Use full-scale gains only while the development ID11 request is active;
    # keep the generic/inactive candidate at the conservative zero defaults.
    self.tss3_inactive_companions = TSS3B6CompanionFields()
    self.tss3_active_companions = TSS3B6CompanionFields(
      additive_term_suppress=0, contribution_pct_1=100, contribution_pct_2=100,
    )
    self.tss3_safety_candidate = TSS3PandaSafetyCandidate()
    self.tss3_last_application = None
    self.tss3_last_safety_decision = None

    # Zero-MAC28 bridge sender state (exact F33). Anchored to the observed
    # 0x00F epoch; sequence advances exactly +1 per sent frame so the
    # ALLOW_DEBUG panda TSS3 dev hook always sees a valid progression.
    self.tss3_bridge_epoch: int | None = None
    self.tss3_bridge_safety_candidate = TSS3PandaSafetyCandidate()
    self.tss3_bridge_sequence = 0
    self.tss3_bridge_message_counter = 0
    self.tss3_bridge_prev_angle_raw: int | None = None

  def update(self, CC, CS, now_nanos):
    if self.CP.flags & ToyotaFlags.TSS3:
      # Compute the exact-F33 *shadow* B6 application command for inspection and
      # replay, but do not schedule it, authenticate it, or return it as CAN.
      # A real sender sequence is owned by TSS3ReplacementFreshnessState only
      # after a newer authenticated 0x00F epoch and validated stock cadence.
      output = CC.actuators.as_builder()
      target_lateral_id = TSS3_B6_TARGET_LATERAL_ID_LTA_LCA if CC.latActive else TSS3_B6_TARGET_LATERAL_ID_INACTIVE
      target_angle_raw = target_angle_deg_to_raw(CC.actuators.steeringAngleDeg) if CC.latActive else 0
      shadow_sequence = self.frame & 0x3F
      self.tss3_last_application = build_b6_application(
        target_lateral_id=target_lateral_id,
        target_angle_raw=target_angle_raw,
        sequence=shadow_sequence,
        template=self.tss3_template,
        companions=self.tss3_active_companions if CC.latActive else self.tss3_inactive_companions,
      )
      self.tss3_last_safety_decision = self.tss3_safety_candidate.check(
        target_lateral_id=target_lateral_id,
        target_angle_raw=target_angle_raw,
        sequence=shadow_sequence,
        steering_angle_velocity_raw=int(round(CS.out.steeringRateDeg)),
        now_nanos=now_nanos,
      )

      can_sends = []

      if self.ephemeral_secoc_bridge:
        # Development-only zero-MAC28 sender for an installed exact-F33
        # EPS bridge. One frame per cycle on bus 0; the ALLOW_DEBUG panda
        # TSS3 hook independently enforces the same F33 envelope on every
        # transmitted frame.
        sync = CS.secoc_synchronization
        epoch = (int(sync['TRIP_CNT']) << 20) | int(sync['RESET_CNT'])
        if self.tss3_bridge_epoch != epoch:
          # New 0x00F epoch: re-anchor progression. The panda hook resets
          # its previous-frame state on a strictly newer epoch as well.
          self.tss3_bridge_epoch = epoch
          self.tss3_bridge_sequence = 0
          self.tss3_bridge_message_counter = 0
          self.tss3_bridge_prev_angle_raw = None
          self.tss3_bridge_safety_candidate = TSS3PandaSafetyCandidate()

        # Follow openpilot's normal lateral-authority contract. controlsd owns
        # CC.latActive; platform-specific coexistence/suppression belongs in
        # Panda forwarding/safety, not in CarController policy.
        want_active = CC.latActive
        desired_raw = target_angle_raw if want_active else 0

        # Stay inside the receiver's per-gap slew envelope while active. An
        # inactive ID0/angle0 is an immediate authority release and is not slew
        # limited against the previous active target.
        if want_active and self.tss3_bridge_prev_angle_raw is not None:
          desired_raw = max(self.tss3_bridge_prev_angle_raw - TSS3_B6_TARGET_DELTA_MAX_PER_GAP_RAW,
                            min(self.tss3_bridge_prev_angle_raw + TSS3_B6_TARGET_DELTA_MAX_PER_GAP_RAW,
                                desired_raw))
        desired_raw = max(-TSS3_B6_TARGET_ANGLE_MAX_RAW, min(TSS3_B6_TARGET_ANGLE_MAX_RAW, desired_raw))

        send_active = want_active
        send_id = TSS3_B6_TARGET_LATERAL_ID_LTA_LCA if send_active else TSS3_B6_TARGET_LATERAL_ID_INACTIVE
        send_raw = desired_raw if send_active else 0

        decision = self.tss3_bridge_safety_candidate.check(
          target_lateral_id=send_id,
          target_angle_raw=send_raw,
          sequence=self.tss3_bridge_sequence,
          steering_angle_velocity_raw=int(round(CS.out.steeringRateDeg)),
          now_nanos=now_nanos,
        )
        if not decision.static_limits_ok:
          # Fail closed to an immediate inactive release. Re-check the final
          # frame so the local sequence state re-anchors exactly as Panda's
          # inactive-release rule does.
          send_active = False
          send_id = TSS3_B6_TARGET_LATERAL_ID_INACTIVE
          send_raw = 0
          decision = self.tss3_bridge_safety_candidate.check(
            target_lateral_id=send_id,
            target_angle_raw=send_raw,
            sequence=self.tss3_bridge_sequence,
            steering_angle_velocity_raw=int(round(CS.out.steeringRateDeg)),
            now_nanos=now_nanos,
          )

        application = build_b6_application(
          target_lateral_id=send_id,
          target_angle_raw=send_raw,
          sequence=self.tss3_bridge_sequence,
          template=self.tss3_template,
          companions=self.tss3_active_companions if send_active else self.tss3_inactive_companions,
        )
        freshness = TSS3Freshness(int(sync['TRIP_CNT']), int(sync['RESET_CNT']),
                                  self.tss3_bridge_message_counter)
        can_sends.append(build_b6_zero_marker_frame(application, freshness))

        # Report the slew-limited angle we actually transmitted so the
        # lateral planner tracks the receiver's envelope, not the request.
        actuators_output = CC.actuators.as_builder()
        actuators_output.steeringAngleDeg = send_raw * TSS3_B6_TARGET_ANGLE_SCALE_DEG
        output = actuators_output

        self.tss3_bridge_sequence = (self.tss3_bridge_sequence + 1) & 0x3F
        self.tss3_bridge_message_counter = (self.tss3_bridge_message_counter + 1) & 0xFF
        self.tss3_bridge_prev_angle_raw = send_raw

      self.frame += 1
      return output, can_sends

    actuators = CC.actuators
    stopping = actuators.longControlState == LongCtrlState.stopping
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])
      self.pitch_hp.update(CC.orientationNED[1])

    # *** control msgs ***
    can_sends = []

    # *** handle secoc reset counter increase ***
    if self.CP.flags & ToyotaFlags.SECOC.value:
      if CS.secoc_synchronization['RESET_CNT'] != self.secoc_prev_reset_counter:
        self.secoc_lka_message_counter = 0
        self.secoc_lta_message_counter = 0
        self.secoc_acc_message_counter = 0
        self.secoc_prev_reset_counter = CS.secoc_synchronization['RESET_CNT']

        if not self.ephemeral_secoc_bridge:
          expected_mac = build_sync_mac(self.secoc_key, int(CS.secoc_synchronization['TRIP_CNT']), int(CS.secoc_synchronization['RESET_CNT']))
          if int(CS.secoc_synchronization['AUTHENTICATOR']) != expected_mac:
            carlog.error("SecOC synchronization MAC mismatch, wrong key?")

    # *** steer torque ***
    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    apply_torque = apply_meas_steer_torque_limits(new_torque, self.last_torque, CS.out.steeringTorqueEps, self.params)

    # >100 degree/sec steering fault prevention
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, lat_active,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    if not lat_active:
      apply_torque = 0

    # *** steer angle ***
    if self.CP.steerControlType == SteerControlType.angle:
      # If using LTA control, disable LKA and set steering angle command
      apply_torque = 0
      apply_steer_req = False
      if self.frame % 2 == 0:
        # EPS uses the torque sensor angle to control with, offset to compensate
        apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        # Angular rate limit based on speed
        self.last_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgoRaw,
                                                       CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg,
                                                       CC.latActive, self.params.ANGLE_LIMITS)

    self.last_torque = apply_torque

    # toyota can trace shows STEERING_LKA at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    steer_command = toyotacan.create_steer_command(self.packer, apply_torque, apply_steer_req)
    if self.CP.flags & ToyotaFlags.SECOC.value:
      if self.ephemeral_secoc_bridge:
        steer_command = add_mac28_zero_marker(int(CS.secoc_synchronization['RESET_CNT']),
                                              self.secoc_lka_message_counter, steer_command)
      else:
        # TODO: check if this slow and needs to be done by the CANPacker
        steer_command = add_mac(self.secoc_key,
                                int(CS.secoc_synchronization['TRIP_CNT']),
                                int(CS.secoc_synchronization['RESET_CNT']),
                                self.secoc_lka_message_counter,
                                steer_command)
      self.secoc_lka_message_counter += 1
    can_sends.append(steer_command)

    # STEERING_LTA does not seem to allow more rate by sending faster, and may wind up easier
    if self.frame % 2 == 0 and self.CP.flags & ToyotaFlags.TSS2:
      lta_active = lat_active and self.CP.steerControlType == SteerControlType.angle
      # cut steering torque with TORQUE_WIND_DOWN when either EPS torque or driver torque is above
      # the threshold, to limit max lateral acceleration and for driver torque blending respectively.
      full_torque_condition = (abs(CS.out.steeringTorqueEps) < self.params.STEER_MAX and
                               abs(CS.out.steeringTorque) < self.params.MAX_LTA_DRIVER_TORQUE_ALLOWANCE)

      # TORQUE_WIND_DOWN at 0 ramps down torque at roughly the max down rate of 1500 units/sec
      torque_wind_down = 100 if lta_active and full_torque_condition else 0
      can_sends.append(toyotacan.create_lta_steer_command(self.packer, self.CP.steerControlType, self.last_angle,
                                                          lta_active, self.frame // 2, torque_wind_down))

      if self.CP.flags & ToyotaFlags.SECOC.value:
        lta_steer_2 = toyotacan.create_lta_steer_command_2(self.packer, self.frame // 2)
        if self.ephemeral_secoc_bridge:
          lta_steer_2 = add_mac28_zero_marker(int(CS.secoc_synchronization['RESET_CNT']),
                                               self.secoc_lta_message_counter, lta_steer_2)
        else:
          lta_steer_2 = add_mac(self.secoc_key,
                                int(CS.secoc_synchronization['TRIP_CNT']),
                                int(CS.secoc_synchronization['RESET_CNT']),
                                self.secoc_lta_message_counter,
                                lta_steer_2)
        self.secoc_lta_message_counter += 1
        can_sends.append(lta_steer_2)

    # handle UI messages
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    lead = hud_control.leadVisible or CS.out.vEgo < 12.  # at low speed we always assume the lead is present so ACC can be engaged

    # *** gas and brake ***
    if self.CP.openpilotLongitudinalControl:
      # if user engages at a stop with foot on brake, PCM starts in a special cruise standstill mode. on resume press,
      # brakes can take a while to ramp up causing a lurch forward. prevent resume press until planner wants to move.
      # don't use CC.cruiseControl.resume since it is gated on CS.cruiseState.standstill which goes false for 3s after resume press
      # whitelist hybrids as they do not have this issue and can stay stopped after resume press
      if not self.CP.flags & ToyotaFlags.HYBRID.value:
        should_resume = actuators.accel > 0
        if should_resume:
          self.standstill_req = False

        if not should_resume and CS.out.cruiseState.standstill:
          self.standstill_req = True

      if self.frame % 3 == 0:
        # Press distance button until we are at the correct bar length. Only change while enabled to avoid skipping startup popup
        if self.frame % 6 == 0 and self.CP.openpilotLongitudinalControl:
          desired_distance = 4 - hud_control.leadDistanceBars
          if CS.out.cruiseState.enabled and CS.pcm_follow_distance != desired_distance:
            self.distance_button = not self.distance_button
          else:
            self.distance_button = 0

        # internal PCM gas command can get stuck unwinding from negative accel so we apply a generous rate limit
        pcm_accel_cmd = actuators.accel
        if CC.longActive:
          pcm_accel_cmd = rate_limit(pcm_accel_cmd, self.prev_accel, ACCEL_WINDDOWN_LIMIT, ACCEL_WINDUP_LIMIT)
        self.prev_accel = pcm_accel_cmd

        # calculate amount of acceleration PCM should apply to reach target, given pitch.
        # clipped to only include downhill angles, avoids erroneously unsetting PERMIT_BRAKING when stopping on uphills
        accel_due_to_pitch = math.sin(min(self.pitch.x, 0.0)) * ACCELERATION_DUE_TO_GRAVITY
        # TODO: on uphills this sometimes sets PERMIT_BRAKING low not considering the creep force
        net_acceleration_request = pcm_accel_cmd + accel_due_to_pitch

        # GVC does not overshoot ego acceleration when starting from stop, but still has a similar delay
        if not self.CP.flags & ToyotaFlags.SECOC.value:
          a_ego_blended = float(np.interp(CS.out.vEgo, [1.0, 2.0], [CS.gvc, CS.out.aEgo]))
        else:
          a_ego_blended = CS.out.aEgo

        # wind down integral when approaching target for step changes and smooth ramps to reduce overshoot
        prev_aego = self.aego.x
        self.aego.update(a_ego_blended)
        j_ego = (self.aego.x - prev_aego) / (DT_CTRL * 3)

        future_t = float(np.interp(CS.out.vEgo, [2., 5.], [0.25, 0.5]))
        a_ego_future = a_ego_blended + j_ego * future_t

        if CC.longActive:
          # constantly slowly unwind integral to recover from large temporary errors
          self.long_pid.i -= ACCEL_PID_UNWIND * float(np.sign(self.long_pid.i))

          error_future = pcm_accel_cmd - a_ego_future

          if not stopping:
            # Toyota's PCM slowly responds to changes in pitch. On change, we amplify our
            # acceleration request to compensate for the undershoot and following overshoot
            pitch_compensation = float(np.clip(math.sin(self.pitch_hp.x) * ACCELERATION_DUE_TO_GRAVITY,
                                               -MAX_PITCH_COMPENSATION, MAX_PITCH_COMPENSATION))
            pcm_accel_cmd += pitch_compensation

          pcm_accel_cmd = self.long_pid.update(error_future,
                                               speed=CS.out.vEgo,
                                               feedforward=pcm_accel_cmd,
                                               freeze_integrator=actuators.longControlState != LongCtrlState.pid)
        else:
          self.long_pid.reset()

        # Along with rate limiting positive jerk above, this greatly improves gas response time
        # Consider the net acceleration request that the PCM should be applying (pitch included)
        net_acceleration_request_min = min(actuators.accel + accel_due_to_pitch, net_acceleration_request)
        if net_acceleration_request_min < 0.2 or stopping or not CC.longActive:
          self.permit_braking = True
        elif net_acceleration_request_min > 0.3:
          self.permit_braking = False

        pcm_accel_cmd = float(np.clip(pcm_accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX))

        main_accel_cmd = 0. if self.CP.flags & ToyotaFlags.SECOC.value else pcm_accel_cmd
        can_sends.append(toyotacan.create_accel_command(self.packer, main_accel_cmd, pcm_cancel_cmd, self.permit_braking, self.standstill_req, lead,
                                                        CS.acc_type, fcw_alert, self.distance_button))
        if self.CP.flags & ToyotaFlags.SECOC.value:
          acc_cmd_2 = toyotacan.create_accel_command_2(self.packer, pcm_accel_cmd)
          acc_cmd_2 = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_acc_message_counter,
                              acc_cmd_2)
          self.secoc_acc_message_counter += 1
          can_sends.append(acc_cmd_2)

        self.accel = pcm_accel_cmd

    else:
      # we can spam can to cancel the system even if we are using lat only control
      if pcm_cancel_cmd:
        if self.CP.flags & ToyotaFlags.UNSUPPORTED_DSU:
          can_sends.append(toyotacan.create_acc_cancel_command(self.packer))
        else:
          can_sends.append(toyotacan.create_accel_command(self.packer, 0, pcm_cancel_cmd, True, False, lead, CS.acc_type, False, self.distance_button))

    # *** hud ui ***
    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      # ui mesg is at 1Hz but we send asap if:
      # - there is something to display
      # - there is something to stop displaying
      send_ui = False
      if ((fcw_alert or steer_alert) and not self.alert_active) or \
         (not (fcw_alert or steer_alert) and self.alert_active):
        send_ui = True
        self.alert_active = not self.alert_active
      elif pcm_cancel_cmd:
        # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
        send_ui = True

      if self.frame % 20 == 0 or send_ui:
        can_sends.append(toyotacan.create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
                                                     hud_control.rightLaneVisible, hud_control.leftLaneDepart,
                                                     hud_control.rightLaneDepart, CC.enabled, CS.lkas_hud))

      if (self.frame % 100 == 0 or send_ui) and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
        can_sends.append(toyotacan.create_fcw_command(self.packer, fcw_alert))

    # keep radar disabled
    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append(make_tester_present_msg(0x750, 0, 0xF))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
