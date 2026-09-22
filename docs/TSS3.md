# Toyota TSS3 development scope

The current vehicle port is the repinned 2026 Camry Hybrid. The interface specifies that topology directly: chassis/state on bus 0, FRC source on bus 2, and radar on bus 1. It uses openpilot longitudinal control. There is no stock-harness alternative or topology eligibility detector.

Corolla platform registration and its separate control and safety branches have been removed. The shared TSS3 DBC remains intact. `test_tss3_corolla.py` retains captured Corolla steering, torque, pedal, and gear decoding evidence independently of vehicle support. The historical Camry fixtures remain available, and the working repin fixture is now consumed on its original buses rather than translated to stock-harness buses.

Following-distance selector changes do not adjust openpilot personality. The personality feedback adapter and its synthetic gap-button events have been removed. Other existing button behavior remains.

The Camry's existing authentication capability is still required. This cleanup removes the obsolete direct-steering helper; it does not change the retained authentication implementation or resolve the controller's dependence on its publication scheduling. That boundary remains a separate outstanding issue.

## Recorded-route validation

`opendbc/car/tests/routes.py` registers segment 2 of `5211e3c6c7b088d3/00000051--894db634a8`. Provenance, hash, recorded software version, and current local cache location are in `opendbc/car/toyota/tests/fixtures/repinned_camry_route.json`.

The existing `TestCarModelBase` CarParams, CarInterface, radar, safety RX, and CarState/safety agreement checks pass against this capture. The test loader itself is unchanged. The local cache contains the user-provided log; it has not been uploaded, and availability from remote CI is not established. The capture predates this cleanup and does not qualify new hardware behavior.

Both supported CAN input forms—one publication and a batch—use the same convention as CANParser. The standard model tests exposed the former mismatch in the custom TSS3 observation and radar paths.

## Panda changes

The experimental global sample-point change and coupling between host BRS and automatic FD selection have been restored to upstream behavior. Forwarded-frame preservation, host checksum/metadata handling, orientation mapping, and passive-connection support are separate existing changes and remain in the panda diff. No hardware timing or live vehicle test was performed during this cleanup.
