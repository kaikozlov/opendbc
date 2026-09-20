import unittest
from unittest.mock import patch

from opendbc.car.fw_query_definitions import PlatformResolverContext
from opendbc.car.toyota.platform_resolver import load_data, resolve_all_regions, resolve_region
from opendbc.car.toyota.values import CAR, resolve_platform


class TestToyotaPlatformResolver(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.data = load_data()

  def test_current_p5_camry_uses_vin_source_category_and_phase(self):
    # Synthetic VIN satisfying Toyota's wildcarded 2026 Camry-HV decision row.
    matches = resolve_region(self.data, "NA", "JTDAA12K0T0123456", 0x7E8)
    camry = next(match for match in matches if match.vehicle_type == 12862)
    self.assertEqual(camry.name, "Camry HV")
    self.assertEqual(camry.source_keys, ((372, 0x12),))
    self.assertTrue(camry.resolution_complete)
    self.assertEqual(camry.resolver_stage, "vin_final")
    self.assertIn(498, camry.category_ids)

  def test_unknown_vin_source_does_not_degrade_to_vin_only(self):
    self.assertEqual(resolve_region(self.data, "NA", "JTDAA12K0T0123456", 0x123), ())

  def test_p3_p4_vin_hit_is_not_final(self):
    matches = resolve_region(self.data, "NA", "JTDBR12E031234567")
    legacy = [match for match in matches if match.resolver_stage == "requires_type41_vehicle_decision"]
    self.assertTrue(legacy)
    self.assertTrue(all(not match.resolution_complete for match in legacy))

  def test_all_region_resolution_retains_region_identity(self):
    matches = resolve_all_regions(self.data, "JTDAA12K0T0123456")
    self.assertTrue(matches)
    self.assertEqual({match.region for match in matches}, {match.region.upper() for match in matches})

  def test_curated_platform_bridge_maps_supported_camry(self):
    context = PlatformResolverContext(vin_rx_addr=0x7E8, vin_rx_bus=1)
    self.assertEqual(resolve_platform({}, "JTD1Z12K0N0123456", {}, context), {str(CAR.TOYOTA_CAMRY_TSS2)})

  def test_oem_identity_maps_exact_tss3_camry_platform(self):
    context = PlatformResolverContext(vin_rx_addr=0x7E8, vin_rx_bus=1)
    self.assertEqual(resolve_platform({}, "JTDAA12K0T0123456", {}, context), {str(CAR.TOYOTA_CAMRY_TSS3)})

  def test_ambiguous_identity_requires_every_candidate_to_be_mapped(self):
    context = PlatformResolverContext(vin_rx_addr=0x7E8, vin_rx_bus=1)
    supported = resolve_region(self.data, "NA", "JTD1Z12K0N0123456", 0x7E8)[0]
    unsupported = resolve_region(self.data, "NA", "JTDAA12K0T0123456", 0x7E8)[0]
    with patch("opendbc.car.toyota.platform_resolver.resolve_all_regions", return_value=(supported, unsupported)):
      self.assertEqual(resolve_platform({}, "JTD1Z12K0N0123456", {}, context), set())


if __name__ == "__main__":
  unittest.main()
