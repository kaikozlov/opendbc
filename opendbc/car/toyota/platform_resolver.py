"""Toyota's GTS-derived vehicle and installed-architecture resolver.

This module is intentionally independent of Toyota diagnostics. It turns the
same read-only identity context openpilot already collects at startup into an
OEM vehicle identity. Mapping that identity to an openpilot platform remains an
explicit compatibility decision in ``values.py``.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import cache
import gzip
from importlib import resources
import json
from typing import Any

from opendbc.car.vin import VIN_UNKNOWN, is_valid_vin


SCHEMA = "toyota-platform-resolver-v1"


@dataclass(frozen=True)
class ToyotaEcuRoute:
  category_id: int
  phase_type: int
  physical_request_address: int | None
  request_address: int | None
  sub_address: int | None
  transport_kind: str | None
  controller: str | None
  legislated_request_address: int | None


@dataclass(frozen=True)
class ToyotaVehicle:
  region: str
  vehicle_type: int
  name: str
  resolver_stage: str
  resolution_complete: bool
  source_keys: tuple[tuple[int, int], ...]
  architecture: tuple[ToyotaEcuRoute, ...]

  @property
  def category_ids(self) -> frozenset[int]:
    return frozenset(route.category_id for route in self.architecture)


@cache
def load_data() -> dict[str, Any]:
  resource = resources.files("opendbc.car.toyota.data").joinpath("platform_resolver.json.gz")
  with resource.open("rb") as raw:
    with gzip.GzipFile(fileobj=raw) as compressed:
      document = json.load(compressed)
  if document.get("schema") != SCHEMA:
    raise ValueError(f"unsupported Toyota platform resolver schema {document.get('schema')!r}")
  return document


def _vin_matches(row: list, vin: str) -> bool:
  _, _, flags, prefix_hex, _ = row
  prefix = bytes.fromhex(prefix_hex)
  vin11 = vin[:11].encode("ascii")
  return len(prefix) == len(vin11) == 11 and all(
    (int(flags) & (1 << index)) or expected == actual
    for index, (expected, actual) in enumerate(zip(prefix, vin11, strict=True))
  )


def _resolver_stage(region: dict, category_id: int) -> str:
  generation = region["category_generations"].get(str(category_id))
  return region["resolver_stages"].get(str(generation), "resolver_path_unresolved")


def _route(row: list) -> ToyotaEcuRoute:
  category_id, phase_type, physical, request, extension, transport, controller, legislated = row
  return ToyotaEcuRoute(
    category_id=int(category_id),
    phase_type=int(phase_type),
    physical_request_address=int(physical) if physical is not None else None,
    request_address=int(request) if request is not None else None,
    sub_address=int(extension) or None,
    transport_kind=str(transport) if transport else None,
    controller=str(controller) if controller else None,
    legislated_request_address=int(legislated) or None,
  )


def resolve_region(data: dict[str, Any], region_name: str, vin: str, vin_rx_addr: int | None = None) -> tuple[ToyotaVehicle, ...]:
  """Resolve Toyota vehicle candidates using the OEM VIN10 decision key.

  When the VIN response address identifies a Toyota source category/phase, it
  remains part of the key. A supplied but unknown source never silently falls
  back to VIN-only matching.
  """
  if vin == VIN_UNKNOWN or not is_valid_vin(vin):
    return ()
  region_key = region_name.upper()
  region = data["regions"].get(region_key)
  if not isinstance(region, dict):
    raise ValueError(f"unknown Toyota resolver region {region_key!r}")

  source_keys: set[tuple[int, int]] = set()
  if vin_rx_addr is not None:
    source_keys = {
      (int(item[0]), int(item[1]))
      for item in region.get("source_response_keys", {}).get(str(vin_rx_addr), [])
    }
    if not source_keys:
      return ()

  rows = [
    row for row in region["vin_rows"]
    if _vin_matches(row, vin) and (not source_keys or (int(row[0]), int(row[1])) in source_keys)
  ]
  by_vehicle: dict[int, list[list]] = {}
  for row in rows:
    by_vehicle.setdefault(int(row[4]), []).append(row)

  matches = []
  for vehicle_type, decision_rows in sorted(by_vehicle.items()):
    vehicle = region["vehicles"].get(str(vehicle_type))
    if vehicle is None:
      continue
    name, architecture_id = vehicle
    stages = {_resolver_stage(region, int(row[0])) for row in decision_rows}
    stage = next(iter(stages)) if len(stages) == 1 else "resolver_path_ambiguous"
    decision_source_keys = tuple(sorted({(int(row[0]), int(row[1])) for row in decision_rows}))
    architecture = tuple(
      _route(region["routes"][int(route_id)])
      for route_id in region["architectures"][int(architecture_id)]
    )
    matches.append(ToyotaVehicle(
      region=region_key,
      vehicle_type=vehicle_type,
      name=str(name),
      resolver_stage=stage,
      resolution_complete=stage == "vin_final",
      source_keys=decision_source_keys,
      architecture=architecture,
    ))
  return tuple(matches)


def resolve_all_regions(data: dict[str, Any], vin: str, vin_rx_addr: int | None = None) -> tuple[ToyotaVehicle, ...]:
  return tuple(
    match
    for region in sorted(data["regions"])
    for match in resolve_region(data, region, vin, vin_rx_addr)
  )
