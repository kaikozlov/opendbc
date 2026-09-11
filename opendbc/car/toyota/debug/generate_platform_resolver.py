#!/usr/bin/env python3
"""Build the compact Toyota platform resolver from a clean GTS+ bundle.

The input is the derived ``toyota-diagnostics-bundle-v2`` export, not Toyota
software. Only vehicle selection and installed-architecture metadata needed by
openpilot is retained. Diagnostic catalogs, sessions, DTCs, and operations are
deliberately excluded.
"""
from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path
import zipfile


SCHEMA = "toyota-platform-resolver-v1"
INPUT_SCHEMA = "toyota-diagnostics-bundle-v2"


def _route_endpoint(route: dict) -> list:
  return [
    route.get("physical_request_address"),
    route.get("request_address"),
    int(route.get("address_extension") or 0),
    route.get("transport_kind"),
    route.get("controller"),
    int(route.get("legislated_request_address") or 0),
  ]


def _architecture(region: dict, vehicle: dict) -> tuple[tuple, ...]:
  install_sets = region.get("install_sets", {})
  routes = region.get("routes", {})
  rows = set()
  for install_set_id in vehicle.get("install_set_ids", []):
    for item in install_sets.get(str(install_set_id), []):
      category_id = int(item["category_id"])
      phase_type = int(item["connection_phase_type"])
      route = routes.get(str(item.get("route_key")), {})
      rows.add((category_id, phase_type, *tuple(_route_endpoint(route))))
  return tuple(sorted(rows, key=lambda row: (row[0], row[1], str(row[2:]))))


def _compact_region(region: dict) -> dict:
  vehicle_architectures = {
    str(vehicle_type): _architecture(region, vehicle)
    for vehicle_type, vehicle in sorted(region["vehicles"].items(), key=lambda item: int(item[0]))
  }

  # A category/phase has one route within a regional master. Store each route
  # once, then express installed architectures as small route-ID sets.
  route_rows = sorted(
    {route for architecture in vehicle_architectures.values() for route in architecture},
    key=lambda row: (row[0], row[1], str(row[2:])),
  )
  route_ids = {route: index for index, route in enumerate(route_rows)}

  architectures: dict[tuple[int, ...], int] = {}
  architecture_rows: list[list[int]] = []
  vehicles: dict[str, list] = {}

  for vehicle_type, vehicle in sorted(region["vehicles"].items(), key=lambda item: int(item[0])):
    architecture = tuple(sorted(route_ids[route] for route in vehicle_architectures[str(vehicle_type)]))
    architecture_id = architectures.get(architecture)
    if architecture_id is None:
      architecture_id = len(architecture_rows)
      architectures[architecture] = architecture_id
      architecture_rows.append(list(architecture))
    vehicles[str(vehicle_type)] = [str(vehicle.get("name") or ""), architecture_id]

  category_generations = {
    str(category_id): category.get("generation_low5")
    for category_id, category in sorted(region["categories"].items(), key=lambda item: int(item[0]))
  }

  vin_rows = [
    [int(row["category_id"]), int(row["phase_type"]), int(row["flags"]), str(row["vin_prefix_hex"]), int(row["vehicle_type"])]
    for row in region["vin_decision"]["rows"]
  ]

  source_response_keys: dict[str, list[list[int]]] = {}
  source_keys = sorted({(row[0], row[1]) for row in vin_rows})
  for category_id, phase_type in source_keys:
    route = region["routes"].get(f"{category_id}:{phase_type}")
    if not isinstance(route, dict):
      continue
    request = int(route.get("legislated_request_address") or 0)
    if 0x7E0 <= request <= 0x7E7:
      source_response_keys.setdefault(str(request + 8), []).append([category_id, phase_type])

  dispatch = region.get("vehicle_resolver_dispatch", {})
  resolver_stages = {
    str(generation): "vin_final" if path in {"phase5", "phase6"} else "requires_type41_vehicle_decision"
    for generation, path in dispatch.get("vin10_generation_low5", {}).items()
  }
  resolver_stages.update({
    str(generation): "requires_legacy_vehicle_selector"
    for generation in dispatch.get("vin10_rejected_generation_low5", [])
  })

  return {
    "architectures": architecture_rows,
    "category_generations": category_generations,
    "resolver_stages": resolver_stages,
    "routes": [list(route) for route in route_rows],
    "source_response_keys": source_response_keys,
    "vehicles": vehicles,
    "vin_rows": vin_rows,
  }


def build(source: Path) -> dict:
  with zipfile.ZipFile(source) as archive:
    document = json.loads(archive.read("index.json"))
  if document.get("schema") != INPUT_SCHEMA:
    raise ValueError(f"expected {INPUT_SCHEMA}, got {document.get('schema')!r}")

  return {
    "release": document.get("release"),
    "regions": {
      name: _compact_region(region)
      for name, region in sorted(document["regions"].items())
    },
    "schema": SCHEMA,
    "source_identity": {
      "vehicle_resolver": document.get("source_identity", {}).get("vehicle_resolver"),
    },
  }


def write(document: dict, output: Path) -> None:
  payload = json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
  output.parent.mkdir(parents=True, exist_ok=True)
  with output.open("wb") as raw:
    with gzip.GzipFile(filename="", mode="wb", fileobj=raw, compresslevel=9, mtime=0) as compressed:
      compressed.write(payload)


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("bundle", type=Path)
  parser.add_argument("output", type=Path)
  args = parser.parse_args()
  write(build(args.bundle), args.output)


if __name__ == "__main__":
  main()
