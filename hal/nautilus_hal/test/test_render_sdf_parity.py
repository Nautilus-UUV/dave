"""Drift guard: rendering model.sdf.jinja with HydrodynamicsSpec()
defaults must produce a byte-identical copy of the canonical model.sdf.

The template and the canonical SDF live next to each other in
`dave_robot_models/description/glider_nautilus/`. They share authorship
in lockstep: change one, change the other. This test exists so the
canonical SDF (what *runs* in the default flow) and the template (what
*runs* during sampled parameter sweeps) can't silently diverge — drift
in either file fails CI before it can mask a sweep run.

A second test perturbs the spec and confirms the perturbations actually
land in the rendered XML (and that the canonical values disappear),
catching the dual failure mode where a placeholder is wired to the
wrong field.

Tier 3 marker (`@pytest.mark.sim`) — the test reads installed
`dave_robot_models` artifacts (`model.sdf`, `model.sdf.jinja`), so it
needs the sim-side share dirs available. It does not launch Gazebo.
"""

from __future__ import annotations

import os

import pytest
from ament_index_python.packages import get_package_share_directory
from nautilus_hal.render_sdf import _render
from py_pkg.scenarios.spec.rig import FinAeroSpec, HydrodynamicsSpec


def _model_dir() -> str:
    return os.path.join(
        get_package_share_directory("dave_robot_models"),
        "description",
        "glider_nautilus",
    )


@pytest.mark.sim
def test_template_with_defaults_matches_canonical_sdf():
    template = os.path.join(_model_dir(), "model.sdf.jinja")
    canonical = os.path.join(_model_dir(), "model.sdf")

    rendered = _render(HydrodynamicsSpec(), template)
    with open(canonical) as f:
        expected = f.read()

    assert rendered == expected, (
        "Rendered template diverges from canonical model.sdf. Either the "
        "template lost the lockstep edit, or HydrodynamicsSpec defaults "
        "no longer mirror the SDF's hardcoded values."
    )


@pytest.mark.sim
def test_template_with_perturbed_spec_surfaces_in_xml():
    template = os.path.join(_model_dir(), "model.sdf.jinja")

    spec = HydrodynamicsSpec(
        drag_zW=-200.0,
        added_mass_xx=7.5,
        left_fin=FinAeroSpec(cda=0.35, area=0.0725),
        fluid_density=1012.5,
        drag_zWabsW=-55.0,
        trim_mass_bow=0.25,
        trim_mass_bladder=0.125,
        bladder_spawn_volume_m3=0.0021,
    )
    rendered = _render(spec, template)

    for token in (
        "<zW>-200</zW>",
        "<xx>7.5</xx>",
        "<cda>0.35</cda>",
        "<zWabsW>-55</zWabsW>",
        "<mass>0.25</mass>",
        "<mass>0.125</mass>",
        "<default_volume>0.0021</default_volume>",
    ):
        assert token in rendered, f"perturbed value {token!r} missing from rendered XML"

    # fluid_density feeds all five density leaves: Hydrodynamics
    # <water_density>, BuoyancyEngine <fluid_density>, 3x LiftDrag
    # <air_density>.
    assert rendered.count("1012.5") == 5, (
        "fluid_density should surface in exactly 5 density leaves, found "
        f"{rendered.count('1012.5')}"
    )

    for token in (
        "<zW>-162</zW>",
        "<xx>5.30023995</xx>",
        "<zWabsW>0</zWabsW>",
        "<default_volume>0.0022</default_volume>",
    ):
        assert token not in rendered, (
            f"canonical default {token!r} survived in rendered XML "
            "(placeholder wired to the wrong field?)"
        )
