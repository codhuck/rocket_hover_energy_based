from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np
import yaml

from src.simulation import simulate


class SimulationMetricsTest(unittest.TestCase):
    def test_summary_contains_phi_error_band_settling_time(self) -> None:
        config_path = Path("configs/default.yaml")
        with config_path.open("r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        result = simulate(cfg)

        phi_error_band_rad = 0.1
        phi_error = np.abs(result.controls["e_phi"])

        expected_settling_time = None
        for i, t in enumerate(result.t):
            if np.all(phi_error[i:] < phi_error_band_rad):
                expected_settling_time = float(t)
                break

        self.assertIn("phi_error_band_rad", result.summary)
        self.assertIn("settling_time_phi_error_band_s", result.summary)
        self.assertAlmostEqual(result.summary["phi_error_band_rad"], phi_error_band_rad)
        self.assertAlmostEqual(
            result.summary["settling_time_phi_error_band_s"],
            expected_settling_time,
        )


if __name__ == "__main__":
    unittest.main()
