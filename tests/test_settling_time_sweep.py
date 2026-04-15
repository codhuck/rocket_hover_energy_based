from __future__ import annotations

from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from src.settling_time_sweep import (
    K_OMEGA_VALUES,
    K_PHI_VALUES,
    generate_sweep_rows,
    save_sweep_outputs,
)


class SettlingTimeSweepTest(unittest.TestCase):
    def test_generate_sweep_rows_covers_full_regular_grid(self) -> None:
        with patch("src.settling_time_sweep.run_case") as run_case:
            run_case.side_effect = lambda base_cfg, k_phi, k_omega: {
                "k_phi": float(k_phi),
                "k_omega": float(k_omega),
                "phi_error_band_rad": 0.1,
                "settling_time_phi_error_band_s": float(k_phi + k_omega),
            }

            rows = generate_sweep_rows({"controller": {}}, K_PHI_VALUES, K_OMEGA_VALUES)

        self.assertEqual(len(rows), 60)
        self.assertEqual(rows[0]["k_phi"], 6.0)
        self.assertEqual(rows[0]["k_omega"], 2.0)
        self.assertEqual(rows[-1]["k_phi"], 24.0)
        self.assertEqual(rows[-1]["k_omega"], 7.0)

    def test_save_sweep_outputs_writes_csv_json_and_plot(self) -> None:
        rows = [
            {
                "k_phi": 6.0,
                "k_omega": 2.0,
                "phi_error_band_rad": 0.1,
                "settling_time_phi_error_band_s": 1.23,
            },
            {
                "k_phi": 8.0,
                "k_omega": 2.0,
                "phi_error_band_rad": 0.1,
                "settling_time_phi_error_band_s": 1.11,
            },
            {
                "k_phi": 6.0,
                "k_omega": 3.0,
                "phi_error_band_rad": 0.1,
                "settling_time_phi_error_band_s": 0.98,
            },
            {
                "k_phi": 8.0,
                "k_omega": 3.0,
                "phi_error_band_rad": 0.1,
                "settling_time_phi_error_band_s": 0.91,
            },
        ]

        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            save_sweep_outputs(rows, output_dir)

            self.assertTrue((output_dir / "settling_time_grid.csv").exists())
            self.assertTrue((output_dir / "settling_time_grid.json").exists())
            self.assertTrue((output_dir / "settling_time_surface.png").exists())


if __name__ == "__main__":
    unittest.main()
