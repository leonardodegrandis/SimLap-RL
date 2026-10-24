"""Check numerical order/error control against a known ODE solution."""

import math
import unittest
from dataclasses import dataclass, replace

from simlap.config import load_config, validate
from simlap.integration import integrate


@dataclass(frozen=True)
class Scalar:
    value: float

    def shifted(self, derivative, dt):
        return Scalar(self.value + dt * derivative.value)


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.settings = load_config().simulation

    def test_rk4_fourth_order_and_substeps(self):
        errors = []
        for h in (.2, .1):
            settings = replace(self.settings, max_internal_step_s=h)
            result, stats = integrate(lambda y: Scalar(y.value), Scalar(1), 1, settings)
            errors.append(abs(result.value - math.e))
            self.assertEqual(stats.accepted, round(1 / h))
            self.assertEqual(stats.evaluations, 4 * stats.accepted)
        self.assertGreater(errors[0] / errors[1], 14)
        self.assertLess(errors[0] / errors[1], 17)

    def test_adaptive_rejection_accuracy_and_tolerance_failure(self):
        settings = replace(self.settings, integrator="rk45", max_internal_step_s=1,
                           relative_tolerance=1e-9, absolute_tolerance=1e-11)
        result, stats = integrate(lambda y: Scalar(-20 * y.value), Scalar(1), .2, settings)
        self.assertAlmostEqual(result.value, math.exp(-4), delta=1e-10)
        self.assertGreater(stats.rejected, 0)
        self.assertLessEqual(stats.max_accepted_error, 1)
        self.assertEqual(stats.evaluations, 7 * (stats.accepted + stats.rejected))
        with self.assertRaisesRegex(RuntimeError, "min_internal_step"):
            integrate(lambda y: Scalar(-20 * y.value), Scalar(1), .2,
                      replace(settings, min_internal_step_s=.2))
        with self.assertRaisesRegex(RuntimeError, "max_internal_steps"):
            integrate(lambda y: Scalar(-20 * y.value), Scalar(1), .2,
                      replace(settings, max_internal_steps=1))

    def test_configuration_rejects_unknown_solver_and_invalid_step_bounds(self):
        config = load_config()
        for settings in (replace(self.settings, integrator="typo"), replace(self.settings, min_internal_step_s=1)):
            with self.assertRaises(ValueError):
                validate(replace(config, simulation=settings))
