"""Dempster-Shafer classification fusion.

Ref: Shafer (1976), "A Mathematical Theory of Evidence".
"""

from __future__ import annotations

CLASSES: list[str] = ["vehicle", "person", "aircraft", "unknown"]


class DSClassifier:
    def __init__(self, classes: list[str] | None = None) -> None:
        self.classes = classes or CLASSES

    def build_mass_function(
        self,
        sensor_type: str,
        class_label: str | None,
        confidence: float,
    ) -> dict[frozenset[str], float]:
        """Construct a mass function from a single sensor classification.

        Returns mapping from focal elements (frozensets) to mass values, summing to 1.0.
        Theta = frozenset(self.classes) represents total ignorance.
        """
        raise NotImplementedError

    def combine(
        self,
        m1: dict[frozenset[str], float],
        m2: dict[frozenset[str], float],
    ) -> dict[frozenset[str], float]:
        """Dempster's rule of combination. Handles conflict via normalization."""
        raise NotImplementedError

    def belief(self, mass: dict[frozenset[str], float], hypothesis: frozenset[str]) -> float:
        """Bel(A) = sum of m(B) for all B that are subsets of A."""
        raise NotImplementedError

    def plausibility(self, mass: dict[frozenset[str], float], hypothesis: frozenset[str]) -> float:
        """Pl(A) = sum of m(B) for all B that intersect A."""
        raise NotImplementedError

    def decide(self, mass: dict[frozenset[str], float]) -> tuple[str, float]:
        """Max-plausibility decision rule. Returns (class_label, confidence)."""
        raise NotImplementedError
