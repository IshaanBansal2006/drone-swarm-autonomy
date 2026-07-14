"""Dempster-Shafer classification fusion.

YOU implement every method marked with NotImplementedError.
"""

from __future__ import annotations


# The frame of discernment — all possible classes.
# Extend this as your sim grows.
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
        """Build a mass function from a single sensor classification.

        Args:
            sensor_type: "camera", "radar", or "lidar"
            class_label: the class the sensor reported (None if no classification)
            confidence: sensor's confidence in that label, 0-1

        Returns:
            mass: mapping from subsets of classes (frozensets) to mass values.
                  Must sum to 1.0.

        Design guidance:
            - Camera: high confidence → mass on {class_label}, remainder on full frame (ignorance).
            - Radar: usually broad categories. E.g., "vehicle" → mass on {"vehicle"}, rest on Theta.
            - No classification (label=None): all mass on Theta (total ignorance).
            - The full frame Theta = frozenset(self.classes).
        """
        raise NotImplementedError("YOUR IMPLEMENTATION — mass function construction")

    def combine(
        self,
        m1: dict[frozenset[str], float],
        m2: dict[frozenset[str], float],
    ) -> dict[frozenset[str], float]:
        """Combine two mass functions using Dempster's rule of combination.

        Args:
            m1, m2: mass functions (frozenset -> float, summing to 1.0)

        Returns:
            combined mass function

        Steps:
            1. For each pair (A in m1, B in m2), compute:
               - intersection = A & B
               - product = m1[A] * m2[B]
            2. Sum products by intersection.
            3. Conflict K = sum of products where intersection is empty.
            4. Normalize: m_combined[C] = sum_of_products[C] / (1 - K) for non-empty C.
            5. If K >= 1.0, the sources totally conflict — handle gracefully.

        Zadeh's paradox: when K is high, normalization amplifies a
        potentially nonsensical result. Consider implementing Yager's rule
        as an alternative (assign conflict mass to Theta instead of normalizing).
        """
        raise NotImplementedError("YOUR IMPLEMENTATION — Dempster's combination rule")

    def belief(self, mass: dict[frozenset[str], float], hypothesis: frozenset[str]) -> float:
        """Compute belief for a hypothesis: sum of mass of all subsets of hypothesis.

        Bel(A) = sum(m(B) for B subset of A, B non-empty)
        """
        raise NotImplementedError("YOUR IMPLEMENTATION — belief")

    def plausibility(self, mass: dict[frozenset[str], float], hypothesis: frozenset[str]) -> float:
        """Compute plausibility: sum of mass of all sets that intersect hypothesis.

        Pl(A) = sum(m(B) for B where B & A is non-empty)
        """
        raise NotImplementedError("YOUR IMPLEMENTATION — plausibility")

    def decide(self, mass: dict[frozenset[str], float]) -> tuple[str, float]:
        """Pick the most likely class from a combined mass function.

        Returns:
            (class_label, confidence) — the singleton class with highest plausibility.
        """
        raise NotImplementedError("YOUR IMPLEMENTATION — decision rule")
