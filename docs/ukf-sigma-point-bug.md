# Anatomy of a silent estimator bug: sigma points from the wrong triangle

**Found:** 2026-07-29, while implementing the square-root UKF (decision 014).
**Lived in:** `src/swarm_autonomy/edge/ukf.py`, `generate_sigma_points` — since the UKF's first
implementation (2026-07-13).
**Class:** silent numerical/math bug — no crash, no exception, plausible outputs, wrong math.
**Fix:** one line. **Understanding it:** the point of this document.

---

## 1. What the code did

```python
sqrt_P = np.linalg.cholesky((n + lam) * P)     # lower-triangular L, L @ L.T = (n+lam)P
sigma[i + 1]     = x + sqrt_P[i]               # ROW i of L  <-- the bug
sigma[n + i + 1] = x - sqrt_P[i]
```

Sigma points were spread along the **rows** of the Cholesky factor `L`.

## 2. What the math requires

The 2n+1 sigma points must encode the covariance exactly: the weighted sum of their
deviation outer-products has to reconstruct `P`,

&nbsp;&nbsp;&nbsp;&nbsp;Σᵢ Wᵢ (χᵢ − x̄)(χᵢ − x̄)ᵀ = P.

With offsets `±sᵢ` taken from a matrix `M`, that sum is Σᵢ sᵢsᵢᵀ. Now the crux:

- if the `sᵢ` are the **columns** of `M`, then Σ sᵢsᵢᵀ = **M Mᵀ**
- if the `sᵢ` are the **rows** of `M`, then Σ sᵢsᵢᵀ = **Mᵀ M**

`np.linalg.cholesky` returns **lower**-triangular `L` defined by `L Lᵀ = P`. So:

- columns of `L` → reconstruct `L Lᵀ = P` ✅
- rows of `L` → reconstruct `Lᵀ L` ❌ — generally **≠ P**

`LᵀL` is symmetric, positive-definite, has the *same eigenvalues* as `P` (they're similar
matrices), the same trace, the same determinant — but its **eigenvectors are rotated**. The
sigma cloud had the right total "size" in every invariant sense, and the wrong *shape*: its
correlation directions didn't match `P`'s.

### A 2×2 worked example

Take `P = [[4, 2], [2, 2]]`. Cholesky: `L = [[2, 0], [1, 1]]` (check: `LLᵀ = P` ✓).

- **Columns** of `L`: (2, 1) and (0, 1) → Σ ccᵀ = [[4,2],[2,2]] = `P` ✓
- **Rows** of `L`: (2, 0) and (1, 1) → Σ rrᵀ = [[4,0],[0,0]] + [[1,1],[1,1]] =
  **[[5, 1], [1, 1]]** ≠ P ✗

Same trace (6), same determinant (4), same eigenvalues — different matrix. The spread says
"x₁ is more uncertain and barely correlated with x₂" when the truth is "strongly correlated."
That's the bug in miniature: **every predict and update ran on a rotated belief.**

## 3. Why it stayed hidden (five reasons)

1. **No invariant it violated was checked.** `LᵀL` is symmetric PSD with the right scale —
   nothing crashes, nothing NaNs, tracks converge.
2. **The filter is self-consistent in the error.** Predict *reconstructs* covariance from the
   same (mis-shaped) sigma cloud, so each cycle is internally coherent — the rotation just
   quietly degrades the fusion's use of cross-correlations.
3. **Weak nonlinearity forgives.** For near-linear `h` and mild `P`, the rotated spread gives
   nearly the right answers; errors hide inside measurement noise.
4. **The strong-nonlinearity symptom was misattributed.** When the camera model's
   perspective+yaw nonlinearity DID interact badly with the mis-shaped spread, the visible
   symptom was `P` going indefinite — which looked like (and partially was) the well-known
   `P − KSKᵀ` fragility, so the guards went there. Post-fix, PSD repairs dropped to **zero
   in all benchmark scenarios including stress** — strong evidence the rows/cols bug was a
   primary driver of the instability, not just a passenger.
5. **Precedent bias.** Spreading along "rows of the Cholesky result" is a real pattern in
   well-known code (e.g. FilterPy) — but with **scipy**'s Cholesky, whose default is
   **upper**-triangular `R` (`RᵀR = P`), for which rows are *correct*. Porting the row idiom
   to numpy's **lower** factor flips it into a bug. Same words, different convention.

## 4. How it was found

Implementing the square-root UKF forced explicit reasoning about factor conventions —
SR-UKF's entire state *is* the factor, so "columns of a lower factor / which triangle does
this library return" had to be answered precisely. Auditing the standard UKF against that
answer exposed the row indexing. The lesson generalizes: **re-deriving a component from a
different formulation is a powerful audit of the original.**

## 5. The fix and the regression lock

```python
L = np.linalg.cholesky(scaled)   # lower: L @ L.T = scaled
sqrt_P = L.T                     # rows of L.T == columns of L  -> correct offsets
```

Locked by `tests/test_ukf.py::test_sigma_points_reproduce_covariance`: build a random,
strongly-correlated SPD `P`, generate the sigma set, reconstruct Σ Wᵢ(χᵢ−x̄)(χᵢ−x̄)ᵀ, and
require it to equal `P` to 1e-8. Rows-of-`L` fails this loudly; the fix passes exactly.
(The test uses a *correlated* `P` deliberately — for diagonal `P`, `LᵀL = LLᵀ` and the bug is
invisible. Test data must exercise the failure mode.)

## 6. Consequences observed after the fix

- PSD repairs: previously required (crash → jitter → eigen-floor escalation); now **0**
  across 20-seed nominal *and* stress benchmarks for all three covariance strategies.
- Filter accuracy in the nominal scenario was similar before/after (reason 3 above), but the
  stress-scenario stability that motivated the whole D-B2 investigation improved to the
  point where the strategy choice became a margins question (see decision `014`).

## 7. Whiteboard rules to keep

1. For a factor `M` with `MMᵀ = P`, sigma offsets are the **columns** of `M`.
2. **numpy** `cholesky` → lower `L` (`LLᵀ = P`): use columns (or rows of `Lᵀ`).
   **scipy** `cholesky` default → upper `R` (`RᵀR = P`): rows work. Name the convention
   before writing the indexing.
3. Symmetric-PSD-and-right-scale is not right: `LᵀL` shares eigenvalues, trace, and
   determinant with `P` and is still the wrong matrix. Test *reconstruction*, not health.
4. A "numerical stability" symptom can be a *correctness* bug wearing a disguise. When
   guards keep firing, audit the math upstream of the guard.
5. Regression tests for shape/direction bugs need **correlated** test matrices — diagonal
   cases can't tell the two triangles apart.
