"""Numerics kept dependency-free for quick simulator startup."""

from bisect import bisect_right


def clip(value, lower, upper):
    return max(lower, min(upper, value))


def interp(x, xs, ys):
    i = max(0, min(len(xs) - 2, bisect_right(xs, x) - 1))
    fraction = clip((x - xs[i]) / (xs[i + 1] - xs[i]), 0, 1)
    return ys[i] + fraction * (ys[i + 1] - ys[i])


def solve_linear(matrix, rhs):
    """Pivoted elimination for the three-variable cornering trim."""
    rows = [list(row) + [b] for row, b in zip(matrix, rhs)]
    n = len(rhs)
    for i in range(n):
        pivot = max(range(i, n), key=lambda j: abs(rows[j][i]))
        rows[i], rows[pivot] = rows[pivot], rows[i]
        if abs(rows[i][i]) < 1e-12:
            raise ValueError("Singular trim: requested corner may be infeasible")
        scale = rows[i][i]
        rows[i] = [x / scale for x in rows[i]]
        for j in range(n):
            if j != i:
                scale = rows[j][i]
                rows[j] = [a - scale * b for a, b in zip(rows[j], rows[i])]
    return [row[-1] for row in rows]
