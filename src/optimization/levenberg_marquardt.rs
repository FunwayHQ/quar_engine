//! Levenberg-Marquardt optimizer for non-linear least squares.
//!
//! The LM algorithm blends gradient descent and Gauss-Newton:
//! - High damping (λ) → gradient descent (robust but slow)
//! - Low damping (λ) → Gauss-Newton (fast but may diverge)
//!
//! The algorithm adaptively adjusts λ based on improvement.

/// Configuration for the LM optimizer.
#[derive(Debug, Clone)]
pub struct LMConfig {
    /// Initial damping parameter
    pub initial_lambda: f64,
    /// Factor to increase lambda when step fails
    pub lambda_up: f64,
    /// Factor to decrease lambda when step succeeds
    pub lambda_down: f64,
    /// Maximum damping parameter
    pub max_lambda: f64,
    /// Minimum damping parameter
    pub min_lambda: f64,
    /// Maximum number of iterations
    pub max_iterations: usize,
    /// Convergence tolerance for cost improvement
    pub cost_tolerance: f64,
    /// Convergence tolerance for parameter change
    pub param_tolerance: f64,
    /// Convergence tolerance for gradient norm
    pub gradient_tolerance: f64,
}

impl Default for LMConfig {
    fn default() -> Self {
        Self {
            initial_lambda: 1e-3,
            lambda_up: 10.0,
            lambda_down: 0.1,
            max_lambda: 1e10,
            min_lambda: 1e-10,
            max_iterations: 50,
            cost_tolerance: 1e-8,
            param_tolerance: 1e-8,
            gradient_tolerance: 1e-8,
        }
    }
}

/// Result of LM optimization.
#[derive(Debug, Clone)]
pub struct LMResult {
    /// Final parameters
    pub params: Vec<f64>,
    /// Final cost (sum of squared residuals)
    pub cost: f64,
    /// Number of iterations performed
    pub iterations: usize,
    /// Whether optimization converged
    pub converged: bool,
    /// Reason for termination
    pub termination_reason: TerminationReason,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TerminationReason {
    /// Cost tolerance reached
    CostConverged,
    /// Parameter tolerance reached
    ParamConverged,
    /// Gradient tolerance reached
    GradientConverged,
    /// Maximum iterations reached
    MaxIterations,
    /// Lambda exceeded maximum (stuck)
    LambdaExceeded,
    /// Invalid residuals (NaN or Inf)
    InvalidResiduals,
}

/// Levenberg-Marquardt optimizer.
pub struct LMOptimizer {
    config: LMConfig,
}

impl LMOptimizer {
    pub fn new(config: LMConfig) -> Self {
        Self { config }
    }

    pub fn with_defaults() -> Self {
        Self::new(LMConfig::default())
    }

    /// Optimize parameters using the LM algorithm.
    ///
    /// # Arguments
    /// * `initial_params` - Initial parameter values
    /// * `residual_fn` - Function that computes residuals for given parameters
    /// * `jacobian_fn` - Function that computes Jacobian matrix for given parameters
    ///
    /// # Returns
    /// Optimization result containing final parameters and convergence info
    pub fn optimize<R, J>(
        &self,
        initial_params: &[f64],
        residual_fn: R,
        jacobian_fn: J,
    ) -> LMResult
    where
        R: Fn(&[f64]) -> Vec<f64>,
        J: Fn(&[f64]) -> Vec<Vec<f64>>,
    {
        let mut params = initial_params.to_vec();
        let mut lambda = self.config.initial_lambda;

        // Initial cost
        let mut residuals = residual_fn(&params);
        if !is_valid_residuals(&residuals) {
            return LMResult {
                params,
                cost: f64::MAX,
                iterations: 0,
                converged: false,
                termination_reason: TerminationReason::InvalidResiduals,
            };
        }
        let mut cost = compute_cost(&residuals);

        for iter in 0..self.config.max_iterations {
            // Compute Jacobian
            let jacobian = jacobian_fn(&params);

            // Compute JᵀJ and Jᵀr
            let (jtj, jtr) = compute_normal_equations(&jacobian, &residuals);

            // Check gradient convergence
            let grad_norm = jtr.iter().map(|x| x * x).sum::<f64>().sqrt();
            if grad_norm < self.config.gradient_tolerance {
                return LMResult {
                    params,
                    cost,
                    iterations: iter,
                    converged: true,
                    termination_reason: TerminationReason::GradientConverged,
                };
            }

            // Try to find a good step
            loop {
                // Solve (JᵀJ + λI)δ = -Jᵀr
                let delta = solve_damped_normal_equations(&jtj, &jtr, lambda);

                // Check parameter change
                let delta_norm = delta.iter().map(|x| x * x).sum::<f64>().sqrt();
                let param_norm = params.iter().map(|x| x * x).sum::<f64>().sqrt().max(1.0);

                if delta_norm / param_norm < self.config.param_tolerance {
                    return LMResult {
                        params,
                        cost,
                        iterations: iter,
                        converged: true,
                        termination_reason: TerminationReason::ParamConverged,
                    };
                }

                // Try the step
                let new_params: Vec<f64> = params.iter().zip(delta.iter())
                    .map(|(p, d)| p + d)
                    .collect();

                let new_residuals = residual_fn(&new_params);
                if !is_valid_residuals(&new_residuals) {
                    // Increase damping and try again
                    lambda *= self.config.lambda_up;
                    if lambda > self.config.max_lambda {
                        return LMResult {
                            params,
                            cost,
                            iterations: iter,
                            converged: false,
                            termination_reason: TerminationReason::LambdaExceeded,
                        };
                    }
                    continue;
                }

                let new_cost = compute_cost(&new_residuals);

                // Check if step improved cost
                if new_cost < cost {
                    // Accept step
                    let improvement = cost - new_cost;
                    params = new_params;
                    residuals = new_residuals;
                    cost = new_cost;

                    // Decrease damping (more Gauss-Newton like)
                    lambda = (lambda * self.config.lambda_down).max(self.config.min_lambda);

                    // Check cost convergence
                    if improvement / cost.max(1e-10) < self.config.cost_tolerance {
                        return LMResult {
                            params,
                            cost,
                            iterations: iter + 1,
                            converged: true,
                            termination_reason: TerminationReason::CostConverged,
                        };
                    }

                    break; // Move to next iteration
                } else {
                    // Reject step, increase damping
                    lambda *= self.config.lambda_up;
                    if lambda > self.config.max_lambda {
                        return LMResult {
                            params,
                            cost,
                            iterations: iter,
                            converged: false,
                            termination_reason: TerminationReason::LambdaExceeded,
                        };
                    }
                }
            }
        }

        LMResult {
            params,
            cost,
            iterations: self.config.max_iterations,
            converged: false,
            termination_reason: TerminationReason::MaxIterations,
        }
    }
}

/// Compute sum of squared residuals.
fn compute_cost(residuals: &[f64]) -> f64 {
    residuals.iter().map(|r| r * r).sum::<f64>() * 0.5
}

/// Check if residuals are valid (no NaN or Inf).
fn is_valid_residuals(residuals: &[f64]) -> bool {
    residuals.iter().all(|r| r.is_finite())
}

/// Compute normal equations: JᵀJ and Jᵀr.
fn compute_normal_equations(jacobian: &[Vec<f64>], residuals: &[f64]) -> (Vec<Vec<f64>>, Vec<f64>) {
    if jacobian.is_empty() || jacobian[0].is_empty() {
        return (vec![], vec![]);
    }

    let num_params = jacobian[0].len();
    let num_residuals = jacobian.len();

    // JᵀJ is num_params x num_params
    let mut jtj = vec![vec![0.0; num_params]; num_params];
    // Jᵀr is num_params x 1
    let mut jtr = vec![0.0; num_params];

    // Compute JᵀJ and Jᵀr
    #[allow(clippy::needless_range_loop)]
    for i in 0..num_params {
        for j in 0..=i {
            let mut sum = 0.0;
            for row in &jacobian[..num_residuals] {
                sum += row[i] * row[j];
            }
            jtj[i][j] = sum;
            jtj[j][i] = sum; // Symmetric
        }

        let mut sum = 0.0;
        for (k, row) in jacobian[..num_residuals].iter().enumerate() {
            sum += row[i] * residuals[k];
        }
        jtr[i] = -sum; // Note: we want to minimize, so -Jᵀr
    }

    (jtj, jtr)
}

/// Solve damped normal equations: (JᵀJ + λI)δ = b.
fn solve_damped_normal_equations(jtj: &[Vec<f64>], b: &[f64], lambda: f64) -> Vec<f64> {
    if jtj.is_empty() || b.is_empty() {
        return vec![];
    }

    // Add damping to diagonal
    let mut a = jtj.to_vec();
    for (i, row) in a.iter_mut().enumerate() {
        row[i] += lambda;
    }

    // Solve using Cholesky decomposition (A is symmetric positive definite)
    // Fall back to LU if Cholesky fails
    if let Some(x) = solve_cholesky(&a, b) {
        return x;
    }

    // Fallback: simple Gaussian elimination
    solve_gaussian(&a, b)
}

/// Solve Ax = b using Cholesky decomposition.
#[allow(clippy::needless_range_loop)]
fn solve_cholesky(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let n = a.len();

    // Compute Cholesky decomposition A = LLᵀ
    let mut l = vec![vec![0.0; n]; n];

    for i in 0..n {
        for j in 0..=i {
            let mut sum = 0.0;
            for k in 0..j {
                sum += l[i][k] * l[j][k];
            }

            if i == j {
                let val = a[i][i] - sum;
                if val <= 0.0 {
                    return None; // Not positive definite
                }
                l[i][j] = val.sqrt();
            } else {
                if l[j][j].abs() < 1e-15 {
                    return None;
                }
                l[i][j] = (a[i][j] - sum) / l[j][j];
            }
        }
    }

    // Solve Ly = b (forward substitution)
    let mut y = vec![0.0; n];
    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..i {
            sum += l[i][j] * y[j];
        }
        if l[i][i].abs() < 1e-15 {
            return None;
        }
        y[i] = (b[i] - sum) / l[i][i];
    }

    // Solve Lᵀx = y (backward substitution)
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = 0.0;
        for j in (i + 1)..n {
            sum += l[j][i] * x[j];
        }
        if l[i][i].abs() < 1e-15 {
            return None;
        }
        x[i] = (y[i] - sum) / l[i][i];
    }

    Some(x)
}

/// Solve Ax = b using Gaussian elimination with partial pivoting.
#[allow(clippy::needless_range_loop)]
fn solve_gaussian(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let n = a.len();
    if n == 0 {
        return vec![];
    }

    // Augmented matrix
    let mut aug: Vec<Vec<f64>> = a.iter()
        .zip(b.iter())
        .map(|(row, &bi)| {
            let mut r = row.clone();
            r.push(bi);
            r
        })
        .collect();

    // Forward elimination with partial pivoting
    for i in 0..n {
        // Find pivot
        let mut max_row = i;
        let mut max_val = aug[i][i].abs();
        for k in (i + 1)..n {
            if aug[k][i].abs() > max_val {
                max_val = aug[k][i].abs();
                max_row = k;
            }
        }

        // Swap rows
        aug.swap(i, max_row);

        // Check for singular matrix
        if aug[i][i].abs() < 1e-15 {
            continue;
        }

        // Eliminate column
        for k in (i + 1)..n {
            let factor = aug[k][i] / aug[i][i];
            for j in i..=n {
                aug[k][j] -= factor * aug[i][j];
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        if aug[i][i].abs() < 1e-15 {
            continue;
        }
        let mut sum = aug[i][n];
        for j in (i + 1)..n {
            sum -= aug[i][j] * x[j];
        }
        x[i] = sum / aug[i][i];
    }

    x
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_cost() {
        let residuals = vec![1.0, 2.0, 3.0];
        let cost = compute_cost(&residuals);
        assert!((cost - 7.0).abs() < 1e-10); // 0.5 * (1 + 4 + 9) = 7
    }

    #[test]
    fn test_solve_cholesky() {
        // Simple 2x2 positive definite matrix
        let a = vec![
            vec![4.0, 2.0],
            vec![2.0, 3.0],
        ];
        let b = vec![1.0, 2.0];

        let x = solve_cholesky(&a, &b).unwrap();

        // Verify Ax = b
        let ax0 = a[0][0] * x[0] + a[0][1] * x[1];
        let ax1 = a[1][0] * x[0] + a[1][1] * x[1];

        assert!((ax0 - b[0]).abs() < 1e-10);
        assert!((ax1 - b[1]).abs() < 1e-10);
    }

    #[test]
    fn test_solve_gaussian() {
        let a = vec![
            vec![2.0, 1.0],
            vec![1.0, 3.0],
        ];
        let b = vec![3.0, 4.0];

        let x = solve_gaussian(&a, &b);

        // Verify Ax = b
        let ax0 = a[0][0] * x[0] + a[0][1] * x[1];
        let ax1 = a[1][0] * x[0] + a[1][1] * x[1];

        assert!((ax0 - b[0]).abs() < 1e-10);
        assert!((ax1 - b[1]).abs() < 1e-10);
    }

    #[test]
    fn test_lm_quadratic() {
        // Minimize f(x) = (x - 3)^2 + (y - 2)^2
        // Optimal: x = 3, y = 2
        let optimizer = LMOptimizer::with_defaults();

        let residual_fn = |params: &[f64]| {
            vec![params[0] - 3.0, params[1] - 2.0]
        };

        let jacobian_fn = |_params: &[f64]| {
            vec![
                vec![1.0, 0.0],
                vec![0.0, 1.0],
            ]
        };

        let result = optimizer.optimize(&[0.0, 0.0], residual_fn, jacobian_fn);

        assert!(result.converged);
        assert!((result.params[0] - 3.0).abs() < 1e-6);
        assert!((result.params[1] - 2.0).abs() < 1e-6);
        assert!(result.cost < 1e-10);
    }

    #[test]
    fn test_lm_rosenbrock() {
        // Rosenbrock function: f(x,y) = (1-x)^2 + 100(y-x^2)^2
        // Optimal: x = 1, y = 1
        let optimizer = LMOptimizer::new(LMConfig {
            max_iterations: 200,
            ..Default::default()
        });

        let residual_fn = |params: &[f64]| {
            let x = params[0];
            let y = params[1];
            vec![
                1.0 - x,
                10.0 * (y - x * x),
            ]
        };

        let jacobian_fn = |params: &[f64]| {
            let x = params[0];
            vec![
                vec![-1.0, 0.0],
                vec![-20.0 * x, 10.0],
            ]
        };

        let result = optimizer.optimize(&[0.0, 0.0], residual_fn, jacobian_fn);

        assert!(result.converged, "Rosenbrock did not converge: {:?}", result.termination_reason);
        assert!((result.params[0] - 1.0).abs() < 1e-4);
        assert!((result.params[1] - 1.0).abs() < 1e-4);
    }

    #[test]
    fn test_lm_exponential() {
        // Fit y = a * exp(-b * x) to data
        // True: a = 2, b = 0.5
        let x_data = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y_data: Vec<f64> = x_data.iter().map(|&x: &f64| 2.0 * (-0.5 * x).exp()).collect();

        let optimizer = LMOptimizer::with_defaults();

        let residual_fn = |params: &[f64]| {
            let a = params[0];
            let b = params[1];
            x_data.iter().zip(y_data.iter())
                .map(|(&x, &y)| a * (-b * x).exp() - y)
                .collect()
        };

        let jacobian_fn = |params: &[f64]| {
            let a = params[0];
            let b = params[1];
            x_data.iter()
                .map(|&x| {
                    let exp_bx = (-b * x).exp();
                    vec![exp_bx, -a * x * exp_bx]
                })
                .collect()
        };

        let result = optimizer.optimize(&[1.0, 1.0], residual_fn, jacobian_fn);

        assert!(result.converged);
        assert!((result.params[0] - 2.0).abs() < 1e-4, "a = {} (expected 2)", result.params[0]);
        assert!((result.params[1] - 0.5).abs() < 1e-4, "b = {} (expected 0.5)", result.params[1]);
    }

    #[test]
    fn test_lm_config() {
        let config = LMConfig {
            max_iterations: 10,
            cost_tolerance: 1e-6,
            ..Default::default()
        };

        assert_eq!(config.max_iterations, 10);
        assert!((config.cost_tolerance - 1e-6).abs() < 1e-15);
    }

    #[test]
    fn test_lm_max_iterations() {
        // Problem that takes many iterations
        let optimizer = LMOptimizer::new(LMConfig {
            max_iterations: 2,
            ..Default::default()
        });

        let residual_fn = |params: &[f64]| vec![params[0] - 100.0];
        let jacobian_fn = |_params: &[f64]| vec![vec![1.0]];

        let result = optimizer.optimize(&[0.0], residual_fn, jacobian_fn);

        // Should terminate but not converge to optimal
        assert_eq!(result.termination_reason, TerminationReason::MaxIterations);
    }

    #[test]
    fn test_normal_equations() {
        // J = [[1, 2], [3, 4]], r = [1, 2]
        let jacobian = vec![
            vec![1.0, 2.0],
            vec![3.0, 4.0],
        ];
        let residuals = vec![1.0, 2.0];

        let (jtj, jtr) = compute_normal_equations(&jacobian, &residuals);

        // JᵀJ = [[10, 14], [14, 20]]
        assert!((jtj[0][0] - 10.0).abs() < 1e-10);
        assert!((jtj[0][1] - 14.0).abs() < 1e-10);
        assert!((jtj[1][0] - 14.0).abs() < 1e-10);
        assert!((jtj[1][1] - 20.0).abs() < 1e-10);

        // -Jᵀr = -[7, 10]
        assert!((jtr[0] - (-7.0)).abs() < 1e-10);
        assert!((jtr[1] - (-10.0)).abs() < 1e-10);
    }
}
