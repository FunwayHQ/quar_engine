//! Histogram Computation for Lighting Estimation
//!
//! Provides efficient histogram computation for luminance analysis.

/// Compute a 256-bin luminance histogram from grayscale image data.
///
/// # Arguments
/// * `gray` - Grayscale pixel values (0-255)
///
/// # Returns
/// Array of 256 counts, one per intensity level
pub fn compute_histogram(gray: &[u8]) -> [u32; 256] {
    let mut hist = [0u32; 256];
    for &pixel in gray {
        hist[pixel as usize] += 1;
    }
    hist
}

/// Compute the weighted mean of a histogram (average luminance).
///
/// # Arguments
/// * `hist` - 256-bin histogram
///
/// # Returns
/// Average intensity normalized to 0.0-1.0
pub fn histogram_mean(hist: &[u32; 256]) -> f32 {
    let total: u64 = hist.iter().map(|&c| c as u64).sum();
    if total == 0 {
        return 0.0;
    }

    let weighted: u64 = hist
        .iter()
        .enumerate()
        .map(|(i, &c)| i as u64 * c as u64)
        .sum();

    weighted as f32 / total as f32 / 255.0
}

/// Compute the standard deviation of a histogram.
///
/// Used to measure spread/confidence of lighting estimate.
///
/// # Arguments
/// * `hist` - 256-bin histogram
/// * `mean` - Pre-computed mean (0-255 scale, not normalized)
///
/// # Returns
/// Standard deviation in 0-255 scale
pub fn histogram_std_dev(hist: &[u32; 256], mean: f32) -> f32 {
    let total: u64 = hist.iter().map(|&c| c as u64).sum();
    if total == 0 {
        return 0.0;
    }

    let mean_255 = mean * 255.0;
    let variance: f64 = hist
        .iter()
        .enumerate()
        .map(|(i, &c)| {
            let diff = i as f64 - mean_255 as f64;
            diff * diff * c as f64
        })
        .sum::<f64>()
        / total as f64;

    variance.sqrt() as f32
}

/// Compute histogram percentile value.
///
/// # Arguments
/// * `hist` - 256-bin histogram
/// * `percentile` - Percentile to find (0.0-1.0)
///
/// # Returns
/// Intensity value at the given percentile (0-255)
pub fn histogram_percentile(hist: &[u32; 256], percentile: f32) -> u8 {
    let total: u64 = hist.iter().map(|&c| c as u64).sum();
    if total == 0 {
        return 0;
    }

    let target = (total as f64 * percentile as f64) as u64;
    let mut cumulative: u64 = 0;

    for (i, &count) in hist.iter().enumerate() {
        cumulative += count as u64;
        if cumulative >= target {
            return i as u8;
        }
    }

    255
}

/// Compute histogram confidence based on spread.
///
/// Narrow histograms (low std dev) indicate uniform lighting with high confidence.
/// Wide histograms indicate complex lighting with lower confidence.
///
/// # Arguments
/// * `std_dev` - Standard deviation of histogram (0-128 typical range)
///
/// # Returns
/// Confidence value 0.0-1.0
pub fn histogram_confidence(std_dev: f32) -> f32 {
    // Map std_dev from 0-80 range to 1.0-0.2 confidence
    // Low spread = high confidence, high spread = lower confidence
    let normalized = (std_dev / 80.0).min(1.0);
    1.0 - normalized * 0.8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_histogram_empty() {
        let hist = compute_histogram(&[]);
        assert!(hist.iter().all(|&c| c == 0));
    }

    #[test]
    fn test_compute_histogram_uniform() {
        let data = vec![128u8; 100];
        let hist = compute_histogram(&data);
        assert_eq!(hist[128], 100);
        assert_eq!(hist[0], 0);
        assert_eq!(hist[255], 0);
    }

    #[test]
    fn test_compute_histogram_range() {
        let data: Vec<u8> = (0..=255).collect();
        let hist = compute_histogram(&data);
        for (i, &count) in hist.iter().enumerate() {
            assert_eq!(count, 1, "bin {} should have count 1", i);
        }
    }

    #[test]
    fn test_histogram_mean_uniform() {
        // All pixels at value 128
        let data = vec![128u8; 100];
        let hist = compute_histogram(&data);
        let mean = histogram_mean(&hist);
        assert!((mean - 128.0 / 255.0).abs() < 0.01);
    }

    #[test]
    fn test_histogram_mean_black() {
        let data = vec![0u8; 100];
        let hist = compute_histogram(&data);
        let mean = histogram_mean(&hist);
        assert!((mean - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_histogram_mean_white() {
        let data = vec![255u8; 100];
        let hist = compute_histogram(&data);
        let mean = histogram_mean(&hist);
        assert!((mean - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_histogram_mean_empty() {
        let hist = [0u32; 256];
        let mean = histogram_mean(&hist);
        assert_eq!(mean, 0.0);
    }

    #[test]
    fn test_histogram_std_dev_uniform() {
        // All same value = zero std dev
        let data = vec![128u8; 100];
        let hist = compute_histogram(&data);
        let mean = histogram_mean(&hist);
        let std_dev = histogram_std_dev(&hist, mean);
        assert!(std_dev < 0.01);
    }

    #[test]
    fn test_histogram_std_dev_bimodal() {
        // Half black, half white = high std dev
        let mut data = vec![0u8; 50];
        data.extend(vec![255u8; 50]);
        let hist = compute_histogram(&data);
        let mean = histogram_mean(&hist);
        let std_dev = histogram_std_dev(&hist, mean);
        assert!(std_dev > 100.0); // High spread
    }

    #[test]
    fn test_histogram_percentile_median() {
        let data: Vec<u8> = (0..=255).collect();
        let hist = compute_histogram(&data);
        let median = histogram_percentile(&hist, 0.5);
        assert!(median >= 126 && median <= 128);
    }

    #[test]
    fn test_histogram_percentile_extremes() {
        let data: Vec<u8> = (0..=255).collect();
        let hist = compute_histogram(&data);

        let p10 = histogram_percentile(&hist, 0.1);
        assert!(p10 < 30);

        let p90 = histogram_percentile(&hist, 0.9);
        assert!(p90 > 225);
    }

    #[test]
    fn test_histogram_percentile_empty() {
        let hist = [0u32; 256];
        let p = histogram_percentile(&hist, 0.5);
        assert_eq!(p, 0);
    }

    #[test]
    fn test_histogram_confidence_narrow() {
        // Low std dev = high confidence
        let confidence = histogram_confidence(10.0);
        assert!(confidence > 0.8);
    }

    #[test]
    fn test_histogram_confidence_wide() {
        // High std dev = lower confidence
        let confidence = histogram_confidence(80.0);
        assert!(confidence < 0.3);
    }

    #[test]
    fn test_histogram_confidence_bounds() {
        // Verify confidence stays in reasonable range
        assert!(histogram_confidence(0.0) <= 1.0);
        assert!(histogram_confidence(0.0) >= 0.0);
        // High std dev caps at normalized = 1.0, so confidence = 0.2
        // Use >= 0.19 to account for floating point
        assert!(histogram_confidence(80.0) >= 0.19);
        // Very high std dev still gets clamped via min(1.0)
        assert!(histogram_confidence(200.0) >= 0.19);
    }
}
