//! Color Temperature Estimation
//!
//! Estimates correlated color temperature (CCT) from image data
//! using white patch detection and McCamy's approximation.

/// Color temperature estimation result
#[derive(Debug, Clone, Copy)]
pub struct ColorTemperatureEstimate {
    /// Correlated color temperature in Kelvin (typically 2000-10000K)
    pub temperature: f32,
    /// Ambient color in normalized RGB (0.0-1.0 per channel)
    pub color: [f32; 3],
    /// Confidence in the estimate (0.0-1.0)
    pub confidence: f32,
}

/// Minimum brightness threshold for white pixel detection (0-255)
const WHITE_BRIGHTNESS_THRESHOLD: u8 = 200;

/// Maximum saturation for white pixel detection (0.0-1.0)
const WHITE_SATURATION_THRESHOLD: f32 = 0.15;

/// Detect pixels that are likely white/neutral in an RGBA image.
///
/// White pixels have high brightness and low saturation.
/// Returns RGB tuples of detected white pixels.
pub fn detect_white_pixels(rgba: &[u8], width: u32, height: u32) -> Vec<(u8, u8, u8)> {
    let pixel_count = (width * height) as usize;
    let mut white_pixels = Vec::new();

    for i in 0..pixel_count {
        let offset = i * 4;
        if offset + 2 >= rgba.len() {
            break;
        }

        let r = rgba[offset];
        let g = rgba[offset + 1];
        let b = rgba[offset + 2];

        // Check brightness (average of RGB)
        let brightness = ((r as u16 + g as u16 + b as u16) / 3) as u8;
        if brightness < WHITE_BRIGHTNESS_THRESHOLD {
            continue;
        }

        // Check saturation
        let max_val = r.max(g).max(b);
        let min_val = r.min(g).min(b);
        let saturation = if max_val > 0 {
            (max_val - min_val) as f32 / max_val as f32
        } else {
            0.0
        };

        if saturation < WHITE_SATURATION_THRESHOLD {
            white_pixels.push((r, g, b));
        }
    }

    white_pixels
}

/// Estimate color temperature from detected white pixels.
///
/// Uses McCamy's approximation for correlated color temperature.
pub fn estimate_color_temperature(white_pixels: &[(u8, u8, u8)]) -> ColorTemperatureEstimate {
    if white_pixels.is_empty() {
        return ColorTemperatureEstimate {
            temperature: 6500.0, // Default daylight
            color: [1.0, 1.0, 1.0],
            confidence: 0.0,
        };
    }

    // Average the white pixels to get overall color cast
    let mut sum_r = 0u64;
    let mut sum_g = 0u64;
    let mut sum_b = 0u64;

    for &(r, g, b) in white_pixels {
        sum_r += r as u64;
        sum_g += g as u64;
        sum_b += b as u64;
    }

    let count = white_pixels.len() as f32;
    let avg_r = sum_r as f32 / count / 255.0;
    let avg_g = sum_g as f32 / count / 255.0;
    let avg_b = sum_b as f32 / count / 255.0;

    // Calculate CCT using McCamy's formula
    let temperature = rgb_to_cct(avg_r, avg_g, avg_b);

    // Clamp to reasonable range
    let clamped_temp = temperature.clamp(2000.0, 10000.0);

    // Confidence based on number of white pixels and consistency
    let pixel_confidence = (white_pixels.len() as f32 / 100.0).min(1.0);

    ColorTemperatureEstimate {
        temperature: clamped_temp,
        color: [avg_r, avg_g, avg_b],
        confidence: pixel_confidence,
    }
}

/// Convert RGB to Correlated Color Temperature using McCamy's approximation.
///
/// First converts RGB to CIE XYZ, then to chromaticity coordinates,
/// then applies McCamy's formula.
///
/// # Arguments
/// * `r`, `g`, `b` - Normalized RGB values (0.0-1.0)
///
/// # Returns
/// Color temperature in Kelvin
pub fn rgb_to_cct(r: f32, g: f32, b: f32) -> f32 {
    // Convert sRGB to linear RGB (approximate)
    let r_lin = srgb_to_linear(r);
    let g_lin = srgb_to_linear(g);
    let b_lin = srgb_to_linear(b);

    // Convert to CIE XYZ using sRGB matrix
    let x = 0.4124564 * r_lin + 0.3575761 * g_lin + 0.1804375 * b_lin;
    let y = 0.2126729 * r_lin + 0.7151522 * g_lin + 0.0721750 * b_lin;
    let z = 0.0193339 * r_lin + 0.1191920 * g_lin + 0.9503041 * b_lin;

    // Convert to chromaticity coordinates
    let sum = x + y + z;
    if sum < 0.0001 {
        return 6500.0; // Default for black/very dark
    }

    let xc = x / sum;
    let yc = y / sum;

    // McCamy's formula: n = (x - 0.3320) / (0.1858 - y)
    // CCT = 449n³ + 3525n² + 6823.3n + 5520.33
    let denom = 0.1858 - yc;
    if denom.abs() < 1e-6 {
        return 6500.0; // Default for near-singularity
    }
    let n = (xc - 0.3320) / denom;
    let n2 = n * n;
    let n3 = n2 * n;

    let cct = 449.0 * n3 + 3525.0 * n2 + 6823.3 * n + 5520.33;
    // Clamp to physically meaningful range
    cct.clamp(1000.0, 40000.0)
}

/// Convert sRGB gamma-corrected value to linear.
fn srgb_to_linear(c: f32) -> f32 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

/// Convert color temperature to RGB for visualization/application.
///
/// Uses Tanner Helland's algorithm for approximation.
///
/// # Arguments
/// * `kelvin` - Color temperature in Kelvin (1000-40000)
///
/// # Returns
/// Normalized RGB color (0.0-1.0 per channel)
pub fn cct_to_rgb(kelvin: f32) -> [f32; 3] {
    let temp = (kelvin / 100.0).clamp(10.0, 400.0);

    let r: f32;
    let g: f32;
    let b: f32;

    // Red
    if temp <= 66.0 {
        r = 255.0;
    } else {
        let r_val = temp - 60.0;
        r = (329.698727446 * r_val.powf(-0.1332047592)).clamp(0.0, 255.0);
    }

    // Green
    if temp <= 66.0 {
        g = (99.4708025861 * temp.ln() - 161.1195681661).clamp(0.0, 255.0);
    } else {
        let g_val = temp - 60.0;
        g = (288.1221695283 * g_val.powf(-0.0755148492)).clamp(0.0, 255.0);
    }

    // Blue
    if temp >= 66.0 {
        b = 255.0;
    } else if temp <= 19.0 {
        b = 0.0;
    } else {
        let b_val = temp - 10.0;
        b = (138.5177312231 * b_val.ln() - 305.0447927307).clamp(0.0, 255.0);
    }

    [r / 255.0, g / 255.0, b / 255.0]
}

/// Estimate ambient color from RGBA image.
///
/// Uses histogram-based approach to find dominant color.
pub fn estimate_ambient_color(rgba: &[u8], width: u32, height: u32) -> [f32; 3] {
    let pixel_count = (width * height) as usize;
    if pixel_count == 0 || rgba.len() < 3 {
        return [1.0, 1.0, 1.0];
    }

    let mut sum_r = 0u64;
    let mut sum_g = 0u64;
    let mut sum_b = 0u64;
    let mut count = 0u64;

    for i in 0..pixel_count {
        let offset = i * 4;
        if offset + 2 >= rgba.len() {
            break;
        }

        sum_r += rgba[offset] as u64;
        sum_g += rgba[offset + 1] as u64;
        sum_b += rgba[offset + 2] as u64;
        count += 1;
    }

    if count == 0 {
        return [1.0, 1.0, 1.0];
    }

    let avg_r = sum_r as f32 / count as f32 / 255.0;
    let avg_g = sum_g as f32 / count as f32 / 255.0;
    let avg_b = sum_b as f32 / count as f32 / 255.0;

    // Normalize to preserve relative color but boost brightness
    let max_channel = avg_r.max(avg_g).max(avg_b).max(0.001);
    [avg_r / max_channel, avg_g / max_channel, avg_b / max_channel]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_white_pixels_white_image() {
        // Pure white image
        let rgba = vec![255, 255, 255, 255];
        let white = detect_white_pixels(&rgba, 1, 1);
        assert_eq!(white.len(), 1);
        assert_eq!(white[0], (255, 255, 255));
    }

    #[test]
    fn test_detect_white_pixels_dark_image() {
        // Dark image - no white pixels
        let rgba = vec![50, 50, 50, 255];
        let white = detect_white_pixels(&rgba, 1, 1);
        assert!(white.is_empty());
    }

    #[test]
    fn test_detect_white_pixels_saturated() {
        // Bright but saturated (red) - not white
        let rgba = vec![255, 100, 100, 255];
        let white = detect_white_pixels(&rgba, 1, 1);
        assert!(white.is_empty());
    }

    #[test]
    fn test_detect_white_pixels_mixed() {
        // Mix of white and colored pixels
        let rgba = vec![
            255, 255, 255, 255, // White
            255, 0, 0, 255, // Red (saturated)
            220, 220, 220, 255, // Light gray (white-ish)
            0, 0, 0, 255, // Black
        ];
        let white = detect_white_pixels(&rgba, 2, 2);
        assert_eq!(white.len(), 2); // White and light gray
    }

    #[test]
    fn test_estimate_color_temperature_neutral() {
        let white_pixels = vec![(255, 255, 255), (250, 250, 250)];
        let est = estimate_color_temperature(&white_pixels);
        // Neutral white should be around 6500K (daylight)
        assert!(est.temperature > 5000.0 && est.temperature < 8000.0);
    }

    #[test]
    fn test_estimate_color_temperature_warm() {
        // Warm (reddish) white
        let white_pixels = vec![(255, 230, 200), (250, 225, 195)];
        let est = estimate_color_temperature(&white_pixels);
        // Should be below 5000K
        assert!(est.temperature < 5000.0);
    }

    #[test]
    fn test_estimate_color_temperature_cool() {
        // Cool (bluish) white
        let white_pixels = vec![(200, 220, 255), (195, 215, 250)];
        let est = estimate_color_temperature(&white_pixels);
        // Should be above 6500K
        assert!(est.temperature > 6500.0);
    }

    #[test]
    fn test_estimate_color_temperature_empty() {
        let est = estimate_color_temperature(&[]);
        assert_eq!(est.temperature, 6500.0); // Default
        assert_eq!(est.confidence, 0.0);
    }

    #[test]
    fn test_rgb_to_cct_white() {
        let cct = rgb_to_cct(1.0, 1.0, 1.0);
        // Pure white should be around D65 (6500K)
        assert!(cct > 5000.0 && cct < 8000.0);
    }

    #[test]
    fn test_rgb_to_cct_warm() {
        // More red = lower temperature
        let cct_warm = rgb_to_cct(1.0, 0.8, 0.6);
        let cct_neutral = rgb_to_cct(1.0, 1.0, 1.0);
        assert!(cct_warm < cct_neutral);
    }

    #[test]
    fn test_rgb_to_cct_cool() {
        // More blue = higher temperature
        let cct_cool = rgb_to_cct(0.8, 0.9, 1.0);
        let cct_neutral = rgb_to_cct(1.0, 1.0, 1.0);
        assert!(cct_cool > cct_neutral);
    }

    #[test]
    fn test_cct_to_rgb_warm() {
        let rgb = cct_to_rgb(2700.0); // Warm incandescent
        // Should be orangish
        assert!(rgb[0] > rgb[2]); // More red than blue
    }

    #[test]
    fn test_cct_to_rgb_daylight() {
        let rgb = cct_to_rgb(6500.0); // Daylight
        // Should be near white
        assert!(rgb[0] > 0.9 && rgb[1] > 0.9 && rgb[2] > 0.9);
    }

    #[test]
    fn test_cct_to_rgb_cool() {
        let rgb = cct_to_rgb(10000.0); // Cool
        // Should be bluish
        assert!(rgb[2] >= rgb[0]); // Blue >= Red
    }

    #[test]
    fn test_estimate_ambient_color_uniform() {
        let rgba = vec![200, 150, 100, 255, 200, 150, 100, 255, 200, 150, 100, 255, 200, 150, 100, 255];
        let color = estimate_ambient_color(&rgba, 2, 2);
        // Normalized: max channel should be 1.0
        assert!((color[0] - 1.0).abs() < 0.01); // Red is max
        assert!(color[1] < color[0]); // Green < Red
        assert!(color[2] < color[1]); // Blue < Green
    }

    #[test]
    fn test_estimate_ambient_color_white() {
        let rgba = vec![255, 255, 255, 255];
        let color = estimate_ambient_color(&rgba, 1, 1);
        assert!((color[0] - 1.0).abs() < 0.01);
        assert!((color[1] - 1.0).abs() < 0.01);
        assert!((color[2] - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_estimate_ambient_color_empty() {
        let color = estimate_ambient_color(&[], 0, 0);
        assert_eq!(color, [1.0, 1.0, 1.0]);
    }

    #[test]
    fn test_srgb_to_linear_black() {
        assert_eq!(srgb_to_linear(0.0), 0.0);
    }

    #[test]
    fn test_srgb_to_linear_white() {
        assert!((srgb_to_linear(1.0) - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_srgb_to_linear_mid() {
        // Mid-gray in sRGB is darker in linear
        let linear = srgb_to_linear(0.5);
        assert!(linear < 0.5);
        assert!(linear > 0.2);
    }
}
