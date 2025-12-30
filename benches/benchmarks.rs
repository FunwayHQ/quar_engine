//! Performance benchmarks for QUAR Engine.
//!
//! Run with: cargo bench
//!
//! Target performance: <5ms for 640x480 frame (full pipeline)

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use quar_engine::features::{non_maximum_suppression, rgba_to_grayscale, FastDetector, KeyPoint};

/// Generate a test image with patterns that create edges and corners.
fn generate_test_image(width: u32, height: u32) -> Vec<u8> {
    let mut rgba = vec![0u8; (width * height * 4) as usize];

    // Fill with a pattern that creates edges and corners
    for y in 0..height {
        for x in 0..width {
            let idx = ((y * width + x) * 4) as usize;

            // Create a checkerboard-like pattern with gradients
            let value = if (x / 32 + y / 32) % 2 == 0 {
                ((x % 32) as f32 / 31.0 * 200.0) as u8
            } else {
                200 - ((y % 32) as f32 / 31.0 * 200.0) as u8
            };

            // Add some "noise" by using position to vary values
            let noise = ((x * 7 + y * 13) % 50) as u8;

            rgba[idx] = value.saturating_add(noise);
            rgba[idx + 1] = value;
            rgba[idx + 2] = value.saturating_sub(noise / 2);
            rgba[idx + 3] = 255;
        }
    }

    rgba
}

/// Benchmark grayscale conversion at different resolutions.
fn bench_grayscale(c: &mut Criterion) {
    let mut group = c.benchmark_group("grayscale_conversion");

    for (width, height) in [(320, 240), (640, 480), (1280, 720), (1920, 1080)] {
        let rgba = generate_test_image(width, height);

        group.bench_with_input(
            BenchmarkId::new("rgba_to_grayscale", format!("{}x{}", width, height)),
            &rgba,
            |b, data| {
                b.iter(|| black_box(rgba_to_grayscale(data)));
            },
        );
    }

    group.finish();
}

/// Benchmark FAST corner detection at different resolutions.
fn bench_fast_detection(c: &mut Criterion) {
    let mut group = c.benchmark_group("fast_detection");

    for (width, height) in [(320, 240), (640, 480), (1280, 720)] {
        let rgba = generate_test_image(width, height);
        let grayscale = rgba_to_grayscale(&rgba);
        let detector = FastDetector::new(20);

        group.bench_with_input(
            BenchmarkId::new("detect", format!("{}x{}", width, height)),
            &grayscale,
            |b, data| {
                b.iter(|| black_box(detector.detect(data, width, height)));
            },
        );
    }

    group.finish();
}

/// Benchmark FAST with different thresholds.
fn bench_fast_thresholds(c: &mut Criterion) {
    let mut group = c.benchmark_group("fast_thresholds");

    let rgba = generate_test_image(640, 480);
    let grayscale = rgba_to_grayscale(&rgba);

    for threshold in [10, 20, 30, 50] {
        let detector = FastDetector::new(threshold);

        group.bench_with_input(
            BenchmarkId::new("threshold", threshold),
            &grayscale,
            |b, data| {
                b.iter(|| black_box(detector.detect(data, 640, 480)));
            },
        );
    }

    group.finish();
}

/// Benchmark non-maximum suppression.
fn bench_nms(c: &mut Criterion) {
    let mut group = c.benchmark_group("nms");

    // Generate keypoints for NMS benchmarks
    let keypoints: Vec<KeyPoint> = (0..1000)
        .map(|i| {
            KeyPoint::new(
                (i * 7 % 640) as u32,
                (i * 13 % 480) as u32,
                (i as f32 / 1000.0),
            )
        })
        .collect();

    for radius in [3, 5, 8, 16] {
        group.bench_with_input(BenchmarkId::new("radius", radius), &keypoints, |b, kps| {
            b.iter(|| black_box(non_maximum_suppression(kps, radius)));
        });
    }

    group.finish();
}

/// Benchmark the full pipeline (grayscale + FAST + NMS).
/// This is the most important benchmark - target is <5ms for 640x480.
fn bench_full_pipeline(c: &mut Criterion) {
    let mut group = c.benchmark_group("full_pipeline");

    for (width, height) in [(640, 480), (1280, 720)] {
        let rgba = generate_test_image(width, height);

        group.bench_with_input(
            BenchmarkId::new("full", format!("{}x{}", width, height)),
            &rgba,
            |b, data| {
                b.iter(|| {
                    let grayscale = rgba_to_grayscale(data);
                    let detector = FastDetector::new(20);
                    let keypoints = detector.detect(&grayscale, width, height);
                    black_box(non_maximum_suppression(&keypoints, 3))
                });
            },
        );
    }

    group.finish();
}

criterion_group!(
    benches,
    bench_grayscale,
    bench_fast_detection,
    bench_fast_thresholds,
    bench_nms,
    bench_full_pipeline,
);

criterion_main!(benches);
