/**
 * Unit tests for FrameCapture
 */

import { FrameCapture, calculateFrameStats, GrayscaleFrame } from '../camera/FrameCapture';

describe('FrameCapture', () => {
  let frameCapture: FrameCapture;

  beforeEach(() => {
    frameCapture = new FrameCapture();
  });

  afterEach(() => {
    frameCapture.destroy();
  });

  describe('createProcessingFrame', () => {
    it('should create a processing frame from ImageData', () => {
      const imageData = new ImageData(100, 100);
      const frame = frameCapture.createProcessingFrame(imageData);

      expect(frame.width).toBe(100);
      expect(frame.height).toBe(100);
      expect(frame.data).toBe(imageData.data);
      expect(frame.timestamp).toBeGreaterThan(0);
    });
  });

  describe('toGrayscale', () => {
    it('should convert RGBA to grayscale', () => {
      const width = 2;
      const height = 2;
      const imageData = new ImageData(width, height);

      // Set pixels: red, green, blue, white
      // Red: 255, 0, 0 -> ~77 (0.299 * 255)
      imageData.data.set([255, 0, 0, 255], 0);
      // Green: 0, 255, 0 -> ~150 (0.587 * 255)
      imageData.data.set([0, 255, 0, 255], 4);
      // Blue: 0, 0, 255 -> ~29 (0.114 * 255)
      imageData.data.set([0, 0, 255, 255], 8);
      // White: 255, 255, 255 -> ~255
      imageData.data.set([255, 255, 255, 255], 12);

      const gray = frameCapture.toGrayscale(imageData);

      expect(gray.width).toBe(2);
      expect(gray.height).toBe(2);
      expect(gray.data.length).toBe(4);

      // Check grayscale values (using integer math approximation)
      expect(gray.data[0]).toBeCloseTo(76, -1); // Red
      expect(gray.data[1]).toBeCloseTo(150, -1); // Green
      expect(gray.data[2]).toBeCloseTo(29, -1); // Blue
      expect(gray.data[3]).toBeCloseTo(255, -1); // White
    });

    it('should reuse buffer for same size', () => {
      const imageData1 = new ImageData(100, 100);
      const imageData2 = new ImageData(100, 100);

      const gray1 = frameCapture.toGrayscale(imageData1);
      const gray2 = frameCapture.toGrayscale(imageData2);

      // Should be the same buffer reference
      expect(gray1.data).toBe(gray2.data);
    });

    it('should allocate new buffer for different size', () => {
      const imageData1 = new ImageData(100, 100);
      const imageData2 = new ImageData(200, 200);

      const gray1 = frameCapture.toGrayscale(imageData1);
      const gray1Buffer = gray1.data;

      const gray2 = frameCapture.toGrayscale(imageData2);

      // Buffer should be different
      expect(gray2.data).not.toBe(gray1Buffer);
      expect(gray2.data.length).toBe(200 * 200);
    });
  });

  describe('downsample2x', () => {
    it('should halve dimensions', () => {
      const frame: GrayscaleFrame = {
        data: new Uint8Array(100 * 100),
        width: 100,
        height: 100,
        timestamp: 0,
      };

      const downsampled = frameCapture.downsample2x(frame);

      expect(downsampled.width).toBe(50);
      expect(downsampled.height).toBe(50);
      expect(downsampled.data.length).toBe(50 * 50);
    });

    it('should average 2x2 blocks', () => {
      // Create 4x4 frame with known values
      const data = new Uint8Array([
        100, 100, 200, 200,
        100, 100, 200, 200,
        50, 50, 150, 150,
        50, 50, 150, 150,
      ]);
      const frame: GrayscaleFrame = {
        data,
        width: 4,
        height: 4,
        timestamp: 0,
      };

      const downsampled = frameCapture.downsample2x(frame);

      expect(downsampled.data[0]).toBe(100); // Average of top-left 2x2
      expect(downsampled.data[1]).toBe(200); // Average of top-right 2x2
      expect(downsampled.data[2]).toBe(50);  // Average of bottom-left 2x2
      expect(downsampled.data[3]).toBe(150); // Average of bottom-right 2x2
    });
  });

  describe('buildPyramid', () => {
    it('should build correct number of levels', () => {
      const frame: GrayscaleFrame = {
        data: new Uint8Array(256 * 256),
        width: 256,
        height: 256,
        timestamp: 0,
      };

      const pyramid = frameCapture.buildPyramid(frame, 4);

      expect(pyramid.length).toBe(4);
      expect(pyramid[0].width).toBe(256);
      expect(pyramid[1].width).toBe(128);
      expect(pyramid[2].width).toBe(64);
      expect(pyramid[3].width).toBe(32);
    });

    it('should include original frame as first level', () => {
      const frame: GrayscaleFrame = {
        data: new Uint8Array(64 * 64),
        width: 64,
        height: 64,
        timestamp: 0,
      };

      const pyramid = frameCapture.buildPyramid(frame, 2);

      expect(pyramid[0]).toBe(frame);
    });
  });

  describe('getFrameDelta', () => {
    it('should return 0 on first call', () => {
      expect(frameCapture.getFrameDelta()).toBe(0);
    });

    it('should return positive delta on subsequent calls', async () => {
      frameCapture.getFrameDelta(); // First call

      // Wait a bit
      await new Promise(resolve => setTimeout(resolve, 10));

      const delta = frameCapture.getFrameDelta();
      expect(delta).toBeGreaterThan(0);
    });
  });

  describe('resetTiming', () => {
    it('should reset frame timing', () => {
      frameCapture.getFrameDelta(); // Start timing

      frameCapture.resetTiming();

      expect(frameCapture.getFrameDelta()).toBe(0);
    });
  });

  describe('destroy', () => {
    it('should release buffers', () => {
      // Create some buffers
      const imageData = new ImageData(100, 100);
      frameCapture.toGrayscale(imageData);

      frameCapture.destroy();

      // After destroy, new conversion should work fine
      const gray = frameCapture.toGrayscale(imageData);
      expect(gray.data.length).toBe(10000);
    });
  });
});

describe('calculateFrameStats', () => {
  it('should calculate min, max, mean, variance', () => {
    const frame: GrayscaleFrame = {
      data: new Uint8Array([0, 50, 100, 150, 200, 250]),
      width: 6,
      height: 1,
      timestamp: 0,
    };

    const stats = calculateFrameStats(frame);

    expect(stats.min).toBe(0);
    expect(stats.max).toBe(250);
    expect(stats.mean).toBeCloseTo(125, 0);
    expect(stats.variance).toBeGreaterThan(0);
  });

  it('should handle uniform frame', () => {
    const frame: GrayscaleFrame = {
      data: new Uint8Array([128, 128, 128, 128]),
      width: 2,
      height: 2,
      timestamp: 0,
    };

    const stats = calculateFrameStats(frame);

    expect(stats.min).toBe(128);
    expect(stats.max).toBe(128);
    expect(stats.mean).toBe(128);
    expect(stats.variance).toBe(0);
  });
});
