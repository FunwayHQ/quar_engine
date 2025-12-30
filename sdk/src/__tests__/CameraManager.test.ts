/**
 * Unit tests for CameraManager
 */

import { CameraManager, ResolutionPresets } from '../camera/CameraManager';
import { QuarError, QuarErrorCode } from '../types';
import { createMockGetUserMedia, setupMediaDevicesMock } from './setup';

describe('CameraManager', () => {
  let camera: CameraManager;
  let mockGetUserMedia: jest.Mock;

  beforeEach(() => {
    camera = new CameraManager();
    mockGetUserMedia = createMockGetUserMedia(true);
    setupMediaDevicesMock(mockGetUserMedia);
  });

  afterEach(() => {
    camera.destroy();
    jest.clearAllMocks();
  });

  describe('initialization', () => {
    it('should create a new instance', () => {
      expect(camera).toBeInstanceOf(CameraManager);
      expect(camera.isReady()).toBe(false);
    });

    it('should throw error if camera API not available', async () => {
      // Remove mediaDevices
      Object.defineProperty(navigator, 'mediaDevices', {
        value: undefined,
        configurable: true,
      });

      await expect(camera.init()).rejects.toThrow(QuarError);
      await expect(camera.init()).rejects.toMatchObject({
        code: QuarErrorCode.CAMERA_NOT_AVAILABLE,
      });
    });

    it('should request camera access with default config', async () => {
      await camera.init();

      expect(mockGetUserMedia).toHaveBeenCalledWith({
        video: {
          facingMode: { ideal: 'environment' },
          width: { ideal: 1280 },
          height: { ideal: 720 },
          frameRate: { ideal: 30 },
        },
        audio: false,
      });
    });

    it('should accept custom configuration', async () => {
      await camera.init({
        facingMode: 'user',
        resolution: ResolutionPresets.vga,
        frameRate: 60,
      });

      expect(mockGetUserMedia).toHaveBeenCalledWith({
        video: {
          facingMode: { ideal: 'user' },
          width: { ideal: 640 },
          height: { ideal: 480 },
          frameRate: { ideal: 60 },
        },
        audio: false,
      });
    });
  });

  describe('error handling', () => {
    it('should throw CAMERA_PERMISSION_DENIED on NotAllowedError', async () => {
      mockGetUserMedia = createMockGetUserMedia(
        false,
        new DOMException('Permission denied', 'NotAllowedError')
      );
      setupMediaDevicesMock(mockGetUserMedia);

      await expect(camera.init()).rejects.toMatchObject({
        code: QuarErrorCode.CAMERA_PERMISSION_DENIED,
      });
    });

    it('should throw CAMERA_NOT_AVAILABLE on NotFoundError', async () => {
      mockGetUserMedia = createMockGetUserMedia(
        false,
        new DOMException('No camera', 'NotFoundError')
      );
      setupMediaDevicesMock(mockGetUserMedia);

      await expect(camera.init()).rejects.toMatchObject({
        code: QuarErrorCode.CAMERA_NOT_AVAILABLE,
      });
    });

    it('should throw CAMERA_NOT_AVAILABLE on OverconstrainedError', async () => {
      mockGetUserMedia = createMockGetUserMedia(
        false,
        new DOMException('Resolution not supported', 'OverconstrainedError')
      );
      setupMediaDevicesMock(mockGetUserMedia);

      await expect(camera.init()).rejects.toMatchObject({
        code: QuarErrorCode.CAMERA_NOT_AVAILABLE,
      });
    });
  });

  describe('getFrame', () => {
    it('should throw error if not initialized', () => {
      expect(() => camera.getFrame()).toThrow(QuarError);
      expect(() => camera.getFrame()).toThrow('Camera not initialized');
    });
  });

  describe('getResolution', () => {
    it('should return resolution after initialization', async () => {
      await camera.init();
      const resolution = camera.getResolution();

      expect(resolution).toHaveProperty('width');
      expect(resolution).toHaveProperty('height');
      expect(resolution.width).toBeGreaterThan(0);
      expect(resolution.height).toBeGreaterThan(0);
    });

    it('should return zero resolution before initialization', () => {
      const resolution = camera.getResolution();
      expect(resolution).toEqual({ width: 0, height: 0 });
    });
  });

  describe('getFacingMode', () => {
    it('should return configured facing mode', async () => {
      await camera.init({ facingMode: 'user' });
      expect(camera.getFacingMode()).toBe('user');
    });
  });

  describe('switchCamera', () => {
    it('should throw error if not initialized', async () => {
      await expect(camera.switchCamera()).rejects.toThrow(QuarError);
    });

    it('should toggle facing mode', async () => {
      await camera.init({ facingMode: 'environment' });
      expect(camera.getFacingMode()).toBe('environment');

      await camera.switchCamera();
      expect(camera.getFacingMode()).toBe('user');

      await camera.switchCamera();
      expect(camera.getFacingMode()).toBe('environment');
    });
  });

  describe('pause and resume', () => {
    it('should not throw when pausing uninitialized camera', () => {
      expect(() => camera.pause()).not.toThrow();
    });

    it('should not throw when resuming uninitialized camera', () => {
      expect(() => camera.resume()).not.toThrow();
    });
  });

  describe('destroy', () => {
    it('should clean up resources', async () => {
      await camera.init();
      expect(camera.isReady()).toBe(true);

      camera.destroy();
      expect(camera.isReady()).toBe(false);
    });

    it('should be safe to call multiple times', () => {
      camera.destroy();
      camera.destroy();
      expect(camera.isReady()).toBe(false);
    });
  });
});

describe('ResolutionPresets', () => {
  it('should have HD preset', () => {
    expect(ResolutionPresets.hd).toEqual({ width: 1280, height: 720 });
  });

  it('should have FHD preset', () => {
    expect(ResolutionPresets.fhd).toEqual({ width: 1920, height: 1080 });
  });

  it('should have VGA preset', () => {
    expect(ResolutionPresets.vga).toEqual({ width: 640, height: 480 });
  });
});
