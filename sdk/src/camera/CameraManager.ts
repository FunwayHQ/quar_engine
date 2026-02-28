/**
 * CameraManager - Handles camera access and frame capture for QUAR Engine
 *
 * Supports:
 * - getUserMedia camera access
 * - iOS Safari quirks (playsinline, muted autoplay)
 * - Efficient frame extraction via OffscreenCanvas
 * - Camera switching (front/back)
 */

import { QuarError, QuarErrorCode } from '../types';

/**
 * Camera configuration options.
 */
export interface CameraManagerConfig {
  /** Which camera to use: 'environment' (back) or 'user' (front) */
  facingMode: 'environment' | 'user';
  /** Target resolution */
  resolution: {
    width: number;
    height: number;
  };
  /** Target frame rate */
  frameRate: number;
}

/**
 * Resolution presets for common use cases.
 */
export const ResolutionPresets = {
  /** 1280x720 - Good balance of quality and performance */
  hd: { width: 1280, height: 720 },
  /** 1920x1080 - High quality, more processing required */
  fhd: { width: 1920, height: 1080 },
  /** 640x480 - Lower quality, best performance */
  vga: { width: 640, height: 480 },
} as const;

/**
 * Default camera configuration.
 */
const DEFAULT_CONFIG: CameraManagerConfig = {
  facingMode: 'environment',
  resolution: ResolutionPresets.hd,
  frameRate: 30,
};

/**
 * CameraManager handles camera access and frame extraction.
 *
 * @example
 * ```typescript
 * const camera = new CameraManager();
 * await camera.init({ facingMode: 'environment' });
 *
 * // Get frames for processing
 * const frame = camera.getFrame();
 * console.log(frame.width, frame.height);
 *
 * // Clean up
 * camera.destroy();
 * ```
 */
export class CameraManager {
  private config: CameraManagerConfig;
  private videoElement: HTMLVideoElement | null = null;
  private stream: MediaStream | null = null;
  private canvas: HTMLCanvasElement | OffscreenCanvas | null = null;
  private ctx: CanvasRenderingContext2D | OffscreenCanvasRenderingContext2D | null = null;
  private isInitialized = false;
  private actualResolution: { width: number; height: number } = { width: 0, height: 0 };

  constructor() {
    this.config = { ...DEFAULT_CONFIG };
  }

  /**
   * Initialize the camera with the given configuration.
   * @param config - Partial camera configuration (merged with defaults)
   * @throws QuarError if camera access fails
   */
  async init(config: Partial<CameraManagerConfig> = {}): Promise<void> {
    // Merge with defaults
    this.config = {
      ...DEFAULT_CONFIG,
      ...config,
      resolution: {
        ...DEFAULT_CONFIG.resolution,
        ...config.resolution,
      },
    };

    // Check for camera API support
    if (!navigator.mediaDevices?.getUserMedia) {
      throw new QuarError(
        QuarErrorCode.CAMERA_NOT_AVAILABLE,
        'Camera API not available. Please use HTTPS and a modern browser.',
        false,
        'Ensure you are using HTTPS and a browser that supports getUserMedia'
      );
    }

    // Create video element for capturing stream
    this.videoElement = this.createVideoElement();

    // Request camera access
    try {
      this.stream = await this.requestCameraAccess();
    } catch (error) {
      // Clean up video element on failure
      if (this.videoElement) {
        this.videoElement.remove();
        this.videoElement = null;
      }
      this.handleCameraError(error);
    }

    // Connect stream to video element
    if (this.videoElement && this.stream) {
      this.videoElement.srcObject = this.stream;
      await this.waitForVideoReady();
    }

    // Create canvas for frame extraction
    this.createCanvas();

    this.isInitialized = true;
  }

  /**
   * Get the current video frame as ImageData.
   * @returns ImageData containing RGBA pixel data
   * @throws QuarError if camera not initialized
   */
  getFrame(): ImageData {
    if (!this.isInitialized || !this.videoElement || !this.ctx) {
      throw new QuarError(
        QuarErrorCode.CAMERA_NOT_AVAILABLE,
        'Camera not initialized. Call init() first.',
        true,
        'Ensure camera.init() completes before calling getFrame()'
      );
    }

    const { width, height } = this.actualResolution;

    // Draw current video frame to canvas
    this.ctx.drawImage(this.videoElement, 0, 0, width, height);

    // Extract pixel data
    return this.ctx.getImageData(0, 0, width, height);
  }

  /**
   * Get the current video frame as a Uint8ClampedArray (RGBA).
   * More efficient than getFrame() when you only need the raw bytes.
   * @returns Uint8ClampedArray of RGBA pixel data
   */
  getFrameData(): Uint8ClampedArray {
    return this.getFrame().data;
  }

  /**
   * Get the actual camera resolution (may differ from requested).
   */
  getResolution(): { width: number; height: number } {
    return { ...this.actualResolution };
  }

  /**
   * Get the configured frame rate.
   */
  getFrameRate(): number {
    return this.config.frameRate;
  }

  /**
   * Check if the camera is initialized and ready.
   */
  isReady(): boolean {
    return this.isInitialized;
  }

  /**
   * Switch between front and back cameras.
   * @throws QuarError if switch fails
   */
  async switchCamera(): Promise<void> {
    if (!this.isInitialized) {
      throw new QuarError(
        QuarErrorCode.CAMERA_NOT_AVAILABLE,
        'Camera not initialized. Call init() first.',
        true
      );
    }

    // Toggle facing mode
    const newFacingMode = this.config.facingMode === 'environment' ? 'user' : 'environment';

    // Stop current stream
    this.stopStream();

    // Reinitialize with new facing mode
    this.config.facingMode = newFacingMode;

    try {
      this.stream = await this.requestCameraAccess();
      if (this.videoElement && this.stream) {
        this.videoElement.srcObject = this.stream;
        await this.waitForVideoReady();
        this.createCanvas(); // Recreate canvas for potentially different resolution
      }
    } catch (error) {
      // Revert on failure
      this.config.facingMode = newFacingMode === 'environment' ? 'user' : 'environment';
      this.handleCameraError(error);
    }
  }

  /**
   * Get the current facing mode.
   */
  getFacingMode(): 'environment' | 'user' {
    return this.config.facingMode;
  }

  /**
   * Get the video element for direct rendering.
   * Use this for efficient canvas drawing instead of getFrame().
   */
  getVideoElement(): HTMLVideoElement | null {
    return this.videoElement;
  }

  /**
   * Pause the video stream (reduces CPU usage when not processing).
   */
  pause(): void {
    if (this.videoElement) {
      this.videoElement.pause();
    }
  }

  /**
   * Resume the video stream.
   */
  resume(): void {
    if (this.videoElement) {
      this.videoElement.play().catch(() => {
        // Ignore autoplay errors
      });
    }
  }

  /**
   * Clean up all resources.
   */
  destroy(): void {
    this.stopStream();

    if (this.videoElement) {
      this.videoElement.srcObject = null;
      this.videoElement.remove();
      this.videoElement = null;
    }

    this.canvas = null;
    this.ctx = null;
    this.isInitialized = false;
  }

  // ============ Private Methods ============

  /**
   * Create a video element configured for iOS Safari compatibility.
   */
  private createVideoElement(): HTMLVideoElement {
    const video = document.createElement('video');

    // iOS Safari requirements
    video.setAttribute('playsinline', ''); // Required for iOS
    video.setAttribute('webkit-playsinline', ''); // Legacy iOS
    video.muted = true; // Required for autoplay
    video.autoplay = true;

    // Hide the video element (we only need it for capture)
    video.style.position = 'absolute';
    video.style.width = '1px';
    video.style.height = '1px';
    video.style.opacity = '0';
    video.style.pointerEvents = 'none';

    // Add to DOM (required for some browsers)
    document.body.appendChild(video);

    return video;
  }

  /**
   * Request camera access with the configured constraints.
   */
  private async requestCameraAccess(): Promise<MediaStream> {
    const constraints: MediaStreamConstraints = {
      video: {
        facingMode: { ideal: this.config.facingMode },
        width: { ideal: this.config.resolution.width },
        height: { ideal: this.config.resolution.height },
        frameRate: { ideal: this.config.frameRate },
      },
      audio: false,
    };

    return navigator.mediaDevices.getUserMedia(constraints);
  }

  /**
   * Wait for video element to be ready and playing.
   */
  private async waitForVideoReady(): Promise<void> {
    if (!this.videoElement) return;

    return new Promise((resolve, reject) => {
      const video = this.videoElement!;
      const timeout = setTimeout(() => {
        reject(new QuarError(
          QuarErrorCode.CAMERA_NOT_AVAILABLE,
          'Camera initialization timed out',
          true,
          'Try refreshing the page or checking camera permissions'
        ));
      }, 10000);

      const onLoadedMetadata = () => {
        // Update actual resolution from video
        this.actualResolution = {
          width: video.videoWidth,
          height: video.videoHeight,
        };

        clearTimeout(timeout);
        video.removeEventListener('loadedmetadata', onLoadedMetadata);
        video.removeEventListener('error', onError);

        // Start playing
        video.play()
          .then(() => resolve())
          .catch((err) => {
            // Autoplay might be blocked, but we can still capture frames
            console.warn('[QUAR] Autoplay blocked, attempting manual play:', err);
            resolve();
          });
      };

      const onError = (event: Event) => {
        clearTimeout(timeout);
        video.removeEventListener('loadedmetadata', onLoadedMetadata);
        video.removeEventListener('error', onError);
        reject(new QuarError(
          QuarErrorCode.CAMERA_NOT_AVAILABLE,
          `Video element error: ${(event as ErrorEvent).message || 'Unknown error'}`,
          false
        ));
      };

      video.addEventListener('loadedmetadata', onLoadedMetadata);
      video.addEventListener('error', onError);

      // If already loaded (cached), trigger immediately
      if (video.readyState >= 1) {
        onLoadedMetadata();
      }
    });
  }

  /**
   * Create canvas for frame extraction.
   * Uses OffscreenCanvas when available for better performance.
   */
  private createCanvas(): void {
    const { width, height } = this.actualResolution;

    // Try OffscreenCanvas first (better performance, works in workers)
    if (typeof OffscreenCanvas !== 'undefined') {
      try {
        this.canvas = new OffscreenCanvas(width, height);
        this.ctx = this.canvas.getContext('2d', {
          willReadFrequently: true, // Optimize for getImageData
        }) as OffscreenCanvasRenderingContext2D;

        if (this.ctx) return;
      } catch {
        // Fall back to regular canvas
      }
    }

    // Fallback: Regular canvas (Safari, older browsers)
    const canvas = document.createElement('canvas');
    canvas.width = width;
    canvas.height = height;
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d', {
      willReadFrequently: true,
    });
  }

  /**
   * Stop the media stream and release camera.
   */
  private stopStream(): void {
    if (this.stream) {
      this.stream.getTracks().forEach((track) => track.stop());
      this.stream = null;
    }
  }

  /**
   * Handle camera access errors and throw appropriate QuarError.
   */
  private handleCameraError(error: unknown): never {
    if (error instanceof QuarError) {
      throw error;
    }

    const err = error as DOMException;

    switch (err.name) {
      case 'NotAllowedError':
      case 'PermissionDeniedError':
        throw new QuarError(
          QuarErrorCode.CAMERA_PERMISSION_DENIED,
          'Camera permission denied. Please allow camera access.',
          false,
          'Go to browser settings and enable camera access for this site'
        );

      case 'NotFoundError':
      case 'DevicesNotFoundError':
        throw new QuarError(
          QuarErrorCode.CAMERA_NOT_AVAILABLE,
          'No camera found on this device.',
          false,
          'Connect a camera or use a device with a built-in camera'
        );

      case 'NotReadableError':
      case 'TrackStartError':
        throw new QuarError(
          QuarErrorCode.CAMERA_NOT_AVAILABLE,
          'Camera is in use by another application.',
          true,
          'Close other apps using the camera and try again'
        );

      case 'OverconstrainedError':
        throw new QuarError(
          QuarErrorCode.CAMERA_NOT_AVAILABLE,
          'Camera does not support the requested resolution.',
          true,
          'Try a lower resolution setting'
        );

      case 'SecurityError':
        throw new QuarError(
          QuarErrorCode.CAMERA_NOT_AVAILABLE,
          'Camera access blocked due to security policy.',
          false,
          'Ensure you are using HTTPS'
        );

      default:
        throw new QuarError(
          QuarErrorCode.CAMERA_NOT_AVAILABLE,
          `Camera error: ${err.message || 'Unknown error'}`,
          false
        );
    }
  }
}
