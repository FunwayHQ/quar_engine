/**
 * Jest test setup - Mock browser APIs not available in jsdom
 */

// Mock ImageData (available in jsdom but sometimes missing)
if (typeof ImageData === 'undefined') {
  // @ts-expect-error - Mocking global
  global.ImageData = class ImageData {
    data: Uint8ClampedArray;
    width: number;
    height: number;
    colorSpace: PredefinedColorSpace = 'srgb';

    constructor(sw: number, sh: number);
    constructor(data: Uint8ClampedArray, sw: number, sh?: number);
    constructor(dataOrWidth: Uint8ClampedArray | number, widthOrHeight: number, maybeHeight?: number) {
      if (typeof dataOrWidth === 'number') {
        this.width = dataOrWidth;
        this.height = widthOrHeight;
        this.data = new Uint8ClampedArray(this.width * this.height * 4);
      } else {
        this.data = dataOrWidth;
        this.width = widthOrHeight;
        this.height = maybeHeight ?? (dataOrWidth.length / (widthOrHeight * 4));
      }
    }
  };
}

// Mock OffscreenCanvas (not available in jsdom)
class MockOffscreenCanvas {
  width: number;
  height: number;
  private ctx: CanvasRenderingContext2D | null = null;

  constructor(width: number, height: number) {
    this.width = width;
    this.height = height;
  }

  getContext(contextId: string, options?: CanvasRenderingContext2DSettings) {
    if (contextId === '2d') {
      // Create a real canvas to get a real context
      const canvas = document.createElement('canvas');
      canvas.width = this.width;
      canvas.height = this.height;
      this.ctx = canvas.getContext('2d', options);
      return this.ctx;
    }
    return null;
  }

  transferToImageBitmap() {
    throw new Error('Not implemented in mock');
  }
}

// @ts-expect-error - Mocking global
global.OffscreenCanvas = MockOffscreenCanvas;

// Mock MediaStream
class MockMediaStream {
  private tracks: MediaStreamTrack[] = [];

  constructor(tracks: MediaStreamTrack[] = []) {
    this.tracks = tracks;
  }

  getTracks() {
    return this.tracks;
  }

  getVideoTracks() {
    return this.tracks.filter(t => t.kind === 'video');
  }

  getAudioTracks() {
    return this.tracks.filter(t => t.kind === 'audio');
  }

  addTrack(track: MediaStreamTrack) {
    this.tracks.push(track);
  }

  removeTrack(track: MediaStreamTrack) {
    this.tracks = this.tracks.filter(t => t !== track);
  }
}

// Mock MediaStreamTrack
class MockMediaStreamTrack {
  kind: string;
  label: string;
  enabled: boolean = true;
  muted: boolean = false;
  readyState: string = 'live';

  constructor(kind: string = 'video') {
    this.kind = kind;
    this.label = `Mock ${kind} track`;
  }

  stop() {
    this.readyState = 'ended';
  }

  clone() {
    return new MockMediaStreamTrack(this.kind);
  }

  getSettings() {
    return {
      width: 1280,
      height: 720,
      frameRate: 30,
      facingMode: 'environment',
    };
  }

  getCapabilities() {
    return {
      width: { min: 320, max: 1920 },
      height: { min: 240, max: 1080 },
      frameRate: { min: 1, max: 60 },
      facingMode: ['user', 'environment'],
    };
  }

  getConstraints() {
    return {};
  }

  applyConstraints() {
    return Promise.resolve();
  }
}

// @ts-expect-error - Mocking global
global.MediaStream = MockMediaStream;
// @ts-expect-error - Mocking global
global.MediaStreamTrack = MockMediaStreamTrack;

// Mock performance.now if not available
if (typeof performance === 'undefined') {
  // @ts-expect-error - Mocking global
  global.performance = {
    now: () => Date.now(),
  };
}

// Mock URL.createObjectURL and revokeObjectURL
if (typeof URL.createObjectURL === 'undefined') {
  URL.createObjectURL = jest.fn().mockReturnValue('blob:mock-url');
}
if (typeof URL.revokeObjectURL === 'undefined') {
  URL.revokeObjectURL = jest.fn();
}

// Helper to create mock getUserMedia
export function createMockGetUserMedia(shouldSucceed = true, error?: DOMException) {
  return jest.fn().mockImplementation(() => {
    if (shouldSucceed) {
      const track = new MockMediaStreamTrack('video');
      return Promise.resolve(new MockMediaStream([track as unknown as MediaStreamTrack]));
    } else {
      return Promise.reject(error || new DOMException('Permission denied', 'NotAllowedError'));
    }
  });
}

// Helper to setup navigator.mediaDevices mock
export function setupMediaDevicesMock(getUserMediaMock: jest.Mock) {
  Object.defineProperty(navigator, 'mediaDevices', {
    value: {
      getUserMedia: getUserMediaMock,
      enumerateDevices: jest.fn().mockResolvedValue([
        { kind: 'videoinput', deviceId: 'camera1', label: 'Front Camera' },
        { kind: 'videoinput', deviceId: 'camera2', label: 'Back Camera' },
      ]),
    },
    configurable: true,
  });
}

// Helper to create mock video element with proper events
export function createMockVideoElement() {
  const video = document.createElement('video');

  // Override property to simulate video loaded
  Object.defineProperty(video, 'videoWidth', { value: 1280, configurable: true });
  Object.defineProperty(video, 'videoHeight', { value: 720, configurable: true });
  Object.defineProperty(video, 'readyState', { value: 4, configurable: true });

  // Mock play to trigger loadedmetadata
  video.play = jest.fn().mockImplementation(() => {
    // Dispatch loadedmetadata event
    setTimeout(() => {
      video.dispatchEvent(new Event('loadedmetadata'));
    }, 0);
    return Promise.resolve();
  });

  return video;
}

// Override document.createElement to return mock video with proper event handling
const originalCreateElement = document.createElement.bind(document);
document.createElement = function(tagName: string, options?: ElementCreationOptions): HTMLElement {
  const element = originalCreateElement(tagName, options);

  if (tagName.toLowerCase() === 'video') {
    const video = element as HTMLVideoElement;

    // Set up srcObject setter to trigger loadedmetadata
    let _srcObject: MediaProvider | null = null;
    Object.defineProperty(video, 'srcObject', {
      get: () => _srcObject,
      set: (value: MediaProvider | null) => {
        _srcObject = value;
        if (value) {
          // Set video dimensions
          Object.defineProperty(video, 'videoWidth', { value: 1280, configurable: true, writable: true });
          Object.defineProperty(video, 'videoHeight', { value: 720, configurable: true, writable: true });
          Object.defineProperty(video, 'readyState', { value: 4, configurable: true, writable: true });

          // Trigger loadedmetadata after a microtask
          queueMicrotask(() => {
            video.dispatchEvent(new Event('loadedmetadata'));
          });
        }
      },
      configurable: true,
    });

    // Mock play
    video.play = jest.fn().mockResolvedValue(undefined);
  }

  return element;
};
