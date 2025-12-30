/**
 * Tests for SharedFrameBuffer double-buffering implementation.
 */

import {
  SharedFrameBuffer,
  TransferableFrameBuffer,
} from '../../worker/SharedFrameBuffer';
import {
  BUFFER_CONTROL_EMPTY,
  BUFFER_CONTROL_FILLED,
  BUFFER_CONTROL_PROCESSING,
} from '../../worker/types';

describe('SharedFrameBuffer', () => {
  const WIDTH = 640;
  const HEIGHT = 480;

  describe('constructor', () => {
    it('should create instance with specified dimensions', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      expect(buffer.getDimensions()).toEqual({ width: WIDTH, height: HEIGHT });
    });

    it('should not be initialized after construction', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      expect(buffer.isInitialized()).toBe(false);
    });
  });

  describe('init', () => {
    it('should initialize successfully with SharedArrayBuffer available', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      expect(() => buffer.init()).not.toThrow();
      expect(buffer.isInitialized()).toBe(true);
    });

    it('should create two buffers for double-buffering', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();
      const buffers = buffer.getBuffers();
      expect(buffers).toHaveLength(2);
    });

    it('should create buffers with correct size', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();
      const buffers = buffer.getBuffers();

      // Size = control word (4 bytes) + RGBA data
      const expectedSize = 4 + WIDTH * HEIGHT * 4;
      expect(buffers[0].byteLength).toBe(expectedSize);
      expect(buffers[1].byteLength).toBe(expectedSize);
    });

    it('should initialize buffers as empty', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();
      const buffers = buffer.getBuffers();

      for (const sharedBuffer of buffers) {
        const controlView = new Int32Array(sharedBuffer, 0, 1);
        expect(Atomics.load(controlView, 0)).toBe(BUFFER_CONTROL_EMPTY);
      }
    });

    it('should be idempotent (calling init twice does nothing)', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();
      const buffers1 = buffer.getBuffers();
      buffer.init();
      const buffers2 = buffer.getBuffers();
      expect(buffers1).toBe(buffers2);
    });
  });

  describe('getBuffers', () => {
    it('should throw if not initialized', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      expect(() => buffer.getBuffers()).toThrow('SharedFrameBuffer not initialized');
    });

    it('should return SharedArrayBuffer instances', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();
      const buffers = buffer.getBuffers();
      expect(buffers[0]).toBeInstanceOf(SharedArrayBuffer);
      expect(buffers[1]).toBeInstanceOf(SharedArrayBuffer);
    });
  });

  describe('writeFrame', () => {
    it('should throw if not initialized', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      const data = new Uint8ClampedArray(WIDTH * HEIGHT * 4);
      expect(() => buffer.writeFrame(data)).toThrow('SharedFrameBuffer not initialized');
    });

    it('should write frame to first buffer initially', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();

      const data = new Uint8ClampedArray(WIDTH * HEIGHT * 4);
      data.fill(128);

      const bufferIndex = buffer.writeFrame(data);
      expect(bufferIndex).toBe(0);
    });

    it('should alternate between buffers', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();

      const data = new Uint8ClampedArray(WIDTH * HEIGHT * 4);

      const index1 = buffer.writeFrame(data);
      expect(index1).toBe(0);

      // Mark first buffer as empty to allow next write
      SharedFrameBuffer.markEmpty(buffer.getBuffers()[0]);

      const index2 = buffer.writeFrame(data);
      expect(index2).toBe(1);

      SharedFrameBuffer.markEmpty(buffer.getBuffers()[1]);

      const index3 = buffer.writeFrame(data);
      expect(index3).toBe(0);
    });

    it('should mark buffer as filled after write', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();

      const data = new Uint8ClampedArray(WIDTH * HEIGHT * 4);
      const bufferIndex = buffer.writeFrame(data);

      const sharedBuffer = buffer.getBuffers()[bufferIndex];
      const controlView = new Int32Array(sharedBuffer, 0, 1);
      expect(Atomics.load(controlView, 0)).toBe(BUFFER_CONTROL_FILLED);
    });

    it('should return -1 if buffer is being processed', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();

      const data = new Uint8ClampedArray(WIDTH * HEIGHT * 4);

      // Write to first buffer
      buffer.writeFrame(data);

      // Mark first buffer as processing
      SharedFrameBuffer.markProcessing(buffer.getBuffers()[0]);

      // Write to second buffer
      const index2 = buffer.writeFrame(data);
      expect(index2).toBe(1);

      // Mark second buffer as processing
      SharedFrameBuffer.markProcessing(buffer.getBuffers()[1]);

      // Both buffers now processing, should return -1
      const index3 = buffer.writeFrame(data);
      expect(index3).toBe(-1);
    });

    it('should throw for invalid frame size', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();

      const wrongSizeData = new Uint8ClampedArray(100);
      expect(() => buffer.writeFrame(wrongSizeData)).toThrow('Invalid frame size');
    });

    it('should correctly copy frame data', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();

      const data = new Uint8ClampedArray(WIDTH * HEIGHT * 4);
      // Fill with pattern
      for (let i = 0; i < data.length; i++) {
        data[i] = i % 256;
      }

      const bufferIndex = buffer.writeFrame(data);
      const sharedBuffer = buffer.getBuffers()[bufferIndex];
      const frameData = SharedFrameBuffer.getFrameData(sharedBuffer, WIDTH, HEIGHT);

      // Verify data was copied correctly
      for (let i = 0; i < data.length; i++) {
        expect(frameData[i]).toBe(data[i]);
      }
    });
  });

  describe('static methods', () => {
    let buffer: SharedFrameBuffer;

    beforeEach(() => {
      buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();
    });

    describe('markProcessing', () => {
      it('should set control to PROCESSING', () => {
        const sharedBuffer = buffer.getBuffers()[0];
        SharedFrameBuffer.markProcessing(sharedBuffer);

        const controlView = new Int32Array(sharedBuffer, 0, 1);
        expect(Atomics.load(controlView, 0)).toBe(BUFFER_CONTROL_PROCESSING);
      });
    });

    describe('markEmpty', () => {
      it('should set control to EMPTY', () => {
        const sharedBuffer = buffer.getBuffers()[0];

        // First mark as filled
        const controlView = new Int32Array(sharedBuffer, 0, 1);
        Atomics.store(controlView, 0, BUFFER_CONTROL_FILLED);

        // Then mark as empty
        SharedFrameBuffer.markEmpty(sharedBuffer);
        expect(Atomics.load(controlView, 0)).toBe(BUFFER_CONTROL_EMPTY);
      });
    });

    describe('isBufferFilled', () => {
      it('should return true when buffer is filled', () => {
        const sharedBuffer = buffer.getBuffers()[0];
        const controlView = new Int32Array(sharedBuffer, 0, 1);
        Atomics.store(controlView, 0, BUFFER_CONTROL_FILLED);

        expect(SharedFrameBuffer.isBufferFilled(sharedBuffer)).toBe(true);
      });

      it('should return false when buffer is empty', () => {
        const sharedBuffer = buffer.getBuffers()[0];
        expect(SharedFrameBuffer.isBufferFilled(sharedBuffer)).toBe(false);
      });

      it('should return false when buffer is processing', () => {
        const sharedBuffer = buffer.getBuffers()[0];
        SharedFrameBuffer.markProcessing(sharedBuffer);
        expect(SharedFrameBuffer.isBufferFilled(sharedBuffer)).toBe(false);
      });
    });

    describe('getFrameData', () => {
      it('should return correct slice of buffer', () => {
        const sharedBuffer = buffer.getBuffers()[0];
        const frameData = SharedFrameBuffer.getFrameData(sharedBuffer, WIDTH, HEIGHT);

        expect(frameData).toBeInstanceOf(Uint8ClampedArray);
        expect(frameData.length).toBe(WIDTH * HEIGHT * 4);
      });

      it('should return data starting at correct offset', () => {
        const sharedBuffer = buffer.getBuffers()[0];

        // Write some data
        const data = new Uint8ClampedArray(WIDTH * HEIGHT * 4);
        data[0] = 42;
        data[1] = 128;
        data[2] = 255;
        data[3] = 100;

        buffer.writeFrame(data);

        const frameData = SharedFrameBuffer.getFrameData(
          buffer.getBuffers()[0],
          WIDTH,
          HEIGHT
        );

        expect(frameData[0]).toBe(42);
        expect(frameData[1]).toBe(128);
        expect(frameData[2]).toBe(255);
        expect(frameData[3]).toBe(100);
      });
    });
  });

  describe('getCurrentWriteIndex', () => {
    it('should start at 0', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();
      expect(buffer.getCurrentWriteIndex()).toBe(0);
    });

    it('should update after successful write', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();

      const data = new Uint8ClampedArray(WIDTH * HEIGHT * 4);
      buffer.writeFrame(data);

      expect(buffer.getCurrentWriteIndex()).toBe(1);
    });
  });

  describe('destroy', () => {
    it('should reset to uninitialized state', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();
      expect(buffer.isInitialized()).toBe(true);

      buffer.destroy();
      expect(buffer.isInitialized()).toBe(false);
    });

    it('should allow re-initialization after destroy', () => {
      const buffer = new SharedFrameBuffer(WIDTH, HEIGHT);
      buffer.init();
      buffer.destroy();

      expect(() => buffer.init()).not.toThrow();
      expect(buffer.isInitialized()).toBe(true);
    });
  });
});

describe('TransferableFrameBuffer', () => {
  const WIDTH = 640;
  const HEIGHT = 480;

  describe('constructor', () => {
    it('should create instance with specified dimensions', () => {
      const buffer = new TransferableFrameBuffer(WIDTH, HEIGHT);
      expect(buffer.getDimensions()).toEqual({ width: WIDTH, height: HEIGHT });
    });
  });

  describe('createTransferable', () => {
    it('should create ArrayBuffer from Uint8ClampedArray', () => {
      const buffer = new TransferableFrameBuffer(WIDTH, HEIGHT);
      const data = new Uint8ClampedArray(WIDTH * HEIGHT * 4);
      data.fill(128);

      const transferable = buffer.createTransferable(data);

      expect(transferable).toBeInstanceOf(ArrayBuffer);
      expect(transferable.byteLength).toBe(WIDTH * HEIGHT * 4);
    });

    it('should create a copy of the data', () => {
      const buffer = new TransferableFrameBuffer(WIDTH, HEIGHT);
      const data = new Uint8ClampedArray(WIDTH * HEIGHT * 4);
      data[0] = 42;
      data[1] = 128;

      const transferable = buffer.createTransferable(data);
      const view = new Uint8ClampedArray(transferable);

      expect(view[0]).toBe(42);
      expect(view[1]).toBe(128);

      // Modify original - copy should be unaffected
      data[0] = 0;
      expect(view[0]).toBe(42);
    });

    it('should handle ImageData objects', () => {
      const buffer = new TransferableFrameBuffer(WIDTH, HEIGHT);

      // Create real ImageData using the global mock
      const data = new Uint8ClampedArray(WIDTH * HEIGHT * 4);
      data.fill(200);
      const imageData = new ImageData(data, WIDTH, HEIGHT);

      const transferable = buffer.createTransferable(imageData);
      const view = new Uint8ClampedArray(transferable);

      expect(view[0]).toBe(200);
    });
  });

  describe('destroy', () => {
    it('should not throw', () => {
      const buffer = new TransferableFrameBuffer(WIDTH, HEIGHT);
      expect(() => buffer.destroy()).not.toThrow();
    });
  });
});
