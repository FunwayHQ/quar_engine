let wasm;

function addToExternrefTable0(obj) {
    const idx = wasm.__externref_table_alloc();
    wasm.__wbindgen_externrefs.set(idx, obj);
    return idx;
}

function getArrayF32FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat32ArrayMemory0().subarray(ptr / 4, ptr / 4 + len);
}

function getArrayU8FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getUint8ArrayMemory0().subarray(ptr / 1, ptr / 1 + len);
}

let cachedDataViewMemory0 = null;
function getDataViewMemory0() {
    if (cachedDataViewMemory0 === null || cachedDataViewMemory0.buffer.detached === true || (cachedDataViewMemory0.buffer.detached === undefined && cachedDataViewMemory0.buffer !== wasm.memory.buffer)) {
        cachedDataViewMemory0 = new DataView(wasm.memory.buffer);
    }
    return cachedDataViewMemory0;
}

let cachedFloat32ArrayMemory0 = null;
function getFloat32ArrayMemory0() {
    if (cachedFloat32ArrayMemory0 === null || cachedFloat32ArrayMemory0.byteLength === 0) {
        cachedFloat32ArrayMemory0 = new Float32Array(wasm.memory.buffer);
    }
    return cachedFloat32ArrayMemory0;
}

function getStringFromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return decodeText(ptr, len);
}

let cachedUint8ArrayMemory0 = null;
function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

function handleError(f, args) {
    try {
        return f.apply(this, args);
    } catch (e) {
        const idx = addToExternrefTable0(e);
        wasm.__wbindgen_exn_store(idx);
    }
}

function isLikeNone(x) {
    return x === undefined || x === null;
}

function passArray8ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 1, 1) >>> 0;
    getUint8ArrayMemory0().set(arg, ptr / 1);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

function passStringToWasm0(arg, malloc, realloc) {
    if (realloc === undefined) {
        const buf = cachedTextEncoder.encode(arg);
        const ptr = malloc(buf.length, 1) >>> 0;
        getUint8ArrayMemory0().subarray(ptr, ptr + buf.length).set(buf);
        WASM_VECTOR_LEN = buf.length;
        return ptr;
    }

    let len = arg.length;
    let ptr = malloc(len, 1) >>> 0;

    const mem = getUint8ArrayMemory0();

    let offset = 0;

    for (; offset < len; offset++) {
        const code = arg.charCodeAt(offset);
        if (code > 0x7F) break;
        mem[ptr + offset] = code;
    }
    if (offset !== len) {
        if (offset !== 0) {
            arg = arg.slice(offset);
        }
        ptr = realloc(ptr, len, len = offset + arg.length * 3, 1) >>> 0;
        const view = getUint8ArrayMemory0().subarray(ptr + offset, ptr + len);
        const ret = cachedTextEncoder.encodeInto(arg, view);

        offset += ret.written;
        ptr = realloc(ptr, len, offset, 1) >>> 0;
    }

    WASM_VECTOR_LEN = offset;
    return ptr;
}

let cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
cachedTextDecoder.decode();
const MAX_SAFARI_DECODE_BYTES = 2146435072;
let numBytesDecoded = 0;
function decodeText(ptr, len) {
    numBytesDecoded += len;
    if (numBytesDecoded >= MAX_SAFARI_DECODE_BYTES) {
        cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
        cachedTextDecoder.decode();
        numBytesDecoded = len;
    }
    return cachedTextDecoder.decode(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}

const cachedTextEncoder = new TextEncoder();

if (!('encodeInto' in cachedTextEncoder)) {
    cachedTextEncoder.encodeInto = function (arg, view) {
        const buf = cachedTextEncoder.encode(arg);
        view.set(buf);
        return {
            read: arg.length,
            written: buf.length
        };
    }
}

let WASM_VECTOR_LEN = 0;

const EngineConfigFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_engineconfig_free(ptr >>> 0, 1));

const Pose3DFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_pose3d_free(ptr >>> 0, 1));

const TrackerHandleFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_trackerhandle_free(ptr >>> 0, 1));

/**
 * Engine configuration options passed from JavaScript.
 */
export class EngineConfig {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        EngineConfigFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_engineconfig_free(ptr, 0);
    }
    /**
     * Get the target FPS.
     * @returns {number}
     */
    get target_fps() {
        const ret = wasm.engineconfig_target_fps(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Set the target FPS (30 or 60).
     * @param {number} fps
     */
    set target_fps(fps) {
        wasm.engineconfig_set_target_fps(this.__wbg_ptr, fps);
    }
    /**
     * Check if adaptive quality is enabled.
     * @returns {boolean}
     */
    get adaptive_quality() {
        const ret = wasm.engineconfig_adaptive_quality(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * Enable or disable adaptive quality.
     * @param {boolean} enabled
     */
    set adaptive_quality(enabled) {
        wasm.engineconfig_set_adaptive_quality(this.__wbg_ptr, enabled);
    }
    /**
     * Create a new engine configuration with default values.
     */
    constructor() {
        const ret = wasm.engineconfig_new();
        this.__wbg_ptr = ret >>> 0;
        EngineConfigFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * Check if debug mode is enabled.
     * @returns {boolean}
     */
    get debug() {
        const ret = wasm.engineconfig_debug(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * Enable or disable debug mode.
     * @param {boolean} enabled
     */
    set debug(enabled) {
        wasm.engineconfig_set_debug(this.__wbg_ptr, enabled);
    }
}
if (Symbol.dispose) EngineConfig.prototype[Symbol.dispose] = EngineConfig.prototype.free;

/**
 * Pose3D represents a 6DoF pose (position + rotation).
 * Used to communicate tracking results back to JavaScript.
 */
export class Pose3D {
    static __wrap(ptr) {
        ptr = ptr >>> 0;
        const obj = Object.create(Pose3D.prototype);
        obj.__wbg_ptr = ptr;
        Pose3DFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        Pose3DFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_pose3d_free(ptr, 0);
    }
    /**
     * X position in meters
     * @returns {number}
     */
    get x() {
        const ret = wasm.__wbg_get_pose3d_x(this.__wbg_ptr);
        return ret;
    }
    /**
     * X position in meters
     * @param {number} arg0
     */
    set x(arg0) {
        wasm.__wbg_set_pose3d_x(this.__wbg_ptr, arg0);
    }
    /**
     * Y position in meters
     * @returns {number}
     */
    get y() {
        const ret = wasm.__wbg_get_pose3d_y(this.__wbg_ptr);
        return ret;
    }
    /**
     * Y position in meters
     * @param {number} arg0
     */
    set y(arg0) {
        wasm.__wbg_set_pose3d_y(this.__wbg_ptr, arg0);
    }
    /**
     * Z position in meters
     * @returns {number}
     */
    get z() {
        const ret = wasm.__wbg_get_pose3d_z(this.__wbg_ptr);
        return ret;
    }
    /**
     * Z position in meters
     * @param {number} arg0
     */
    set z(arg0) {
        wasm.__wbg_set_pose3d_z(this.__wbg_ptr, arg0);
    }
    /**
     * Quaternion X component
     * @returns {number}
     */
    get qx() {
        const ret = wasm.__wbg_get_pose3d_qx(this.__wbg_ptr);
        return ret;
    }
    /**
     * Quaternion X component
     * @param {number} arg0
     */
    set qx(arg0) {
        wasm.__wbg_set_pose3d_qx(this.__wbg_ptr, arg0);
    }
    /**
     * Quaternion Y component
     * @returns {number}
     */
    get qy() {
        const ret = wasm.__wbg_get_pose3d_qy(this.__wbg_ptr);
        return ret;
    }
    /**
     * Quaternion Y component
     * @param {number} arg0
     */
    set qy(arg0) {
        wasm.__wbg_set_pose3d_qy(this.__wbg_ptr, arg0);
    }
    /**
     * Quaternion Z component
     * @returns {number}
     */
    get qz() {
        const ret = wasm.__wbg_get_pose3d_qz(this.__wbg_ptr);
        return ret;
    }
    /**
     * Quaternion Z component
     * @param {number} arg0
     */
    set qz(arg0) {
        wasm.__wbg_set_pose3d_qz(this.__wbg_ptr, arg0);
    }
    /**
     * Quaternion W component
     * @returns {number}
     */
    get qw() {
        const ret = wasm.__wbg_get_pose3d_qw(this.__wbg_ptr);
        return ret;
    }
    /**
     * Quaternion W component
     * @param {number} arg0
     */
    set qw(arg0) {
        wasm.__wbg_set_pose3d_qw(this.__wbg_ptr, arg0);
    }
    /**
     * Get the rotation as a JavaScript array [qx, qy, qz, qw].
     * @returns {Float32Array}
     */
    quaternion() {
        const ret = wasm.pose3d_quaternion(this.__wbg_ptr);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Convert to a 4x4 transformation matrix in column-major order.
     * @returns {Float32Array}
     */
    to_matrix4() {
        const ret = wasm.pose3d_to_matrix4(this.__wbg_ptr);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Create a pose from position and quaternion components.
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @param {number} qx
     * @param {number} qy
     * @param {number} qz
     * @param {number} qw
     * @returns {Pose3D}
     */
    static from_components(x, y, z, qx, qy, qz, qw) {
        const ret = wasm.pose3d_from_components(x, y, z, qx, qy, qz, qw);
        return Pose3D.__wrap(ret);
    }
    /**
     * Create a new identity pose (no rotation, at origin).
     */
    constructor() {
        const ret = wasm.pose3d_new();
        this.__wbg_ptr = ret >>> 0;
        Pose3DFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * Get the position as a JavaScript array [x, y, z].
     * @returns {Float32Array}
     */
    position() {
        const ret = wasm.pose3d_position(this.__wbg_ptr);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
}
if (Symbol.dispose) Pose3D.prototype[Symbol.dispose] = Pose3D.prototype.free;

/**
 * Opaque handle to a tracker instance.
 */
export class TrackerHandle {
    static __wrap(ptr) {
        ptr = ptr >>> 0;
        const obj = Object.create(TrackerHandle.prototype);
        obj.__wbg_ptr = ptr;
        TrackerHandleFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        TrackerHandleFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_trackerhandle_free(ptr, 0);
    }
    /**
     * Create a tracker with custom configuration.
     * @param {number} window_size
     * @param {number} pyramid_levels
     * @param {number} fast_threshold
     * @param {number} max_features
     * @returns {TrackerHandle}
     */
    static with_config(window_size, pyramid_levels, fast_threshold, max_features) {
        const ret = wasm.trackerhandle_with_config(window_size, pyramid_levels, fast_threshold, max_features);
        return TrackerHandle.__wrap(ret);
    }
    /**
     * Process a frame and return the pose as JSON.
     * @param {Uint8Array} rgba
     * @param {number} width
     * @param {number} height
     * @returns {any}
     */
    process_frame(rgba, width, height) {
        const ptr0 = passArray8ToWasm0(rgba, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.trackerhandle_process_frame(this.__wbg_ptr, ptr0, len0, width, height);
        return ret;
    }
    /**
     * Get the number of tracked points.
     * @returns {number}
     */
    tracked_points() {
        const ret = wasm.trackerhandle_tracked_points(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Create a new tracker.
     */
    constructor() {
        const ret = wasm.trackerhandle_new();
        this.__wbg_ptr = ret >>> 0;
        TrackerHandleFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * Reset the tracker.
     */
    reset() {
        wasm.trackerhandle_reset(this.__wbg_ptr);
    }
    /**
     * Get the current pose as JSON.
     * @returns {any}
     */
    get_pose() {
        const ret = wasm.trackerhandle_get_pose(this.__wbg_ptr);
        return ret;
    }
}
if (Symbol.dispose) TrackerHandle.prototype[Symbol.dispose] = TrackerHandle.prototype.free;

/**
 * Count the number of features detected (without returning full keypoint data).
 * Useful for quick feature density checks.
 * @param {Uint8Array} rgba_data
 * @param {number} width
 * @param {number} height
 * @param {number} threshold
 * @returns {number}
 */
export function count_features(rgba_data, width, height, threshold) {
    const ptr0 = passArray8ToWasm0(rgba_data, wasm.__wbindgen_malloc);
    const len0 = WASM_VECTOR_LEN;
    const ret = wasm.count_features(ptr0, len0, width, height, threshold);
    return ret >>> 0;
}

/**
 * Detect FAST corners in an RGBA image.
 *
 * # Arguments
 * * `rgba_data` - RGBA pixel data as a flat array (4 bytes per pixel)
 * * `width` - Image width in pixels
 * * `height` - Image height in pixels
 * * `threshold` - Intensity difference threshold (typically 20-50)
 *
 * # Returns
 * A JsValue containing a JSON array of keypoints with x, y, and score.
 * @param {Uint8Array} rgba_data
 * @param {number} width
 * @param {number} height
 * @param {number} threshold
 * @returns {any}
 */
export function detect_features(rgba_data, width, height, threshold) {
    const ptr0 = passArray8ToWasm0(rgba_data, wasm.__wbindgen_malloc);
    const len0 = WASM_VECTOR_LEN;
    const ret = wasm.detect_features(ptr0, len0, width, height, threshold);
    return ret;
}

/**
 * Detect FAST corners with custom NMS radius.
 *
 * # Arguments
 * * `rgba_data` - RGBA pixel data
 * * `width` - Image width
 * * `height` - Image height
 * * `threshold` - Intensity difference threshold
 * * `nms_radius` - Non-maximum suppression radius in pixels
 *
 * # Returns
 * A JsValue containing a JSON array of keypoints.
 * @param {Uint8Array} rgba_data
 * @param {number} width
 * @param {number} height
 * @param {number} threshold
 * @param {number} nms_radius
 * @returns {any}
 */
export function detect_features_advanced(rgba_data, width, height, threshold, nms_radius) {
    const ptr0 = passArray8ToWasm0(rgba_data, wasm.__wbindgen_malloc);
    const len0 = WASM_VECTOR_LEN;
    const ret = wasm.detect_features_advanced(ptr0, len0, width, height, threshold, nms_radius);
    return ret;
}

/**
 * Log an error message to the browser console.
 * @param {string} message
 */
export function error(message) {
    const ptr0 = passStringToWasm0(message, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
    const len0 = WASM_VECTOR_LEN;
    wasm.error(ptr0, len0);
}

/**
 * Get the grayscale version of an RGBA image.
 * Useful for debugging or visualization.
 * @param {Uint8Array} rgba_data
 * @returns {Uint8Array}
 */
export function get_grayscale(rgba_data) {
    const ptr0 = passArray8ToWasm0(rgba_data, wasm.__wbindgen_malloc);
    const len0 = WASM_VECTOR_LEN;
    const ret = wasm.get_grayscale(ptr0, len0);
    var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
    wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
    return v2;
}

/**
 * Get the current high-resolution timestamp from the browser's Performance API.
 * Returns milliseconds since the page was loaded.
 * @returns {number}
 */
export function get_performance_now() {
    const ret = wasm.get_performance_now();
    return ret;
}

/**
 * A simple greeting function to verify WASM integration is working.
 * This function logs to the browser console and returns a greeting message.
 * @param {string} name
 * @returns {string}
 */
export function greet(name) {
    let deferred2_0;
    let deferred2_1;
    try {
        const ptr0 = passStringToWasm0(name, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.greet(ptr0, len0);
        deferred2_0 = ret[0];
        deferred2_1 = ret[1];
        return getStringFromWasm0(ret[0], ret[1]);
    } finally {
        wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
    }
}

/**
 * Initialize the WASM module with panic hook for better error messages.
 * This function is automatically called when the WASM module is loaded.
 */
export function init() {
    wasm.init();
}

/**
 * Log a message to the browser console.
 * @param {string} message
 */
export function log(message) {
    const ptr0 = passStringToWasm0(message, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
    const len0 = WASM_VECTOR_LEN;
    wasm.log(ptr0, len0);
}

/**
 * Returns the version of the Aether engine.
 * @returns {string}
 */
export function version() {
    let deferred1_0;
    let deferred1_1;
    try {
        const ret = wasm.version();
        deferred1_0 = ret[0];
        deferred1_1 = ret[1];
        return getStringFromWasm0(ret[0], ret[1]);
    } finally {
        wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
    }
}

/**
 * Log a warning message to the browser console.
 * @param {string} message
 */
export function warn(message) {
    const ptr0 = passStringToWasm0(message, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
    const len0 = WASM_VECTOR_LEN;
    wasm.warn(ptr0, len0);
}

const EXPECTED_RESPONSE_TYPES = new Set(['basic', 'cors', 'default']);

async function __wbg_load(module, imports) {
    if (typeof Response === 'function' && module instanceof Response) {
        if (typeof WebAssembly.instantiateStreaming === 'function') {
            try {
                return await WebAssembly.instantiateStreaming(module, imports);
            } catch (e) {
                const validResponse = module.ok && EXPECTED_RESPONSE_TYPES.has(module.type);

                if (validResponse && module.headers.get('Content-Type') !== 'application/wasm') {
                    console.warn("`WebAssembly.instantiateStreaming` failed because your server does not serve Wasm with `application/wasm` MIME type. Falling back to `WebAssembly.instantiate` which is slower. Original error:\n", e);

                } else {
                    throw e;
                }
            }
        }

        const bytes = await module.arrayBuffer();
        return await WebAssembly.instantiate(bytes, imports);
    } else {
        const instance = await WebAssembly.instantiate(module, imports);

        if (instance instanceof WebAssembly.Instance) {
            return { instance, module };
        } else {
            return instance;
        }
    }
}

function __wbg_get_imports() {
    const imports = {};
    imports.wbg = {};
    imports.wbg.__wbg___wbindgen_is_undefined_f6b95eab589e0269 = function(arg0) {
        const ret = arg0 === undefined;
        return ret;
    };
    imports.wbg.__wbg___wbindgen_throw_dd24417ed36fc46e = function(arg0, arg1) {
        throw new Error(getStringFromWasm0(arg0, arg1));
    };
    imports.wbg.__wbg_call_abb4ff46ce38be40 = function() { return handleError(function (arg0, arg1) {
        const ret = arg0.call(arg1);
        return ret;
    }, arguments) };
    imports.wbg.__wbg_error_7534b8e9a36f1ab4 = function(arg0, arg1) {
        let deferred0_0;
        let deferred0_1;
        try {
            deferred0_0 = arg0;
            deferred0_1 = arg1;
            console.error(getStringFromWasm0(arg0, arg1));
        } finally {
            wasm.__wbindgen_free(deferred0_0, deferred0_1, 1);
        }
    };
    imports.wbg.__wbg_error_7bc7d576a6aaf855 = function(arg0) {
        console.error(arg0);
    };
    imports.wbg.__wbg_instanceof_Window_b5cf7783caa68180 = function(arg0) {
        let result;
        try {
            result = arg0 instanceof Window;
        } catch (_) {
            result = false;
        }
        const ret = result;
        return ret;
    };
    imports.wbg.__wbg_log_1d990106d99dacb7 = function(arg0) {
        console.log(arg0);
    };
    imports.wbg.__wbg_new_1ba21ce319a06297 = function() {
        const ret = new Object();
        return ret;
    };
    imports.wbg.__wbg_new_25f239778d6112b9 = function() {
        const ret = new Array();
        return ret;
    };
    imports.wbg.__wbg_new_8a6f238a6ece86ea = function() {
        const ret = new Error();
        return ret;
    };
    imports.wbg.__wbg_new_no_args_cb138f77cf6151ee = function(arg0, arg1) {
        const ret = new Function(getStringFromWasm0(arg0, arg1));
        return ret;
    };
    imports.wbg.__wbg_now_8cf15d6e317793e1 = function(arg0) {
        const ret = arg0.now();
        return ret;
    };
    imports.wbg.__wbg_performance_c77a440eff2efd9b = function(arg0) {
        const ret = arg0.performance;
        return isLikeNone(ret) ? 0 : addToExternrefTable0(ret);
    };
    imports.wbg.__wbg_set_3f1d0b984ed272ed = function(arg0, arg1, arg2) {
        arg0[arg1] = arg2;
    };
    imports.wbg.__wbg_set_7df433eea03a5c14 = function(arg0, arg1, arg2) {
        arg0[arg1 >>> 0] = arg2;
    };
    imports.wbg.__wbg_stack_0ed75d68575b0f3c = function(arg0, arg1) {
        const ret = arg1.stack;
        const ptr1 = passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len1 = WASM_VECTOR_LEN;
        getDataViewMemory0().setInt32(arg0 + 4 * 1, len1, true);
        getDataViewMemory0().setInt32(arg0 + 4 * 0, ptr1, true);
    };
    imports.wbg.__wbg_static_accessor_GLOBAL_769e6b65d6557335 = function() {
        const ret = typeof global === 'undefined' ? null : global;
        return isLikeNone(ret) ? 0 : addToExternrefTable0(ret);
    };
    imports.wbg.__wbg_static_accessor_GLOBAL_THIS_60cf02db4de8e1c1 = function() {
        const ret = typeof globalThis === 'undefined' ? null : globalThis;
        return isLikeNone(ret) ? 0 : addToExternrefTable0(ret);
    };
    imports.wbg.__wbg_static_accessor_SELF_08f5a74c69739274 = function() {
        const ret = typeof self === 'undefined' ? null : self;
        return isLikeNone(ret) ? 0 : addToExternrefTable0(ret);
    };
    imports.wbg.__wbg_static_accessor_WINDOW_a8924b26aa92d024 = function() {
        const ret = typeof window === 'undefined' ? null : window;
        return isLikeNone(ret) ? 0 : addToExternrefTable0(ret);
    };
    imports.wbg.__wbg_warn_6e567d0d926ff881 = function(arg0) {
        console.warn(arg0);
    };
    imports.wbg.__wbindgen_cast_2241b6af4c4b2941 = function(arg0, arg1) {
        // Cast intrinsic for `Ref(String) -> Externref`.
        const ret = getStringFromWasm0(arg0, arg1);
        return ret;
    };
    imports.wbg.__wbindgen_cast_d6cd19b81560fd6e = function(arg0) {
        // Cast intrinsic for `F64 -> Externref`.
        const ret = arg0;
        return ret;
    };
    imports.wbg.__wbindgen_init_externref_table = function() {
        const table = wasm.__wbindgen_externrefs;
        const offset = table.grow(4);
        table.set(0, undefined);
        table.set(offset + 0, undefined);
        table.set(offset + 1, null);
        table.set(offset + 2, true);
        table.set(offset + 3, false);
    };

    return imports;
}

function __wbg_finalize_init(instance, module) {
    wasm = instance.exports;
    __wbg_init.__wbindgen_wasm_module = module;
    cachedDataViewMemory0 = null;
    cachedFloat32ArrayMemory0 = null;
    cachedUint8ArrayMemory0 = null;


    wasm.__wbindgen_start();
    return wasm;
}

function initSync(module) {
    if (wasm !== undefined) return wasm;


    if (typeof module !== 'undefined') {
        if (Object.getPrototypeOf(module) === Object.prototype) {
            ({module} = module)
        } else {
            console.warn('using deprecated parameters for `initSync()`; pass a single object instead')
        }
    }

    const imports = __wbg_get_imports();
    if (!(module instanceof WebAssembly.Module)) {
        module = new WebAssembly.Module(module);
    }
    const instance = new WebAssembly.Instance(module, imports);
    return __wbg_finalize_init(instance, module);
}

async function __wbg_init(module_or_path) {
    if (wasm !== undefined) return wasm;


    if (typeof module_or_path !== 'undefined') {
        if (Object.getPrototypeOf(module_or_path) === Object.prototype) {
            ({module_or_path} = module_or_path)
        } else {
            console.warn('using deprecated parameters for the initialization function; pass a single object instead')
        }
    }

    if (typeof module_or_path === 'undefined') {
        module_or_path = new URL('quar_engine_bg.wasm', import.meta.url);
    }
    const imports = __wbg_get_imports();

    if (typeof module_or_path === 'string' || (typeof Request === 'function' && module_or_path instanceof Request) || (typeof URL === 'function' && module_or_path instanceof URL)) {
        module_or_path = fetch(module_or_path);
    }

    const { instance, module } = await __wbg_load(await module_or_path, imports);

    return __wbg_finalize_init(instance, module);
}

export { initSync };
export default __wbg_init;
